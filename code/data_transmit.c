/*
 * data_transmit.c
 *
 *  Created on: 2026-05-15
 *      Author: GitHub Copilot
 *
 *  Simple data transmit helpers (UART0 or WiFi-SPI).
 */

#include "zf_common_headfile.h"
#include "zf_device_wifi_spi.h"
#include "common_flie.h"
#include "data_transmit.h"
#include <string.h>

// External IMU display values (updated in imu_task_handler_2ms).
extern float show_pitch_deg;
extern float show_yaw_deg;

#define DATA_TX_RECORD_MAX      (6000u)
#define DATA_TX_FRAME_MAX       (256u)

#define DATA_TX_MAGIC0          (0xA5u)
#define DATA_TX_MAGIC1          (0x5Au)

#define DATA_TX_MSG_PITCH_META  (0x10u)
#define DATA_TX_MSG_PITCH_DATA  (0x11u)
#define DATA_TX_MSG_YAW_META    (0x12u)
#define DATA_TX_MSG_YAW_DATA    (0x13u)
#define DATA_TX_MSG_PID_META    (0x20u)
#define DATA_TX_MSG_PID_DATA    (0x21u)

static data_transmit_output_t s_output = DATA_TX_UART0;

static float s_pitch_buf[DATA_TX_RECORD_MAX];
static float s_yaw_buf[DATA_TX_RECORD_MAX];
static uint16 s_record_count = 0u;
static uint16 s_record_divider = 1u;
static uint16 s_record_div_cnt = 0u;
static uint8 s_recording = 0u;

static void data_tx_send_bytes(const uint8 *buf, uint16 len)
{
    if (s_output == DATA_TX_UART0)
    {
        uart_write_buffer(UART_0, buf, len);
    }
    else
    {
        (void)wifi_spi_send_buffer(buf, len);
    }
}

static uint8 data_tx_checksum(const uint8 *buf, uint16 start, uint16 end)
{
    uint8 sum = 0u;
    uint16 i;
    for (i = start; i < end; i++)
    {
        sum = (uint8)(sum + buf[i]);
    }
    return sum;
}

static void data_tx_frame(uint8 msg_id, const uint8 *payload, uint16 payload_len)
{
    uint8 frame[DATA_TX_FRAME_MAX];
    uint16 total_len = (uint16)(payload_len + 6u);

    if (total_len > DATA_TX_FRAME_MAX)
    {
        return;
    }

    frame[0] = DATA_TX_MAGIC0;
    frame[1] = DATA_TX_MAGIC1;
    frame[2] = msg_id;
    frame[3] = (uint8)(payload_len & 0xFFu);
    frame[4] = (uint8)((payload_len >> 8) & 0xFFu);

    if (payload_len > 0u)
    {
        memcpy(&frame[5], payload, payload_len);
    }

    frame[5 + payload_len] = data_tx_checksum(frame, 2u, (uint16)(5u + payload_len));
    data_tx_send_bytes(frame, total_len);
}

static void put_u16_le(uint8 *buf, uint16 *ofs, uint16 v)
{
    buf[(*ofs)++] = (uint8)(v & 0xFFu);
    buf[(*ofs)++] = (uint8)((v >> 8) & 0xFFu);
}

static void put_s16_le(uint8 *buf, uint16 *ofs, int16 v)
{
    put_u16_le(buf, ofs, (uint16)v);
}

static void put_f32_le(uint8 *buf, uint16 *ofs, float v)
{
    uint8 b[4];
    memcpy(b, &v, sizeof(float));
    buf[(*ofs)++] = b[0];
    buf[(*ofs)++] = b[1];
    buf[(*ofs)++] = b[2];
    buf[(*ofs)++] = b[3];
}

void data_transmit_set_output(data_transmit_output_t output)
{
    s_output = output;
}

void data_transmit_start_record(void)
{
    s_record_count = 0u;
    s_record_div_cnt = 0u;
    s_recording = 1u;
}

void data_transmit_stop_record(void)
{
    s_recording = 0u;
}

void data_transmit_reset_record(void)
{
    s_record_count = 0u;
    s_record_div_cnt = 0u;
}

uint8 data_transmit_is_recording(void)
{
    return s_recording;
}

void data_transmit_set_record_divider(uint16 divider)
{
    if (divider == 0u)
    {
        divider = 1u;
    }
    s_record_divider = divider;
    s_record_div_cnt = 0u;
}

void data_transmit_task_5ms(void)
{
    if (!s_recording)
    {
        return;
    }

    s_record_div_cnt++;
    if (s_record_div_cnt < s_record_divider)
    {
        return;
    }
    s_record_div_cnt = 0u;

    if (s_record_count >= DATA_TX_RECORD_MAX)
    {
        s_recording = 0u;
        return;
    }

    s_pitch_buf[s_record_count] = show_pitch_deg;
    s_yaw_buf[s_record_count] = show_yaw_deg;
    s_record_count++;
}

uint16 data_transmit_get_record_count(void)
{
    return s_record_count;
}

static void data_tx_send_series(uint8 msg_meta, uint8 msg_data, const float *buf, uint16 count)
{
    uint8 payload[DATA_TX_FRAME_MAX];
    uint16 ofs = 0u;
    uint16 sample_period_ms = (uint16)(5u * s_record_divider);
    uint16 start = 0u;

    put_u16_le(payload, &ofs, count);
    put_u16_le(payload, &ofs, sample_period_ms);
    data_tx_frame(msg_meta, payload, ofs);

    if (count == 0u)
    {
        return;
    }

    while (start < count)
    {
        uint16 payload_max = (uint16)(DATA_TX_FRAME_MAX - 6u);
        uint16 header_bytes = 4u;
        uint16 max_samples = (uint16)((payload_max - header_bytes) / 4u);
        uint16 remaining = (uint16)(count - start);
        uint16 chunk = (remaining < max_samples) ? remaining : max_samples;

        ofs = 0u;
        put_u16_le(payload, &ofs, start);
        put_u16_le(payload, &ofs, chunk);
        for (uint16 i = 0u; i < chunk; i++)
        {
            put_f32_le(payload, &ofs, buf[start + i]);
        }

        data_tx_frame(msg_data, payload, ofs);
        start = (uint16)(start + chunk);
    }
}

void data_transmit_send_pitch_history(void)
{
    data_tx_send_series(DATA_TX_MSG_PITCH_META, DATA_TX_MSG_PITCH_DATA, s_pitch_buf, s_record_count);
}

void data_transmit_send_yaw_history(void)
{
    data_tx_send_series(DATA_TX_MSG_YAW_META, DATA_TX_MSG_YAW_DATA, s_yaw_buf, s_record_count);
}

void data_transmit_send_pid_mode_cfg(void)
{
    uint8 payload[DATA_TX_FRAME_MAX];
    uint16 ofs = 0u;

    put_u16_le(payload, &ofs, (uint16)PID_MODE_CFG_COUNT);
    data_tx_frame(DATA_TX_MSG_PID_META, payload, ofs);

    for (uint16 i = 0u; i < (uint16)PID_MODE_CFG_COUNT; i++)
    {
        const pid_mode_cfg_t *cfg = &s_pid_mode_cfg[i];
        ofs = 0u;

        payload[ofs++] = (uint8)i;
        payload[ofs++] = (uint8)cfg->strategy;
        put_u16_le(payload, &ofs, 0u);

        put_f32_le(payload, &ofs, cfg->ahrs_kp);
        put_f32_le(payload, &ofs, cfg->ahrs_ki);
        put_f32_le(payload, &ofs, cfg->yaw_gate_off_abs_pitch_deg);
        put_f32_le(payload, &ofs, cfg->yaw_gate_on_abs_pitch_deg);

        put_f32_le(payload, &ofs, cfg->yaw.kp);
        put_f32_le(payload, &ofs, cfg->yaw.ki);
        put_f32_le(payload, &ofs, cfg->yaw.kd);
        put_f32_le(payload, &ofs, cfg->yaw.out_min);
        put_f32_le(payload, &ofs, cfg->yaw.out_max);
        put_f32_le(payload, &ofs, cfg->yaw.i_min);
        put_f32_le(payload, &ofs, cfg->yaw.i_max);

        put_f32_le(payload, &ofs, cfg->angle.kp);
        put_f32_le(payload, &ofs, cfg->angle.ki);
        put_f32_le(payload, &ofs, cfg->angle.kd);
        put_f32_le(payload, &ofs, cfg->angle.out_min);
        put_f32_le(payload, &ofs, cfg->angle.out_max);
        put_f32_le(payload, &ofs, cfg->angle.i_min);
        put_f32_le(payload, &ofs, cfg->angle.i_max);

        put_f32_le(payload, &ofs, cfg->rate.kp);
        put_f32_le(payload, &ofs, cfg->rate.ki);
        put_f32_le(payload, &ofs, cfg->rate.kd);
        put_f32_le(payload, &ofs, cfg->rate.out_min);
        put_f32_le(payload, &ofs, cfg->rate.out_max);
        put_f32_le(payload, &ofs, cfg->rate.i_min);
        put_f32_le(payload, &ofs, cfg->rate.i_max);

        put_f32_le(payload, &ofs, cfg->damp_k);
        put_f32_le(payload, &ofs, cfg->damp_max);
        put_f32_le(payload, &ofs, cfg->imu_filter_alpha);
        put_f32_le(payload, &ofs, cfg->servo_init_out);
        put_s16_le(payload, &ofs, cfg->motor_target_counts_5ms);
        put_u16_le(payload, &ofs, 0u);

        data_tx_frame(DATA_TX_MSG_PID_DATA, payload, ofs);
    }
}
