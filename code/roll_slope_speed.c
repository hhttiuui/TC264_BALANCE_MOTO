/*
 * roll_slope_speed.c
 *
 *  Created on: 2026-05-24
 *      Author: hhttiuui
 *
 *  Roll-based slope speed helper (5ms task).
 */

#include "zf_common_headfile.h"
#include "roll_slope_speed.h"
#include "motor_controller.h"
#include "encoder_count.h"

// roll angle (deg) provided by IMU display variable
extern float show_roll_deg;

#define ROLL_SLOPE_COUNTS_PER_METER (18500.0f)

static roll_slope_speed_cfg_t g_roll_slope_cfg;
static uint8 s_roll_slope_enabled = 0u;
static uint8 s_roll_slope_started = 0u;
static uint8 s_roll_slope_done = 0u;
static uint8 s_roll_slope_restore_pending = 0u;
static int64 s_flat_start_counts = 0;
typedef enum
{
    ROLL_SLOPE_FLAT = 0,
    ROLL_SLOPE_PRE_UP_DECEL,
    ROLL_SLOPE_UP,
    ROLL_SLOPE_FLAT_AFTER_UP,
    ROLL_SLOPE_DOWN,
} roll_slope_state_t;

static roll_slope_state_t s_roll_state = ROLL_SLOPE_FLAT;

void roll_slope_speed_init(void)
{
    g_roll_slope_cfg.pre_up_decel_m = 9.5f;
    g_roll_slope_cfg.up_start_deg = 19.0f;
    g_roll_slope_cfg.up_end_deg = 15.0f;
    g_roll_slope_cfg.down_start_deg = 8.0f;
    g_roll_slope_cfg.down_end_deg = 1.0f;
    g_roll_slope_cfg.up_delta_counts = 200;
    g_roll_slope_cfg.pre_up_decel_counts = -60;
    g_roll_slope_cfg.crest_delta_counts = -130;
    g_roll_slope_cfg.flat_delta_counts = 0;
    s_roll_state = ROLL_SLOPE_FLAT;
    s_roll_slope_enabled = 0u;
    s_roll_slope_started = 0u;
    s_roll_slope_done = 0u;
    s_roll_slope_restore_pending = 0u;
    s_flat_start_counts = encoder_get_demo5_count_total();
}

const roll_slope_speed_cfg_t *roll_slope_speed_get_cfg(void)
{
    return &g_roll_slope_cfg;
}

void roll_slope_speed_set_cfg(const roll_slope_speed_cfg_t *cfg)
{
    if (!cfg)
    {
        return;
    }
    g_roll_slope_cfg = *cfg;
}

uint8 roll_slope_speed_get_enabled(void)
{
    return s_roll_slope_enabled;
}

void roll_slope_speed_set_enabled(uint8 enabled)
{
    s_roll_slope_enabled = (enabled != 0u) ? 1u : 0u;
}

uint8 roll_slope_speed_get_started(void)
{
    return s_roll_slope_started;
}

uint8 roll_slope_speed_get_done(void)
{
    return s_roll_slope_done;
}

void roll_slope_speed_start(void)
{
    if (s_roll_slope_started)
    {
        return;
    }
    s_roll_slope_started = 1u;
    s_roll_slope_done = 0u;
    s_roll_slope_restore_pending = 0u;
    s_roll_state = ROLL_SLOPE_FLAT;
    s_flat_start_counts = encoder_get_demo5_count_total();
}

void roll_slope_speed_reset(void)
{
    s_roll_slope_started = 0u;
    s_roll_slope_done = 0u;
    s_roll_slope_restore_pending = 0u;
    s_roll_state = ROLL_SLOPE_FLAT;
    s_flat_start_counts = encoder_get_demo5_count_total();
}

void roll_slope_speed_task_5ms(void)
{
    if (!s_roll_slope_enabled)
    {
        return;
    }
    if (!s_roll_slope_started)
    {
        return;
    }
    if (s_roll_slope_restore_pending)
    {
        pitch_speed_change(0);
        s_roll_slope_restore_pending = 0u;
        return;
    }
    if (s_roll_slope_done)
    {
        return;
    }
    // IMU roll uses 180 deg as zero; convert to signed delta with wrap.
    float roll = 180.0f - show_roll_deg;
    int64 now_counts = encoder_get_demo5_count_total();

    if (roll > 180.0f)
    {
        roll -= 360.0f;
    }
    else if (roll < -180.0f)
    {
        roll += 360.0f;
    }

    if (now_counts < s_flat_start_counts)
    {
        s_flat_start_counts = now_counts;
    }

    switch (s_roll_state)
    {
    case ROLL_SLOPE_FLAT:
        if (g_roll_slope_cfg.pre_up_decel_m < 1.0f)
        {
            if (roll >= g_roll_slope_cfg.up_start_deg)
            {
                s_roll_state = ROLL_SLOPE_UP;
            }
        }
        else
        {
            int64 threshold_counts = (int64)(g_roll_slope_cfg.pre_up_decel_m * ROLL_SLOPE_COUNTS_PER_METER + 0.5f);
            if ((now_counts - s_flat_start_counts) >= threshold_counts)
            {
                s_roll_state = ROLL_SLOPE_PRE_UP_DECEL;
            }
        }
        pitch_speed_change(g_roll_slope_cfg.flat_delta_counts);
        break;

    case ROLL_SLOPE_PRE_UP_DECEL:
        if (roll >= g_roll_slope_cfg.up_start_deg)
        {
            s_roll_state = ROLL_SLOPE_UP;
        }
        pitch_speed_change(g_roll_slope_cfg.pre_up_decel_counts);
        break;

    case ROLL_SLOPE_UP:
        if (roll <= g_roll_slope_cfg.up_end_deg)
        {
            s_roll_state = ROLL_SLOPE_FLAT_AFTER_UP;
        }
        pitch_speed_change(g_roll_slope_cfg.up_delta_counts);
        break;

    case ROLL_SLOPE_FLAT_AFTER_UP:
        if (roll <= -g_roll_slope_cfg.down_start_deg)
        {
            s_roll_state = ROLL_SLOPE_DOWN;
        }
        else if (roll >= g_roll_slope_cfg.up_start_deg)
        {
            s_roll_state = ROLL_SLOPE_UP;
        }
        pitch_speed_change(g_roll_slope_cfg.crest_delta_counts);
        break;

    case ROLL_SLOPE_DOWN:
        if (roll >= -g_roll_slope_cfg.down_end_deg)
        {
            s_roll_state = ROLL_SLOPE_FLAT;
            s_flat_start_counts = now_counts;
            s_roll_slope_done = 1u;
            s_roll_slope_restore_pending = 1u;
        }
        pitch_speed_change(g_roll_slope_cfg.crest_delta_counts);
        break;

    default:
        s_roll_state = ROLL_SLOPE_FLAT;
        s_flat_start_counts = now_counts;
        pitch_speed_change(g_roll_slope_cfg.flat_delta_counts);
        break;
    }
}
