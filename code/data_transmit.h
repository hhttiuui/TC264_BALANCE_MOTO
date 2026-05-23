/*
 * data_transmit.h
 *
 *  Created on: 2026-05-15
 *      Author: GitHub Copilot
 *
 *  Simple data transmit helpers (UART0 or WiFi-SPI).
 */

#ifndef CODE_DATA_TRANSMIT_H_
#define CODE_DATA_TRANSMIT_H_

#include "zf_common_typedef.h"

typedef enum
{
    DATA_TX_UART0 = 0,
    DATA_TX_WIFI_SPI = 1,
} data_transmit_output_t;

void data_transmit_set_output(data_transmit_output_t output);

typedef enum
{
    DATA_TX_STOPPED = 0,
    DATA_TX_RECORDING = 1,
} data_transmit_state_t;

void data_transmit_start_record(void);
void data_transmit_stop_record(void);
void data_transmit_reset_record(void);
uint8 data_transmit_is_recording(void);

// Divider = 1 means 5ms; 2 means 10ms; etc.
void data_transmit_set_record_divider(uint16 divider);

// Call every 5ms to record pitch/yaw samples.
void data_transmit_task_5ms(void);

uint16 data_transmit_get_record_count(void);

// Send recorded data from start to stop.
void data_transmit_send_pitch_history(void);
void data_transmit_send_yaw_history(void);

// Send PID mode configuration table.
void data_transmit_send_pid_mode_cfg(void);

#endif /* CODE_DATA_TRANSMIT_H_ */
