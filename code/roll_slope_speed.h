/*
 * roll_slope_speed.h
 *
 *  Created on: 2026-05-24
 *      Author: hhttiuui
 *
 *  Roll-based slope speed helper (5ms task).
 */

#ifndef CODE_ROLL_SLOPE_SPEED_H_
#define CODE_ROLL_SLOPE_SPEED_H_

#include "zf_common_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float pre_up_decel_m;
    float up_start_deg;
    float up_end_deg;
    float down_start_deg;
    float down_end_deg;
    int16 up_delta_counts;
    int16 pre_up_decel_counts;
    int16 crest_delta_counts;
    int16 flat_delta_counts;
} roll_slope_speed_cfg_t;

// Initialize defaults for roll-based slope handling.
void roll_slope_speed_init(void);

// 5ms task: apply roll-based speed offset via pitch_speed_change().
void roll_slope_speed_task_5ms(void);

// Accessors for config (for menu/flash usage).
const roll_slope_speed_cfg_t *roll_slope_speed_get_cfg(void);
void roll_slope_speed_set_cfg(const roll_slope_speed_cfg_t *cfg);

// Enable/disable roll-based speed adjustment.
uint8 roll_slope_speed_get_enabled(void);
void roll_slope_speed_set_enabled(uint8 enabled);

// Start/reset control for slope state machine (start is one-shot).
void roll_slope_speed_start(void);
void roll_slope_speed_reset(void);
uint8 roll_slope_speed_get_started(void);

// One-shot completion state.
uint8 roll_slope_speed_get_done(void);

#ifdef __cplusplus
}
#endif

#endif /* CODE_ROLL_SLOPE_SPEED_H_ */
