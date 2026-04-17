/*
 * ttop_rc_control.h
 *
 *  Created on: 2026-04-05
 *      Author: hhttiuui
 *
 *  赛事功能：遥控器通道到控制量映射（CH0->g_yaw_target_deg, CH1->motor target）。
 */

#ifndef CODE_TTOP_RC_CONTROL_H_
#define CODE_TTOP_RC_CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
	float in_min;
	float in_center;
	float in_max;
	float out_min;
	float out_center;
	float out_max;
	float sensitivity;
} ttop_rc_axis_cfg_t;

typedef struct
{
	ttop_rc_axis_cfg_t yaw;
	ttop_rc_axis_cfg_t speed;
	float yaw_rate_limit_deg_per_5ms;
} ttop_rc_control_cfg_t;

void ttop_rc_control_init(void);
void ttop_rc_control_set_config(const ttop_rc_control_cfg_t *cfg);
const ttop_rc_control_cfg_t *ttop_rc_control_get_config(void);
void ttop_rc_control_emergency_stop_now(void);
void ttop_rc_control_task_5ms(void);

#ifdef __cplusplus
}
#endif

#endif /* CODE_TTOP_RC_CONTROL_H_ */
