/*
 * ttop_speed_change.c
 *
 *  Created on: 2026-04-01
 *      Author: hhttiuui
 *
 *  赛事功能：不同速度对应不同 PID 的实现文件（预留）。
 */

#include "zf_common_headfile.h"
#include "ttop_speed_change.h"

// 速度低通滤波系数（0~1，越大越平滑）
#ifndef TTOP_SPEED_CHANGE_LPF_ALPHA
#define TTOP_SPEED_CHANGE_LPF_ALPHA     (0.80f)
#endif

typedef struct
{
	uint8_t inited;
	pid_mode_t mode;
	pid_mode_t mode_high;
	pid_mode_t mode_low;
	float speed_abs_filt;
	float angle_kp_now;
	float rate_kp_now;
} ttop_speed_change_ctrl_t;

static ttop_speed_change_ctrl_t s_spd = {0u, PID_MODE_CASCADE, PID_MODE_CASCADE_EX2, PID_MODE_CASCADE_EX1, 0.0f, 0.0f, 0.0f};

static inline float ttop_absf_local(float x)
{
	return (x < 0.0f) ? -x : x;
}

static inline float ttop_clampf_local(float x, float lo, float hi)
{
	if (x < lo) return lo;
	if (x > hi) return hi;
	return x;
}

static inline float ttop_lerpf_local(float a, float b, float t)
{
	return a + (b - a) * t;
}

static inline const pid_mode_cfg_t *ttop_cfg_by_mode(pid_mode_t mode)
{
	int idx = (int)mode;
	if (idx < 0 || idx >= (int)PID_MODE_CFG_COUNT)
	{
		idx = (int)PID_MODE_CASCADE;
	}
	return &s_pid_mode_cfg[idx];
}

static inline uint8_t ttop_mode_is_cascade(pid_mode_t mode)
{
	const pid_mode_cfg_t *cfg = ttop_cfg_by_mode(mode);
	return (cfg->strategy == PID_STRAT_CASCADE) ? 1u : 0u;
}

void ttop_speed_change_init(pid_mode_t high_mode, pid_mode_t low_mode)
{
	s_spd.inited = 1u;
	s_spd.mode = PID_MODE_CASCADE;

	// 仅接受 cascade 策略作为高/低速参数来源，非法输入回退到 EX2/EX1。
	s_spd.mode_high = ttop_mode_is_cascade(high_mode) ? high_mode : PID_MODE_CASCADE_EX2;
	s_spd.mode_low = ttop_mode_is_cascade(low_mode) ? low_mode : PID_MODE_CASCADE_EX1;
	s_spd.speed_abs_filt = 0.0f;

	const pid_mode_cfg_t *cfg_low = ttop_cfg_by_mode(s_spd.mode_low);
	s_spd.angle_kp_now = cfg_low->angle.kp;
	s_spd.rate_kp_now = cfg_low->rate.kp;

	// 使用低速模式作为动态链路的基础参数（限幅/ki/kd/yaw 等）。
	PID_HOT_SWITCH_BY_MODE(s_spd.mode_low);
}

void ttop_speed_change_task_5ms(void)
{
	if (!s_spd.inited)
	{
		ttop_speed_change_init(PID_MODE_CASCADE_EX2, PID_MODE_CASCADE_EX1);
	}

	// 使用编码器 5ms 增量的绝对值作为速度量。
	const float speed_abs_raw = ttop_absf_local((float)encoder_get_demo5_delta_5ms());
	s_spd.speed_abs_filt = (float)TTOP_SPEED_CHANGE_LPF_ALPHA * s_spd.speed_abs_filt +
						  (1.0f - (float)TTOP_SPEED_CHANGE_LPF_ALPHA) * speed_abs_raw;

	const pid_mode_cfg_t *cfg_low = ttop_cfg_by_mode(s_spd.mode_low);
	const pid_mode_cfg_t *cfg_high = ttop_cfg_by_mode(s_spd.mode_high);

	float v_low = (float)cfg_low->motor_target_counts_5ms;
	float v_high = (float)cfg_high->motor_target_counts_5ms;
	if (v_high <= v_low + 1e-6f)
	{
		v_low = 140.0f;
		v_high = 200.0f;
	}

	const float v_expand = 50.0f;
	float v_lerp_low = v_low - v_expand;
	float v_lerp_high = v_high + v_expand;
	if (v_lerp_low < 0.0f)
	{
		v_lerp_low = 0.0f;
	}
	if (v_lerp_high <= v_lerp_low + 1e-6f)
	{
		v_lerp_high = v_lerp_low + 1.0f;
	}

	float alpha = ttop_clampf_local((s_spd.speed_abs_filt - v_lerp_low) / (v_lerp_high - v_lerp_low), 0.0f, 1.0f);
	float angle_kp = ttop_lerpf_local(cfg_low->angle.kp, cfg_high->angle.kp, alpha);
	float rate_kp = ttop_lerpf_local(cfg_low->rate.kp, cfg_high->rate.kp, alpha);
	s_spd.angle_kp_now = angle_kp;
	s_spd.rate_kp_now = rate_kp;

	// 插值仅在 speed_change 模块内完成，common_flie 仅负责接收参数并执行调度。
	pid_dynamic_cascade_set_kp(angle_kp, rate_kp);

	// 动态 PID 专用处理（固定 cascade 链路）。
	pid_timer_handler_dynamic_cascade(s_spd.mode_low);
}

//返回当前分档选择的 PID 模式（用于调试显示）
pid_mode_t ttop_speed_change_get_mode(void){return s_spd.mode;}

//返回当前滤波后的速度绝对值（用于调试显示）
float ttop_speed_change_get_speed_abs_filt(void){return s_spd.speed_abs_filt;}

//返回当前动态插值后的 angle_kp（用于调试显示）
float ttop_speed_change_get_angle_kp(void){return s_spd.angle_kp_now;}

//返回当前动态插值后的 rate_kp（用于调试显示）
float ttop_speed_change_get_rate_kp(void){return s_spd.rate_kp_now;}