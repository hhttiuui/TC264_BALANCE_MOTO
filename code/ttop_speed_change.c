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

// 速度分档（单位：encoder counts / 5ms，按绝对值比较）
// 目标档位：
// - PID_MODE_CASCADE_EX1 -> 140
// - PID_MODE_CASCADE     -> 170
// - PID_MODE_CASCADE_EX2 -> 200
#ifndef TTOP_SPEED_CHANGE_V1
#define TTOP_SPEED_CHANGE_V1            (140.0f)
#endif

#ifndef TTOP_SPEED_CHANGE_V2
#define TTOP_SPEED_CHANGE_V2            (170.0f)
#endif

#ifndef TTOP_SPEED_CHANGE_V3
#define TTOP_SPEED_CHANGE_V3            (200.0f)
#endif

// 切档滞回（counts/5ms）
#ifndef TTOP_SPEED_CHANGE_HYS
#define TTOP_SPEED_CHANGE_HYS           (10.0f)
#endif

// 速度低通滤波系数（0~1，越大越平滑）
#ifndef TTOP_SPEED_CHANGE_LPF_ALPHA
#define TTOP_SPEED_CHANGE_LPF_ALPHA     (0.80f)
#endif

typedef struct
{
	uint8_t inited;
	pid_mode_t mode;
	float speed_abs_filt;
} ttop_speed_change_ctrl_t;

static ttop_speed_change_ctrl_t s_spd = {0u, PID_MODE_CASCADE, 0.0f};

static inline float ttop_absf_local(float x)
{
	return (x < 0.0f) ? -x : x;
}

static pid_mode_t ttop_speed_change_pick_mode(float speed_abs)
{
	const float up_12 = 0.5f * ((float)TTOP_SPEED_CHANGE_V1 + (float)TTOP_SPEED_CHANGE_V2);
	const float up_23 = 0.5f * ((float)TTOP_SPEED_CHANGE_V2 + (float)TTOP_SPEED_CHANGE_V3);
	const float dn_21 = up_12 - (float)TTOP_SPEED_CHANGE_HYS;
	const float dn_32 = up_23 - (float)TTOP_SPEED_CHANGE_HYS;

	if (s_spd.mode == PID_MODE_CASCADE_EX1)
	{
		if (speed_abs >= up_12) return PID_MODE_CASCADE;
		return PID_MODE_CASCADE_EX1;
	}
	if (s_spd.mode == PID_MODE_CASCADE)
	{
		if (speed_abs >= up_23) return PID_MODE_CASCADE_EX2;
		if (speed_abs <= dn_21) return PID_MODE_CASCADE_EX1;
		return PID_MODE_CASCADE;
	}

	// 当前为 PID_MODE_CASCADE_EX2
	if (speed_abs <= dn_32) return PID_MODE_CASCADE;
	return PID_MODE_CASCADE_EX2;
}

void ttop_speed_change_init(void)
{
	s_spd.inited = 1u;
	s_spd.mode = PID_MODE_CASCADE_EX1;
	s_spd.speed_abs_filt = 0.0f;

	// 初始化时先同步一次到中速档参数。
	PID_HOT_SWITCH_BY_MODE(s_spd.mode);
}

void ttop_speed_change_task_5ms(void)
{
	if (!s_spd.inited)
	{
		ttop_speed_change_init();
	}

	// 使用编码器 5ms 增量的绝对值作为速度量。
	const float speed_abs_raw = ttop_absf_local((float)encoder_get_demo5_delta_5ms());
	s_spd.speed_abs_filt = (float)TTOP_SPEED_CHANGE_LPF_ALPHA * s_spd.speed_abs_filt +
						  (1.0f - (float)TTOP_SPEED_CHANGE_LPF_ALPHA) * speed_abs_raw;

	pid_mode_t next = ttop_speed_change_pick_mode(s_spd.speed_abs_filt);
	if (next != s_spd.mode)
	{
		s_spd.mode = next;
		PID_HOT_SWITCH_BY_MODE(s_spd.mode);
	}

	// 使用与热切换一致的 mode 执行本周期 PID 计算。
	pid_timer_handler_by_mode(s_spd.mode);
}

pid_mode_t ttop_speed_change_get_mode(void)
{
	return s_spd.mode;
}