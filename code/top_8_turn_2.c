/*
 * top_8_turn_2.c
 *
 *  Created on: 2026-01-26
 *      Author: hhttiuui
 *
 *  赛事功能：8字绕行（算法2，表驱动 + 每段可调限速平滑）
 *
 *  核心思路：
 *  - 用数组/段表给出每一段的目标 yaw 值（目标值，不是增量）
 *  - 每 5ms 调用一次 task：用限速器让输出 yaw 平滑逼近本段目标
 *  - 输出：直接写 g_yaw_target_deg
 *  - 到位判定：abs(target-out) <= eps 且连续保持 hold_ticks 次
 *  - 所有段完成后返回 1
 */

#include "zf_common_headfile.h"
#include "top_8_turn_2.h"

extern float show_yaw_deg;

// 前置声明：避免依赖 Ttop_funtion.c 的 static 函数
// static inline uint8 ttop_yaw_delay_done_local(void);

// ============================ 可调参数（可按需要在此改） ============================

// 是否启用
#ifndef TTOP_EIGHT2_ENABLE
#define TTOP_EIGHT2_ENABLE                 (1)
#endif

// 到位阈值（deg）：比较 out 与 target
#ifndef TTOP_EIGHT2_INPOS_EPS_DEG
#define TTOP_EIGHT2_INPOS_EPS_DEG          (0.8f)
#endif

// 连续到位确认次数（5ms 一次）：8 次=40ms
#ifndef TTOP_EIGHT2_HOLD_TICKS
#define TTOP_EIGHT2_HOLD_TICKS             (8u)
#endif

// 默认每段限速（deg/s）。若段表里 rate_dps 填 0，则用默认值
#ifndef TTOP_EIGHT2_DEFAULT_RATE_DPS
#define TTOP_EIGHT2_DEFAULT_RATE_DPS       (60.0f)
#endif

// ============================ 段表（你后续只需要改这里就能调每段） ============================
// 约定：目标值是“相对 base0 的偏移量”（deg），最终 target_abs = base0 + target_rel。
// 例如：[0,30,0,-40,0,180,220,180,150,180]

// 对外可写段表（长度固定为 TTOP_EIGHT2_PROG_LEN=12）。
// 未填充的段自动保持为 0（{0}）——也就是 target_rel=0，rate=0（使用默认限速）。
volatile ttop_eight2_seg_t g_eight2_prog[TTOP_EIGHT2_PROG_LEN] =
{
	{ 55.0f,  20.0f },
	{  0.0f,  20.0f },
	{-75.0f,  20.0f },
	{  0.0f,  20.0f },
	{180.0f,  20.0f },
	{240.0f,  20.0f },
	{180.0f,  20.0f },
	{135.0f,  20.0f },
	{180.0f,  20.0f },
	{180.0f,  20.0f },
	{180.0f,  20.0f },
	{180.0f,  20.0f },
};

// 扩展段表 1：预留给其他功能（默认全 0，等效于“目标=0、限速=默认”）
volatile ttop_eight2_seg_t g_eight2_prog_ex1[TTOP_EIGHT2_PROG_LEN] = { {0.0f, 0.0f} };

// 扩展段表 2：预留给其他功能（默认全 0，等效于“目标=0、限速=默认”）
volatile ttop_eight2_seg_t g_eight2_prog_ex2[TTOP_EIGHT2_PROG_LEN] = { {0.0f, 0.0f} };

// 当前活动段表（默认 main）
static volatile ttop_eight2_seg_t *s_eight2_prog_active = g_eight2_prog;
static volatile ttop_eight2_prog_id_t s_eight2_prog_id = TTOP_EIGHT2_PROG_MAIN;

void ttop_eight2_select_program(ttop_eight2_prog_id_t id)
{
	if (id == TTOP_EIGHT2_PROG_EX1)
	{
		s_eight2_prog_active = g_eight2_prog_ex1;
		s_eight2_prog_id = id;
	}
	else if (id == TTOP_EIGHT2_PROG_EX2)
	{
		s_eight2_prog_active = g_eight2_prog_ex2;
		s_eight2_prog_id = id;
	}
	else
	{
		s_eight2_prog_active = g_eight2_prog;
		s_eight2_prog_id = TTOP_EIGHT2_PROG_MAIN;
	}

	// 切表后复位状态机，避免跨段表串状态
	ttop_eight2_reset_now();
}

ttop_eight2_prog_id_t ttop_eight2_get_program(void)
{
	return (ttop_eight2_prog_id_t)s_eight2_prog_id;
}

typedef struct
{
	uint8  started;
	uint8  done;
	uint16 seg_idx;
	uint16 inpos_ticks;
	float  base0_yaw_deg;
	float  yaw_out_deg;
	float  yaw_target_abs_deg;
} ttop_eight2_ctrl_t;

static ttop_eight2_ctrl_t s_eight2;

static inline float ttop_absf(float x)
{
	return (x < 0.0f) ? -x : x;
}

static inline uint8 ttop_inpos(float target_deg, float out_deg)
{
	return (ttop_absf(target_deg - out_deg) <= (float)TTOP_EIGHT2_INPOS_EPS_DEG) ? 1u : 0u;
}

static inline float ttop_limit_slew(float current, float target, float max_delta)
{
	float err = target - current;
	if (err > max_delta) return current + max_delta;
	if (err < -max_delta) return current - max_delta;
	return target;
}

void ttop_eight2_init(void)
{
	s_eight2.started = 0u;
	s_eight2.done = 0u;
	s_eight2.seg_idx = 0u;
	s_eight2.inpos_ticks = 0u;
	s_eight2.base0_yaw_deg = 0.0f;
	s_eight2.yaw_out_deg = 0.0f;
	s_eight2.yaw_target_abs_deg = 0.0f;
}

void ttop_eight2_reset_now(void)
{
	s_eight2.started = 0u;
	s_eight2.done = 0u;
	s_eight2.seg_idx = 0u;
	s_eight2.inpos_ticks = 0u;
	s_eight2.base0_yaw_deg = 0.0f;
	s_eight2.yaw_out_deg = 0.0f;
	s_eight2.yaw_target_abs_deg = 0.0f;
	g_yaw_target_deg = 0.0f;
}

// =======8字绕行任务（算法2），5ms调用一次=======
// return: 0=未完成 1=完成
uint8 ttop_eight2_task_5ms(void)
{
#if (TTOP_EIGHT2_ENABLE)
	// // 停机后不再规划，且保持 yaw 指令为 0
	// if (ttop_stop_is_active())
	// {
	// 	s_eight2.done = 0u;
	// 	s_eight2.started = 0u;
	// 	s_eight2.seg_idx = 0u;
	// 	s_eight2.inpos_ticks = 0u;
	// 	s_eight2.yaw_out_deg = 0.0f;
	// 	s_eight2.yaw_target_abs_deg = 0.0f;
	// 	g_yaw_target_deg = 0.0f;
	// 	return 0u;
	// }

	// // yaw 延时置零未完成时，不做规划
	// if (!ttop_yaw_delay_done_local())
	// {
	// 	s_eight2.started = 0u;
	// 	s_eight2.seg_idx = 0u;
	// 	s_eight2.inpos_ticks = 0u;
	// 	s_eight2.yaw_out_deg = 0.0f;
	// 	s_eight2.yaw_target_abs_deg = 0.0f;
	// 	g_yaw_target_deg = 0.0f;
	// 	return 0u;
	// }

	// 首次进入：抓取 base0，并从当前角开始输出，避免突变
	if (!s_eight2.started)
	{
		s_eight2.started = 1u;
		s_eight2.done = 0u;
		s_eight2.seg_idx = 0u;
		s_eight2.inpos_ticks = 0u;
		s_eight2.base0_yaw_deg = 0.0f;
		s_eight2.yaw_out_deg = s_eight2.base0_yaw_deg;
	}

	// 已完成：保持最后一段目标
	if (s_eight2.done)
	{
		g_yaw_target_deg = s_eight2.yaw_out_deg;
		return 1u;
	}

	// 取当前段（volatile 段表：避免被编译器缓存，允许上层运行时修改）
	const volatile ttop_eight2_seg_t *seg = &s_eight2_prog_active[s_eight2.seg_idx];
	float rate_dps = seg->rate_dps;
	if (rate_dps <= 0.0f) rate_dps = (float)TTOP_EIGHT2_DEFAULT_RATE_DPS;
	float max_delta = rate_dps * ((float)TTOP_TURN_DT_MS * 0.001f);

	// 本段目标：绝对目标 = base0 + 目标偏移
	s_eight2.yaw_target_abs_deg = s_eight2.base0_yaw_deg + seg->target_rel_deg;

	// 限速平滑逼近
	s_eight2.yaw_out_deg = ttop_limit_slew(s_eight2.yaw_out_deg, s_eight2.yaw_target_abs_deg, max_delta);
	g_yaw_target_deg = s_eight2.yaw_out_deg;

	// 到位判定（按 out 接近 target）
	if (ttop_inpos(s_eight2.yaw_target_abs_deg, s_eight2.yaw_out_deg))
	{
		if (++s_eight2.inpos_ticks >= (uint16)TTOP_EIGHT2_HOLD_TICKS)
		{
			s_eight2.inpos_ticks = 0u;
			s_eight2.seg_idx++;
			if (s_eight2.seg_idx >= (uint16)TTOP_EIGHT2_PROG_LEN)
			{
				s_eight2.seg_idx = (uint16)(TTOP_EIGHT2_PROG_LEN - 1u);
				s_eight2.done = 1u;
				return 1u;
			}
		}
	}
	else
	{
		s_eight2.inpos_ticks = 0u;
	}

	return 0u;
#else
	return 0u;
#endif
}

// static inline uint8 ttop_yaw_delay_done_local(void)
// {
// 	const led_bzw_show_status_t *st = led_bzw_show_status_get();
// 	return (st && st->yaw_delay_done) ? 1u : 0u;
// }
