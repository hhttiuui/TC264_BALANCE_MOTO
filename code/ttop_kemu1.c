/*
 * ttop_kemu1.c
 *
 *  Created on: 2026-04-16
 *      Author: hhttiuui
 *
 *  赛事功能：固定5段路径（两直线 + 一转弯 + 两直线）。
 */

#include "ttop_kemu1.h"

#include "encoder_count.h"
#include "motor_controller.h"
#include "top_8_turn_2.h"

// 1m 对应编码器计数（可被菜单修改）
volatile int g_ttop_kemu1_counts_per_meter = 18500;

// 固定 5 段（可被菜单修改）
// 约定：
// - seg[0], seg[1], seg[3], seg[4] 必须是直线段
// - seg[2] 必须是转弯段
volatile ttop_kemu1_seg_t g_ttop_kemu1_seg_table[TTOP_KEMU1_SEG_COUNT] =
{
	{ TTOP_KEMU1_SEG_STRAIGHT, 20.0f, 200, TTOP_EIGHT2_PROG_MAIN },
	{ TTOP_KEMU1_SEG_STRAIGHT,  5.0f, 120, TTOP_EIGHT2_PROG_MAIN },
	{ TTOP_KEMU1_SEG_TURN,      0.0f, 120, TTOP_EIGHT2_PROG_EX1  },
	{ TTOP_KEMU1_SEG_STRAIGHT, 12.5f, 200, TTOP_EIGHT2_PROG_MAIN },
	{ TTOP_KEMU1_SEG_STRAIGHT, 12.5f, 200, TTOP_EIGHT2_PROG_MAIN },
};

static ttop_kemu1_status_t s_kemu1;

static inline long long ttop_abs_i64_local(long long x)
{
	return (x < 0) ? -x : x;
}

static inline long long ttop_meter_to_count_local(float meters)
{
	float m = (meters < 0.0f) ? 0.0f : meters;
	float cpm = (float)((g_ttop_kemu1_counts_per_meter <= 0) ? 1 : g_ttop_kemu1_counts_per_meter);
	float counts_f = m * cpm + 0.5f;
	if (counts_f < 0.0f)
	{
		counts_f = 0.0f;
	}
	return (long long)counts_f;
}

static inline unsigned char ttop_is_straight_seg_local(unsigned char seg_idx)
{
	return (seg_idx == 0u || seg_idx == 1u || seg_idx == 3u || seg_idx == 4u) ? 1u : 0u;
}

static inline unsigned char ttop_straight_slot_from_seg_local(unsigned char seg_idx)
{
	if (seg_idx == 0u) return 0u;
	if (seg_idx == 1u) return 1u;
	if (seg_idx == 3u) return 2u;
	if (seg_idx == 4u) return 3u;
	return 0xFFu;
}

void ttop_kemu1_rebuild_targets(void)
{
	long long accum = 0;
	unsigned char slot = 0u;
	unsigned char i;

	for (i = 0u; i < TTOP_KEMU1_SEG_COUNT; ++i)
	{
		if (!ttop_is_straight_seg_local(i))
		{
			continue;
		}

		accum += ttop_meter_to_count_local(g_ttop_kemu1_seg_table[i].meters);
		if (slot < 4u)
		{
			s_kemu1.straight_end_counts[slot] = accum;
			slot++;
		}
	}

	while (slot < 4u)
	{
		s_kemu1.straight_end_counts[slot++] = accum;
	}
}

void ttop_kemu1_init(void)
{
	s_kemu1.started = 0u;
	s_kemu1.done = 0u;
	s_kemu1.seg_idx = 0u;
	s_kemu1.turn_inited = 0u;
	s_kemu1.straight_counts_accum = 0;
	s_kemu1.last_encoder_total = encoder_get_demo5_count_total();

	ttop_kemu1_rebuild_targets();
}

unsigned char ttop_kemu1_task_5ms(void)
{
	if (s_kemu1.done)
	{
		return 1u;
	}

	if (!s_kemu1.started)
	{
		s_kemu1.started = 1u;
		s_kemu1.done = 0u;
		s_kemu1.seg_idx = 0u;
		s_kemu1.turn_inited = 0u;
		s_kemu1.straight_counts_accum = 0;
		s_kemu1.last_encoder_total = encoder_get_demo5_count_total();
	}

	if (s_kemu1.seg_idx >= TTOP_KEMU1_SEG_COUNT)
	{
		s_kemu1.done = 1u;
		return 1u;
	}

	const volatile ttop_kemu1_seg_t *seg = &g_ttop_kemu1_seg_table[s_kemu1.seg_idx];
	motor_set_target_counts_per_5ms(seg->speed_counts_5ms);

	if (s_kemu1.seg_idx == 2u)
	{
		if (!s_kemu1.turn_inited)
		{
			ttop_eight2_select_program((ttop_eight2_prog_id_t)seg->turn_prog_id);
			ttop_eight2_reset_now();
			s_kemu1.turn_inited = 1u;
		}

		if (ttop_eight2_task_5ms())
		{
			s_kemu1.seg_idx++;
			s_kemu1.turn_inited = 0u;
			if (s_kemu1.seg_idx >= TTOP_KEMU1_SEG_COUNT)
			{
				s_kemu1.done = 1u;
				return 1u;
			}
		}

		return 0u;
	}

	// 仅在直线段累计路程，确保直线总里程不受转弯影响。
	{
		long long now_total = encoder_get_demo5_count_total();
		long long delta = now_total - s_kemu1.last_encoder_total;
		s_kemu1.last_encoder_total = now_total;
		s_kemu1.straight_counts_accum += ttop_abs_i64_local(delta);
	}

	{
		unsigned char slot = ttop_straight_slot_from_seg_local(s_kemu1.seg_idx);
		if (slot < 4u)
		{
			if (s_kemu1.straight_counts_accum >= s_kemu1.straight_end_counts[slot])
			{
				s_kemu1.seg_idx++;
				s_kemu1.turn_inited = 0u;
				if (s_kemu1.seg_idx >= TTOP_KEMU1_SEG_COUNT)
				{
					s_kemu1.done = 1u;
					return 1u;
				}
			}
		}
	}

	return 0u;
}

const ttop_kemu1_status_t *ttop_kemu1_get_status(void)
{
	return &s_kemu1;
}
