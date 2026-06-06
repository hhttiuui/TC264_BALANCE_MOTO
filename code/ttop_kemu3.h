/*
 * ttop_kemu3.h
 *
 *  Created on: 2026-04-18
 *      Author: hhttiuui
 *
 *  赛事功能：固定 8 段路径（两段 kemu1 变体）
 *  顺序：直线,直线,转弯(EX1),直线,直线,转弯(EX2),直线,直线。
 */

#ifndef CODE_TTOP_KEMU3_H_
#define CODE_TTOP_KEMU3_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "zf_common_headfile.h"
#include "top_8_turn_2.h"
#define TTOP_KEMU3_SEG_COUNT           (8u)

typedef enum
{
	TTOP_KEMU3_SEG_STRAIGHT = 0u,
	TTOP_KEMU3_SEG_TURN     = 1u,
} ttop_kemu3_seg_type_t;

typedef struct
{
	unsigned char seg_type;
	float meters;
	short speed_counts_5ms;
	unsigned char turn_prog_id;
} ttop_kemu3_seg_t;

typedef struct
{
	unsigned char started;
	unsigned char done;
	unsigned char seg_idx;
	unsigned char turn_inited;
	long long straight_counts_accum;
	long long straight_end_counts[6];
	long long last_encoder_total;
} ttop_kemu3_status_t;

extern volatile int g_ttop_kemu3_counts_per_meter;
// EX2 单次弯道参数在 g_eight2_prog_ex2 中定义，以下参数用于设置重复次数。
extern volatile unsigned char g_ttop_kemu3_ex2_loop_count;
// KEMU3 最后一段回正参数（写入最后一个扩展段表）
extern volatile ttop_eight2_seg_t g_ttop_kemu3_end_seg;
extern volatile ttop_kemu3_seg_t g_ttop_kemu3_seg_table[TTOP_KEMU3_SEG_COUNT];

// 直线段 yaw 积分纠偏增益（菜单可调）
extern volatile float g_kemu3_yaw_ki_path;

void ttop_kemu3_init(void);
void ttop_kemu3_start(void);
void ttop_kemu3_reset(void);
unsigned char ttop_kemu3_task_5ms(void);
void ttop_kemu3_rebuild_targets(void);
const ttop_kemu3_status_t *ttop_kemu3_get_status(void);

#ifdef __cplusplus
}
#endif

#endif /* CODE_TTOP_KEMU3_H_ */
