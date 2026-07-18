/*
 * ttop_kemu1.h
 *
 *  Created on: 2026-04-16
 *      Author: hhttiuui
 *
 *  赛事功能：固定5段路径（两直线 + 一转弯 + 两直线）。
 *
 *  设计目标：
 *  - 段表结构固定为 5 段，便于菜单直接修改参数。
 *  - 直线路程统计不受转弯段影响。
 *  - 提供独立 init 与 5ms task 接口。
 */

#ifndef CODE_TTOP_KEMU1_H_
#define CODE_TTOP_KEMU1_H_

#ifdef __cplusplus
extern "C" {
#endif

#define TTOP_KEMU1_SEG_COUNT           (7u)

typedef enum
{
	TTOP_KEMU1_SEG_STRAIGHT = 0u,
	TTOP_KEMU1_SEG_TURN     = 1u,
} ttop_kemu1_seg_type_t;

typedef struct
{
	unsigned char seg_type;
	float meters;
	short speed_counts_5ms;
	unsigned char turn_prog_id;
} ttop_kemu1_seg_t;

typedef struct
{
	unsigned char started;
	unsigned char done;
	unsigned char seg_idx;
	unsigned char turn_inited;
	long long straight_counts_accum;
	long long straight_end_counts[6];
	long long last_encoder_total;
} ttop_kemu1_status_t;

// 可菜单修改：1m 对应编码器计数（建议默认 18500）
extern volatile int g_ttop_kemu1_counts_per_meter;

// 可菜单修改：固定 5 段路径参数（段类型固定：0/1/3/4=直线，2=转弯）
extern volatile ttop_kemu1_seg_t g_ttop_kemu1_seg_table[TTOP_KEMU1_SEG_COUNT];

// 初始化状态机（不会清空段表）
void ttop_kemu1_init(void);

// 5ms 调度函数：返回 0=未完成，1=已完成
unsigned char ttop_kemu1_task_5ms(void);

// 当菜单改动“米数”或“1m编码器值”后调用，用于重建直线阈值
void ttop_kemu1_rebuild_targets(void);

// 调试读取状态（只读）
const ttop_kemu1_status_t *ttop_kemu1_get_status(void);

#ifdef __cplusplus
}
#endif

#endif /* CODE_TTOP_KEMU1_H_ */
