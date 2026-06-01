/*
 * top_8_turn_2.h
 *
 *  Created on: 2026-01-26
 *      Author: hhttiuui
 *
 *  赛事功能：8字绕行（算法2，预留）对外接口。
 */

#ifndef CODE_TOP_8_TURN_2_H_
#define CODE_TOP_8_TURN_2_H_

#include "zf_common_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

// 新的“绕 8 字”算法（算法2）：表驱动（每段目标yaw + 每段限速）
// - 由调用方按 5ms 周期调用 task
// - 输出：直接写 g_yaw_target_deg
// - 返回值：0=未完成 1=完成

// 段表对外暴露：上层（例如菜单）可直接修改每一段的目标与限速。
// 约定：target_rel_deg 为“相对 base0 的偏移量”（deg），最终 target_abs = base0 + target_rel。
typedef struct
{
	float target_rel_deg; // 相对 base0 的目标偏移（deg）
	float rate_dps;       // 本段限速（deg/s）。填 0 则使用默认
} ttop_eight2_seg_t;

// 段表长度固定为 12；不足的段保持为 0（{0}），等效“目标=0、限速=默认”。
#ifndef TTOP_EIGHT2_PROG_LEN
#define TTOP_EIGHT2_PROG_LEN  (12u)
#endif

// ============================ 多段表支持（可切换） ============================

typedef enum
{
	TTOP_EIGHT2_PROG_MAIN = 0u, // 默认主段表（g_eight2_prog）
	TTOP_EIGHT2_PROG_EX1  = 1u, // 扩展段表1
	TTOP_EIGHT2_PROG_EX2  = 2u, // 扩展段表2
	TTOP_EIGHT2_PROG_EX3  = 3u, // 扩展段表3
	TTOP_EIGHT2_PROG_EX4  = 4u, // 扩展段表4
	TTOP_EIGHT2_PROG_EX5  = 5u, // 扩展段表5
	TTOP_EIGHT2_PROG_EX6  = 6u, // 扩展段表6
	TTOP_EIGHT2_PROG_EX7  = 7u, // 扩展段表7
} ttop_eight2_prog_id_t;

// 选择当前使用的段表：内部会 reset 状态机，避免跨段表串状态
void ttop_eight2_select_program(ttop_eight2_prog_id_t id);

// 查询当前段表
ttop_eight2_prog_id_t ttop_eight2_get_program(void);

// 对外暴露三套段表：上层（例如菜单/其他功能）可直接修改
extern volatile ttop_eight2_seg_t g_eight2_prog[TTOP_EIGHT2_PROG_LEN];
extern volatile ttop_eight2_seg_t g_eight2_prog_ex1[TTOP_EIGHT2_PROG_LEN];
extern volatile ttop_eight2_seg_t g_eight2_prog_ex2[TTOP_EIGHT2_PROG_LEN];
extern volatile ttop_eight2_seg_t g_eight2_prog_ex3[TTOP_EIGHT2_PROG_LEN];
extern volatile ttop_eight2_seg_t g_eight2_prog_ex4[TTOP_EIGHT2_PROG_LEN];
extern volatile ttop_eight2_seg_t g_eight2_prog_ex5[TTOP_EIGHT2_PROG_LEN];
extern volatile ttop_eight2_seg_t g_eight2_prog_ex6[TTOP_EIGHT2_PROG_LEN];
extern volatile ttop_eight2_seg_t g_eight2_prog_ex7[TTOP_EIGHT2_PROG_LEN];

void ttop_eight2_init(void);
void ttop_eight2_reset_now(void);
uint8 ttop_eight2_task_5ms(void);

#ifdef __cplusplus
}
#endif

#endif /* CODE_TOP_8_TURN_2_H_ */
