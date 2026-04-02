/*
 * ttop_speed_change.h
 *
 *  Created on: 2026-04-01
 *      Author: hhttiuui
 *
 *  赛事功能：不同速度对应不同 PID 的接口声明。
 */

#ifndef CODE_TTOP_SPEED_CHANGE_H_
#define CODE_TTOP_SPEED_CHANGE_H_

#include "zf_common_typedef.h"
#include "common_flie.h"

#ifdef __cplusplus
extern "C" {
#endif

// 速度分档热切换模块初始化（建议上电或模式切换后调用一次）
void ttop_speed_change_init(void);

// 5ms 任务：根据编码器速度实时切换 PID 档位（调用 PID_HOT_SWITCH_BY_MODE）
void ttop_speed_change_task_5ms(void);

// 查询当前分档选择的 PID 模式（用于调试显示）
pid_mode_t ttop_speed_change_get_mode(void);



#ifdef __cplusplus
}
#endif

#endif /* CODE_TTOP_SPEED_CHANGE_H_ */
