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

// 动态速度模块初始化：
// - high_mode: 高速参数来源模式（例如 PID_MODE_CASCADE_EX2 / CASCADE3）
// - low_mode : 低速参数来源模式（例如 PID_MODE_CASCADE_EX1 / CASCADE2）
void ttop_speed_change_init(pid_mode_t high_mode, pid_mode_t low_mode);

// 5ms 任务：根据编码器速度实时切换 PID 档位（调用 PID_HOT_SWITCH_BY_MODE）
void ttop_speed_change_task_5ms(void);

// 查询当前分档选择的 PID 模式（用于调试显示）
pid_mode_t ttop_speed_change_get_mode(void);

// 查询动态插值链路实时量（用于菜单调试显示）
float ttop_speed_change_get_speed_abs_filt(void);
float ttop_speed_change_get_speed_pred(void);
float ttop_speed_change_get_angle_kp(void);
float ttop_speed_change_get_rate_kp(void);
float ttop_speed_change_get_yaw_kp(void);

// 拟合方式选择（可被菜单修改）：0=1/v^2 映射，1=线性 y=kx+b
extern volatile uint8 g_ttop_speed_change_fit_mode;

// 速度低通滤波系数（0~1，可被菜单修改）
extern volatile float g_ttop_speed_change_lpf_alpha;

// 速度预测增益（0~2）：speed_pred = speed_filt + gain * dv_filt
extern volatile float g_ttop_speed_change_pred_gain;



#ifdef __cplusplus
}
#endif

#endif /* CODE_TTOP_SPEED_CHANGE_H_ */
