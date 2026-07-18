/*
 * motor_controller.h
 *
 *  Created on: 2026-01-19
 *      Author: hhttiuui
 *
 *  电机速度环控制接口声明（5ms周期）。
 */

#ifndef CODE_MOTOR_CONTROLLER_H_
#define CODE_MOTOR_CONTROLLER_H_

#include "zf_common_headfile.h"
#include "encoder_count.h"

#ifdef __cplusplus
extern "C" {
#endif

// 初始化电机速度环（在 PID_INIT 或 main 初始化时调用）
void motor_pid_init(void);

// 5ms 调度调用：读取编码器速度，运行速度 PID，输出到电机
// 建议在 main 的 encoder_flag 分支中调用
void motor_control_task_5ms(void);

// 以编码器 5ms 增量为单位设置目标速度（与 encoder_get_demo5_delta_5ms 返回单位一致）
void motor_set_target_counts_per_5ms(int16 target_counts);

// 目标速度偏置（counts/5ms）：正值加速，负值减速；传 0 解除偏置
void pitch_speed_change(int16 delta_counts);

// 设置/获取占空比斜率限制（单位：百分比/秒），由外部模块调用以调整保护强度
void motor_set_duty_slope_limit_percent_per_s(float percent_per_s);
float motor_get_duty_slope_limit_percent_per_s(void);

// 设置/获取速度前馈参数：u_ff = k_v * v_ref + k_s * sign(v_ref)
void motor_set_feedforward_gains(float k_s, float k_v);
void motor_get_feedforward_gains(float *k_s, float *k_v);

// 设置/获取电机速度环 PID 参数
void motor_set_pid_gains(float kp, float ki, float kd);
void motor_get_pid_gains(float *kp, float *ki, float *kd);

// 基于 |pitch| 的目标速度增益：
// - enable=1 时，当 |pitch| > threshold_abs_deg，额外速度增量为
//   k_counts_per_deg * (|pitch| - threshold_abs_deg)
// - 额外增量会叠加到目标速度上
void motor_set_pitch_speed_boost_enable(uint8 enable);
uint8 motor_get_pitch_speed_boost_enable(void);
void motor_set_pitch_speed_boost_params(float threshold_abs_deg, float k_counts_per_deg);
void motor_get_pitch_speed_boost_params(float *threshold_abs_deg, float *k_counts_per_deg);
void motor_set_pitch_speed_boost_max_counts(float max_counts);
float motor_get_pitch_speed_boost_max_counts(void);
float motor_get_pitch_speed_boost_counts(void);

// BOOST 模式：0=OFF, 1=DUTY, 2=PITCH, 3=ACC_RATE
void motor_set_boost_mode(uint8 mode);
uint8 motor_get_boost_mode(void);
void motor_set_boost_map_k(float duty_k, float pitch_k, float acc_k);
void motor_get_boost_map_k(float *duty_k, float *pitch_k, float *acc_k);

// 根据倾角自适应调整目标速度：
// - threshold_deg: 倒下阈值（deg），超过则进入“救车加速”
// - speed_fall: 超阈值时设置的目标速度（counts/5ms）
// - speed_safe: 安全角度内设置的目标速度（counts/5ms）
// 建议在 5ms 周期中调用（例如 encoder 任务里，每次 motor_control_task_5ms 前）。
void motor_acc(float threshold_deg, int16 speed_fall, int16 speed_safe);

// 获取当前过滤后的速度（counts per 5ms）
int16 motor_get_speed_counts_filtered(void);

// 速度清零：清除测量/滤波/PID 状态，常用于 RESET/重新校准
void motor_clear_speed(void);

// 调试状态（可在 UI 中显示）
extern volatile float motor_pid_out;             // PID 输出（映射到 duty）
extern volatile float motor_ff_out;              // 前馈输出（映射到 duty）
extern volatile int16 motor_speed_raw_counts;    // 原始编码器 delta
extern volatile float motor_speed_filtered_counts; // 滤波后的速度
extern volatile uint32 count_motor_pid;          // 调用计数
extern volatile float motor_pitch_boost_counts;  // 基于 |pitch| 的速度增量（counts/5ms）

// 启动控制接口已移除：软起动/推动启动相关逻辑在当前实现中不再使用。

#ifdef __cplusplus
}
#endif

#endif /* CODE_MOTOR_CONTROLLER_H_ */
