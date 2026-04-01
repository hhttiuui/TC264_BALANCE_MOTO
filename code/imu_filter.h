/*
 * imu_filter.h
 *
 *  Created on: 2026-01-18
 *      Author: hhttiuui
 *
 *  IMU 一阶低通滤波封装（加速度/陀螺单位转换后再滤波）。
 */

#ifndef CODE_IMU_FILTER_H_
#define CODE_IMU_FILTER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 初始化滤波器
// alpha: 0..1, 0 -> 无滤波（直接返回输入），1->完全保持历史值（不更新）
void imu_filter_init(float alpha);

// 重置滤波器状态为指定值
void imu_filter_reset(float initial_value);

// 使用原始 int16 原始值（驱动提供）进行转换并返回滤波后的浮点值
// 该函数内部会调用 imu660rx_acc_transition(raw) 进行单位转换
float imu660rx_acc_transition_filtered(int16_t raw);

// 使用原始 int16 陀螺仪值进行单位转换并返回滤波后的角速度
// 该函数内部会调用 imu660rx_gyro_transition(raw) 进行单位转换，单位为 deg/s
float imu660rx_gyro_transition_filtered(int16_t raw);

// -------- 兼容别名（可选）--------
// 如果工程里还有旧的 imu660rb_*_transition_filtered 调用，为了少改代码，这里提供一个别名。
// 等你确认所有调用都迁移到 rx 后，可以把下面两行删掉。
#define imu660rb_acc_transition_filtered(raw)   imu660rx_acc_transition_filtered((raw))
#define imu660rb_gyro_transition_filtered(raw)  imu660rx_gyro_transition_filtered((raw))

// 直接返回上一次滤波的值
float imu_acc_get_filtered(void);

// 直接返回上一次滤波的陀螺角速度值（deg/s）
float imu_gyro_get_filtered(void);

#ifdef __cplusplus
}
#endif

#endif /* CODE_IMU_FILTER_H_ */
