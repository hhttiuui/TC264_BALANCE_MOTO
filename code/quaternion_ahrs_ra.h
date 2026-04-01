/*
 * quaternion_ahrs_ra.h
 *
 *  Created on: 2026-01-24
 *      Author: hhttiuui
 *
 *  说明：
 *  - 这是 `quaternion_ahrs.c` 的 RA 版本拷贝。
 *  - 相同算法：Mahony AHRS（6轴：gyro+acc）。
 *  - 区别：便捷封装读取 IMU660RA（`zf_device_imu660ra.h`）的数据；并且本文件内所有函数名增加 `_ra` 后缀，避免与 RB 版本冲突。
 */

#ifndef CODE_QUATERNION_AHRS_RA_H_
#define CODE_QUATERNION_AHRS_RA_H_

#include "zf_common_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float q0, q1, q2, q3;      // 四元数（w, x, y, z）
    float twoKp;               // 2 * proportional gain
    float twoKi;               // 2 * integral gain
    float integralFBx;
    float integralFBy;
    float integralFBz;
} ahrs_mahony_ra_t;

// 初始化：kp/ki 为“Mahony增益”（经验：kp 0.5~5，ki 0~0.1，视噪声/漂移而定）
void ahrs_mahony_init_ra(ahrs_mahony_ra_t *s, float kp, float ki);

// 更新：输入 gyro(rad/s) 与 acc(任意等比例单位)，dt(s)
void ahrs_mahony_update_imu_ra(ahrs_mahony_ra_t *s,
                               float gx, float gy, float gz,
                               float ax, float ay, float az,
                               float dt);

// 输出：欧拉角（弧度）
void ahrs_mahony_get_euler_rad_ra(const ahrs_mahony_ra_t *s, float *roll, float *pitch, float *yaw);

// 输出：欧拉角（角度）
void ahrs_mahony_get_euler_deg_ra(const ahrs_mahony_ra_t *s, float *roll, float *pitch, float *yaw);

// 便捷封装：直接使用 IMU660RA 的全局变量作为输入源，内部完成单位转换与 Mahony 更新。
// - 调用前需保证已完成 imu660ra_init()。
// - 调用前请先执行 imu660ra_get_acc(); imu660ra_get_gyro(); 以刷新全局变量。
// - dt_s 为本次更新的周期（秒），必须尽量稳定。
// 输出：roll/pitch/yaw（角度）
void ahrs_mahony_update_imu660ra_euler_deg_ra(ahrs_mahony_ra_t *s,
                                              float dt_s,
                                              float *roll_deg,
                                              float *pitch_deg,
                                              float *yaw_deg);

// ============================ Yaw wrap & zeroing ============================

// 角度 wrap 到 [-180, 180)
float ahrs_wrap_180_deg_ra(float deg);

// 手动置零：设置 yaw0（单位：deg）
void ahrs_yaw_zero_set_ra(float yaw_deg);

// 清除置零状态（恢复为“未置零”）
void ahrs_yaw_zero_clear_ra(void);

// 当前是否已有 yaw0
uint8 ahrs_yaw_zero_is_valid_ra(void);

// 获取相对 yaw（单位：deg），内部会做 wrap
float ahrs_yaw_get_rel_deg_ra(float yaw_deg);

// ============================ Yaw unwrap (for yaw_rel) ============================

void ahrs_yaw_rel_unwrap_reset_ra(float yaw_rel_deg);
float ahrs_yaw_rel_unwrap_720_update_deg_ra(float yaw_rel_deg);
float ahrs_yaw_rel_unwrap_720_get_deg_ra(void);

// 启动延时置零是否已完成（1=已完成，0=未完成）
uint8 ahrs_yaw_delay_done_get_ra(void);

// 自动置零更新
void ahrs_yaw_auto_zero_update_ra(float dt_s, float gz_dps, float yaw_deg);

#ifdef __cplusplus
}
#endif

#endif /* CODE_QUATERNION_AHRS_RA_H_ */
