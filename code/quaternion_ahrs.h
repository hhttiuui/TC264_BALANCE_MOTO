/*
 * quaternion_ahrs.h
 *
 *  Created on: 2026-01-19
 *      Author: hhttiuui
 *
 *  说明：
 *  - 这是一个轻量级四元数姿态解算（AHRS）模块，输入陀螺仪(角速度)与加速度计，输出四元数与欧拉角。
 *  - 算法：Mahony AHRS（6轴：gyro+acc）。不依赖磁力计，yaw 会随时间漂移。
 *
 *  单位约定：
 *  - gyro: rad/s
 *  - acc: 以 g 为单位或 m/s^2 均可（算法内部会归一化，只要三轴同单位即可）
 *  - dt: seconds
 */

#ifndef CODE_QUATERNION_AHRS_H_
#define CODE_QUATERNION_AHRS_H_

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
} ahrs_mahony_t;

// 初始化：kp/ki 为“Mahony增益”（经验：kp 0.5~5，ki 0~0.1，视噪声/漂移而定）
void ahrs_mahony_init(ahrs_mahony_t *s, float kp, float ki);

// 更新：输入 gyro(rad/s) 与 acc(任意等比例单位)，dt(s)
void ahrs_mahony_update_imu(ahrs_mahony_t *s,
                            float gx, float gy, float gz,
                            float ax, float ay, float az,
                            float dt);

// 输出：欧拉角（弧度）
void ahrs_mahony_get_euler_rad(const ahrs_mahony_t *s, float *roll, float *pitch, float *yaw);

// 输出：欧拉角（角度）
void ahrs_mahony_get_euler_deg(const ahrs_mahony_t *s, float *roll, float *pitch, float *yaw);

// 便捷封装：直接使用 IMU660RX 的全局变量作为输入源，内部完成单位转换与 Mahony 更新。
// - 调用前需保证已完成 imu660rx_init()。
// - 调用前请先执行 imu660rx_get_acc(); imu660rx_get_gyro(); 以刷新全局变量。
// - dt_s 为本次更新的周期（秒），必须尽量稳定。
// 输出：roll/pitch/yaw（角度）
void ahrs_mahony_update_imu660rx_euler_deg(ahrs_mahony_t *s,
                                          float dt_s,
                                          float *roll_deg,
                                          float *pitch_deg,
                                          float *yaw_deg);

// ============================ Yaw wrap & zeroing ============================
// 说明：
// - 6轴 AHRS 的 yaw 会漂移，但常用“相对yaw”：yaw_rel = wrap(yaw - yaw0)。
// - yaw0 可通过自动稳定置零或手动置零获得。

// 角度 wrap 到 [-180, 180)
float ahrs_wrap_180_deg(float deg);

// 手动置零：设置 yaw0（单位：deg）
void ahrs_yaw_zero_set(float yaw_deg);

// 清除置零状态（恢复为“未置零”）
void ahrs_yaw_zero_clear(void);

// 当前是否已有 yaw0
uint8 ahrs_yaw_zero_is_valid(void);

// 获取相对 yaw（单位：deg），内部会做 wrap
float ahrs_yaw_get_rel_deg(float yaw_deg);

// ============================ Yaw unwrap (for yaw_rel) ============================
// 说明：
// - yaw_rel_deg 通常是 wrap 到 [-180, 180) 的角度。
// - 为了得到连续角（不会在 180/-180 跳变），这里提供 yaw_rel 的解包（unwrap）。
// - 该连续角会被限幅到 [-720, +720]（可通过宏调整）。

// 复位 yaw_rel 连续角状态：建议在“延时置零完成/手动置零”后调用一次，避免首帧跳变。
void ahrs_yaw_rel_unwrap_reset(float yaw_rel_deg);

// 更新并获取 yaw_rel 连续角（deg，限幅到 ±720）
float ahrs_yaw_rel_unwrap_720_update_deg(float yaw_rel_deg);

// 获取最近一次更新得到的 yaw_rel 连续角（deg）
float ahrs_yaw_rel_unwrap_720_get_deg(void);

// 启动延时置零是否已完成（1=已完成，0=未完成）
// - 该标志由 ahrs_yaw_get_rel_deg() 内部维护（按其调用频率计数）。
uint8 ahrs_yaw_delay_done_get(void);

// 自动置零更新：当未置零且稳定时锁定 yaw0
// - dt_s：本次更新周期（秒）
// - gz_dps：z轴角速度（deg/s），用于稳定判据
// - yaw_deg：当前 yaw（deg）
void ahrs_yaw_auto_zero_update(float dt_s, float gz_dps, float yaw_deg);
void ahrs_yaw_zero_request_set(void);//外部手动置零请求
#ifdef __cplusplus
}
#endif

#endif /* CODE_QUATERNION_AHRS_H_ */
