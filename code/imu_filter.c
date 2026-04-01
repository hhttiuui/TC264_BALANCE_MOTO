/*
 * imu_filter.c
 *
 *  Created on: 2026-01-18
 *      Author: hhttiuui
 *
 *  IMU 一阶低通滤波封装：
 *  - 先用驱动提供的 transition() 将原始 int16 转为物理量
 *  - 再对转换后的浮点值做 IIR 低通：y[n] = alpha*y[n-1] + (1-alpha)*x[n]
 */

#include "zf_common_headfile.h"
#include "imu_filter.h"
#include "zf_device_imu660rx.h"  // for imu660rx_*_transition macro

static float imu_filter_alpha = 0.9f; // 默认 alpha
static float imu_filter_y = 0.0f;
static int imu_filter_initialized = 0;

// 独立的陀螺滤波通道（单位：deg/s）
static float gyro_filter_y = 0.0f;
static int gyro_filter_initialized = 0;

void imu_filter_init(float alpha)
{
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    imu_filter_alpha = alpha;
    imu_filter_initialized = 0;  // 等待第一次 reset() 或第一次滤波调用自动初始化
    gyro_filter_initialized = 0; // 陀螺通道同样重新初始化
}

void imu_filter_reset(float initial_value)
{
    imu_filter_y = initial_value;
    imu_filter_initialized = 1;
}

float imu660rx_gyro_transition_filtered(int16_t raw)
{
    // 调用驱动提供的转换（宏）：把原始值转换为浮点角速度（deg/s）
    float x = imu660rx_gyro_transition(raw);

    if (!gyro_filter_initialized)
    {
        gyro_filter_y = x;
        gyro_filter_initialized = 1;
        return gyro_filter_y;
    }

    gyro_filter_y = imu_filter_alpha * gyro_filter_y + (1.0f - imu_filter_alpha) * x;
    return gyro_filter_y;
}

float imu660rx_acc_transition_filtered(int16_t raw)
{
    // 调用驱动提供的转换（宏）：把原始值转换为浮点加速度（单位与驱动一致，通常为 g）
    float x = imu660rx_acc_transition(raw);

    if (!imu_filter_initialized)
    {
    // 第一次调用时直接设置为当前值
        imu_filter_y = x;
        imu_filter_initialized = 1;
        return imu_filter_y;
    }

    //  (IIR): y[n] = alpha * y[n-1] + (1 - alpha) * x[n]
    imu_filter_y = imu_filter_alpha * imu_filter_y + (1.0f - imu_filter_alpha) * x;
    return imu_filter_y;
}

float imu_acc_get_filtered(void)
{
    return imu_filter_y;
}

float imu_gyro_get_filtered(void)
{
    return gyro_filter_y;
}
