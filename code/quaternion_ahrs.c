/*
 * quaternion_ahrs.c
 *
 *  Created on: 2026-01-19
 *      Author: hhttiuui
 *
 * Mahony AHRS（IMU-only, gyro+acc）四元数姿态解算。
 *
 * 参考思路：Mahony paper / 常见开源实现。
 * 这里实现为：
 * - 使用加速度计提供重力方向，和四元数预测的重力方向做叉积得到误差
 * - 误差经 PI（twoKp/twoKi）反馈回陀螺积分
 * - 积分更新四元数并归一化
 *
 * 注意：
 * - 未使用磁力计，yaw 会漂移。
 * - acc 必须在“主要是重力”的情况下效果最好；剧烈加速/振动时可考虑门限或降低 kp。
 */

#include "quaternion_ahrs.h"

// 便捷封装需要用到 IMU660RX 的数据与转换宏
#include "zf_device_imu660rx.h"

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

// ============================ Yaw wrap & zeroing ============================
// 说明：
// - Mahony(6轴)没有磁力计，yaw 会漂移；但工程上常用“相对yaw”：yaw_rel = wrap(yaw_deg - yaw0_deg)。
// - yaw0_deg 可以在“稳定后”自动锁定一次，也可以运行中手动重置。

static volatile float s_yaw_zero_deg = 0.0f;
static volatile uint8 s_yaw_zero_valid = 0u;

static uint16 s_delay_cnt = 0;//延迟计数
static uint8 s_delay_done = 0u;//置0标志位
static uint8 sd_zero_set_flag = 0u;

// 自动置零：需要连续稳定一段时间才锁定 yaw0
static volatile uint32 s_yaw_stable_cnt = 0u;

// ============================ Yaw unwrap (for yaw_rel) ============================
// yaw_rel_deg 本身通常是 wrap 到 [-180,180) 的角度；这里把它解包成连续角。
// 默认限幅到 ±720°（满足“记录正负720度”需求）。
#ifndef AHRS_YAW_REL_UNWRAP_LIMIT_DEG
#define AHRS_YAW_REL_UNWRAP_LIMIT_DEG   (720.0f)
#endif

static float s_yaw_rel_unwrap_last_deg = 0.0f;
static float s_yaw_rel_unwrap_deg = 0.0f;
static uint8 s_yaw_rel_unwrap_inited = 0u;

static inline float clamp_f(float x, float minv, float maxv)
{
    if (x > maxv) return maxv;
    if (x < minv) return minv;
    return x;
}

void ahrs_yaw_rel_unwrap_reset(float yaw_rel_deg)
{
    s_yaw_rel_unwrap_inited = 1u;
    s_yaw_rel_unwrap_last_deg = yaw_rel_deg;
    s_yaw_rel_unwrap_deg = yaw_rel_deg;
}

float ahrs_yaw_rel_unwrap_720_update_deg(float yaw_rel_deg)
{
    if (!s_yaw_rel_unwrap_inited)
    {
        ahrs_yaw_rel_unwrap_reset(yaw_rel_deg);
        return s_yaw_rel_unwrap_deg;
    }

    float dy = yaw_rel_deg - s_yaw_rel_unwrap_last_deg;

    // 处理跨越 -180/+180 的跳变（unwrap）
    if (dy > 180.0f)  dy -= 360.0f;
    if (dy < -180.0f) dy += 360.0f;

    s_yaw_rel_unwrap_deg += dy;
    s_yaw_rel_unwrap_last_deg = yaw_rel_deg;

    s_yaw_rel_unwrap_deg = clamp_f(s_yaw_rel_unwrap_deg,
                                   -(float)AHRS_YAW_REL_UNWRAP_LIMIT_DEG,
                                   (float)AHRS_YAW_REL_UNWRAP_LIMIT_DEG);
    return s_yaw_rel_unwrap_deg;
}

float ahrs_yaw_rel_unwrap_720_get_deg(void)
{
    return s_yaw_rel_unwrap_deg;
}

// 自动置零的默认参数（可按需要改成函数参数/宏）
#ifndef AHRS_YAW_AUTO_ZERO_STABLE_TIME_S
#define AHRS_YAW_AUTO_ZERO_STABLE_TIME_S   (0.30f)   // 稳定 300ms 后置零
#endif

#ifndef AHRS_YAW_AUTO_ZERO_GYRO_THRESH_DPS
#define AHRS_YAW_AUTO_ZERO_GYRO_THRESH_DPS (3.0f)    // |gyro| < 3 dps 认为稳定
#endif

// wrap 到 [-180, 180)
static float wrap_180_deg(float deg)
{
    while (deg >= 180.0f) deg -= 360.0f;
    while (deg <  -180.0f) deg += 360.0f;
    return deg;
}

// 对外接口：wrap 到 [-180, 180)
float ahrs_wrap_180_deg(float deg)
{
    return wrap_180_deg(deg);
}

// 对外接口：手动置零（下一次输出 yaw_rel 从 0 开始）
void ahrs_yaw_zero_set(float yaw_deg)
{
    s_yaw_zero_deg = yaw_deg;
    s_yaw_zero_valid = 1u;
    s_yaw_stable_cnt = 0u;
}

void ahrs_yaw_zero_clear(void)
{
    s_yaw_zero_valid = 0u;
    s_yaw_stable_cnt = 0u;
    s_yaw_zero_deg = 0.0f;
}

uint8 ahrs_yaw_zero_is_valid(void)
{
    return (uint8)(s_yaw_zero_valid ? 1u : 0u);
}

uint8 ahrs_yaw_delay_done_get(void)
{
    return (uint8)(s_delay_done ? 1u : 0u);
}

void ahrs_yaw_zero_request_set(void)//外部手动置零请求
{
    sd_zero_set_flag = 1u;
}

float ahrs_yaw_get_rel_deg(float yaw_deg)
{
    // 启动延时置零：默认约 1s 后把当前 yaw 作为 yaw0。
    // 注意：这里用“被调用次数”计时，默认假设外部 2ms 调用一次本函数（即 500 次≈1s）。
    // 如果你的调用频率不是 2ms，需要相应调整 500 这个阈值。
    

    if (!s_delay_done)
    {
        if (s_delay_cnt < 1000u)
        {
            s_delay_cnt++;
        }
        else
        {
            // 只在“尚未手动置零”的情况下做自动置零，避免覆盖用户手动行为
            if (!s_yaw_zero_valid)
            {
                ahrs_yaw_zero_set(yaw_deg);
            }
            s_delay_done = 1u;
        }
    }

    if (sd_zero_set_flag)
    {
        ahrs_yaw_zero_set(yaw_deg);
        sd_zero_set_flag = 0u;
    }

    if (!s_yaw_zero_valid)
    {
        // 未置零时：直接返回 wrap 后的 yaw
        return wrap_180_deg(yaw_deg);
    }
    return wrap_180_deg(yaw_deg - (float)s_yaw_zero_deg);
}

// 自动置零：若未置零且稳定，则锁定 yaw0。
// stable 判据：|gyro_z| < thresh（deg/s）连续满足 stable_time_s。
void ahrs_yaw_auto_zero_update(float dt_s, float gz_dps, float yaw_deg)
{
    if (s_yaw_zero_valid)
    {
        return;
    }

    if (dt_s <= 0.0f)
    {
        return;
    }

    float abs_gz = (gz_dps >= 0.0f) ? gz_dps : -gz_dps;
    if (abs_gz < (float)AHRS_YAW_AUTO_ZERO_GYRO_THRESH_DPS)
    {
        s_yaw_stable_cnt++;
    }
    else
    {
        s_yaw_stable_cnt = 0u;
    }

    // 不用浮点计时累加，避免误差；用计数阈值更直观
    uint32 need_cnt = (uint32)(AHRS_YAW_AUTO_ZERO_STABLE_TIME_S / dt_s + 0.5f);
    if (need_cnt < 1u) need_cnt = 1u;

    if (s_yaw_stable_cnt >= need_cnt)
    {
        ahrs_yaw_zero_set(yaw_deg);
    }
}

static float inv_sqrtf(float x)
{
    // 快速倒平方根不是必须，这里用 1/sqrtf 更稳。
    // 如果你追求速度，可替换为 fast inverse sqrt。
    return 1.0f / sqrtf(x);
}

void ahrs_mahony_init(ahrs_mahony_t *s, float kp, float ki)
{
    s->q0 = 1.0f;
    s->q1 = 0.0f;
    s->q2 = 0.0f;
    s->q3 = 0.0f;

    s->twoKp = 2.0f * kp;
    s->twoKi = 2.0f * ki;

    s->integralFBx = 0.0f;
    s->integralFBy = 0.0f;
    s->integralFBz = 0.0f;
}

void ahrs_mahony_update_imu(ahrs_mahony_t *s,
                            float gx, float gy, float gz,
                            float ax, float ay, float az,
                            float dt)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // 保护：dt 必须 > 0
    if (dt <= 0.0f)
    {
        return;
    }

    // 如果加速度有效（非全0），则用它修正姿态
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // 归一化加速度
        recipNorm = inv_sqrtf(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 估计重力方向（由四元数推出）
        // 这里用 half* 形式，后面误差也是 half*，与常见实现一致
        halfvx = s->q1 * s->q3 - s->q0 * s->q2;
        halfvy = s->q0 * s->q1 + s->q2 * s->q3;
        halfvz = s->q0 * s->q0 - 0.5f + s->q3 * s->q3;

        // 误差是估计重力与测量重力的叉积
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // 积分反馈（消除陀螺零偏）
        if (s->twoKi > 0.0f)
        {
            s->integralFBx += s->twoKi * halfex * dt;
            s->integralFBy += s->twoKi * halfey * dt;
            s->integralFBz += s->twoKi * halfez * dt;
            gx += s->integralFBx;
            gy += s->integralFBy;
            gz += s->integralFBz;
        }
        else
        {
            s->integralFBx = 0.0f;
            s->integralFBy = 0.0f;
            s->integralFBz = 0.0f;
        }

        // 比例反馈
        gx += s->twoKp * halfex;
        gy += s->twoKp * halfey;
        gz += s->twoKp * halfez;
    }

    // 积分四元数：q_dot = 0.5 * q ⊗ [0, gyro]
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);

    qa = s->q0;
    qb = s->q1;
    qc = s->q2;

    s->q0 += (-qb * gx - qc * gy - s->q3 * gz);
    s->q1 += (qa * gx + qc * gz - s->q3 * gy);
    s->q2 += (qa * gy - qb * gz + s->q3 * gx);
    s->q3 += (qa * gz + qb * gy - qc * gx);

    // 归一化四元数
    recipNorm = inv_sqrtf(s->q0 * s->q0 + s->q1 * s->q1 + s->q2 * s->q2 + s->q3 * s->q3);
    s->q0 *= recipNorm;
    s->q1 *= recipNorm;
    s->q2 *= recipNorm;
    s->q3 *= recipNorm;
}

void ahrs_mahony_get_euler_rad(const ahrs_mahony_t *s, float *roll, float *pitch, float *yaw)
{
    // 标准四元数->欧拉角（ZYX yaw-pitch-roll）
    // roll  = atan2(2(q0q1+q2q3), 1-2(q1^2+q2^2))
    // pitch = asin(2(q0q2-q3q1))
    // yaw   = atan2(2(q0q3+q1q2), 1-2(q2^2+q3^2))

    float sinr_cosp = 2.0f * (s->q0 * s->q1 + s->q2 * s->q3);
    float cosr_cosp = 1.0f - 2.0f * (s->q1 * s->q1 + s->q2 * s->q2);
    float r = atan2f(sinr_cosp, cosr_cosp);

    float sinp = 2.0f * (s->q0 * s->q2 - s->q3 * s->q1);
    float p;
    if (sinp >= 1.0f)
    {
        p = (float)(M_PI / 2.0);
    }
    else if (sinp <= -1.0f)
    {
        p = (float)(-M_PI / 2.0);
    }
    else
    {
        p = asinf(sinp);
    }

    float siny_cosp = 2.0f * (s->q0 * s->q3 + s->q1 * s->q2);
    float cosy_cosp = 1.0f - 2.0f * (s->q2 * s->q2 + s->q3 * s->q3);
    float y = atan2f(siny_cosp, cosy_cosp);

    if (roll)  *roll  = r;
    if (pitch) *pitch = p;
    if (yaw)   *yaw   = y;
}

void ahrs_mahony_get_euler_deg(const ahrs_mahony_t *s, float *roll, float *pitch, float *yaw)
{
    float r, p, y;
    ahrs_mahony_get_euler_rad(s, &r, &p, &y);

    const float rad2deg = (float)(180.0 / M_PI);
    if (roll)  *roll  = r * rad2deg;
    if (pitch) *pitch = p * rad2deg;
    if (yaw)   *yaw   = y * rad2deg;
}

void ahrs_mahony_update_imu660rx_euler_deg(ahrs_mahony_t *s,
                                          float dt_s,
                                          float *roll_deg,
                                          float *pitch_deg,
                                          float *yaw_deg)
{
    // gyro: deg/s -> rad/s
    const float deg2rad = 0.017453292519943295f;
    float gx = imu660rx_gyro_transition(imu660rx_gyro_x) * deg2rad;
    float gy = imu660rx_gyro_transition(imu660rx_gyro_y) * deg2rad;
    float gz = imu660rx_gyro_transition(imu660rx_gyro_z) * deg2rad;

    // acc: g（Mahony内部会归一化）
    float ax = imu660rx_acc_transition(imu660rx_acc_x);
    float ay = imu660rx_acc_transition(imu660rx_acc_y);
    float az = imu660rx_acc_transition(imu660rx_acc_z);

    ahrs_mahony_update_imu(s, gx, gy, gz, ax, ay, az, dt_s);
    ahrs_mahony_get_euler_deg(s, roll_deg, pitch_deg, yaw_deg);
}
