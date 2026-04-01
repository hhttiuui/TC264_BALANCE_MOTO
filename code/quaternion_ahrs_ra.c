/*
 * quaternion_ahrs_ra.c
 *
  *  Created on: 2026-01-24
 *      Author: hhttiuui
 
 * Mahony AHRS（IMU-only, gyro+acc）四元数姿态解算。
 *
 * 说明：
 * - 本文件来自 `quaternion_ahrs.c` 的拷贝版本；
 * - 算法完全一致；
 * - 差异：便捷封装读取 IMU660RA 的全局变量；并且本文件内所有函数名追加 `_ra` 后缀避免冲突。
 */

#include "quaternion_ahrs_ra.h"

// 便捷封装需要用到 IMU660RA 的数据与转换宏
#include "zf_device_imu660ra.h"

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

// ============================ Yaw wrap & zeroing ============================

static volatile float s_yaw_zero_deg_ra = 0.0f;
static volatile uint8 s_yaw_zero_valid_ra = 0u;

static uint16 s_delay_cnt_ra = 0;     // 延迟计数
static uint8 s_delay_done_ra = 0u;    // 置0标志位

// 自动置零：需要连续稳定一段时间才锁定 yaw0
static volatile uint32 s_yaw_stable_cnt_ra = 0u;

// ============================ Yaw unwrap (for yaw_rel) ============================

#ifndef AHRS_YAW_REL_UNWRAP_LIMIT_DEG
#define AHRS_YAW_REL_UNWRAP_LIMIT_DEG   (720.0f)
#endif

static float s_yaw_rel_unwrap_last_deg_ra = 0.0f;
static float s_yaw_rel_unwrap_deg_ra = 0.0f;
static uint8 s_yaw_rel_unwrap_inited_ra = 0u;

static inline float clamp_f_ra(float x, float minv, float maxv)
{
    if (x > maxv) return maxv;
    if (x < minv) return minv;
    return x;
}

void ahrs_yaw_rel_unwrap_reset_ra(float yaw_rel_deg)
{
    s_yaw_rel_unwrap_inited_ra = 1u;
    s_yaw_rel_unwrap_last_deg_ra = yaw_rel_deg;
    s_yaw_rel_unwrap_deg_ra = yaw_rel_deg;
}

float ahrs_yaw_rel_unwrap_720_update_deg_ra(float yaw_rel_deg)
{
    if (!s_yaw_rel_unwrap_inited_ra)
    {
        ahrs_yaw_rel_unwrap_reset_ra(yaw_rel_deg);
        return s_yaw_rel_unwrap_deg_ra;
    }

    float dy = yaw_rel_deg - s_yaw_rel_unwrap_last_deg_ra;

    // 处理跨越 -180/+180 的跳变（unwrap）
    if (dy > 180.0f)  dy -= 360.0f;
    if (dy < -180.0f) dy += 360.0f;

    s_yaw_rel_unwrap_deg_ra += dy;
    s_yaw_rel_unwrap_last_deg_ra = yaw_rel_deg;

    s_yaw_rel_unwrap_deg_ra = clamp_f_ra(s_yaw_rel_unwrap_deg_ra,
                                         -(float)AHRS_YAW_REL_UNWRAP_LIMIT_DEG,
                                         (float)AHRS_YAW_REL_UNWRAP_LIMIT_DEG);
    return s_yaw_rel_unwrap_deg_ra;
}

float ahrs_yaw_rel_unwrap_720_get_deg_ra(void)
{
    return s_yaw_rel_unwrap_deg_ra;
}

#ifndef AHRS_YAW_AUTO_ZERO_STABLE_TIME_S
#define AHRS_YAW_AUTO_ZERO_STABLE_TIME_S   (0.30f)   // 稳定 300ms 后置零
#endif

#ifndef AHRS_YAW_AUTO_ZERO_GYRO_THRESH_DPS
#define AHRS_YAW_AUTO_ZERO_GYRO_THRESH_DPS (3.0f)    // |gyro| < 3 dps 认为稳定
#endif

// wrap 到 [-180, 180)
static float wrap_180_deg_ra(float deg)
{
    while (deg >= 180.0f) deg -= 360.0f;
    while (deg <  -180.0f) deg += 360.0f;
    return deg;
}

float ahrs_wrap_180_deg_ra(float deg)
{
    return wrap_180_deg_ra(deg);
}

void ahrs_yaw_zero_set_ra(float yaw_deg)
{
    s_yaw_zero_deg_ra = yaw_deg;
    s_yaw_zero_valid_ra = 1u;
    s_yaw_stable_cnt_ra = 0u;
}

void ahrs_yaw_zero_clear_ra(void)
{
    s_yaw_zero_valid_ra = 0u;
    s_yaw_stable_cnt_ra = 0u;
    s_yaw_zero_deg_ra = 0.0f;
}

uint8 ahrs_yaw_zero_is_valid_ra(void)
{
    return (uint8)(s_yaw_zero_valid_ra ? 1u : 0u);
}

uint8 ahrs_yaw_delay_done_get_ra(void)
{
    return (uint8)(s_delay_done_ra ? 1u : 0u);
}

float ahrs_yaw_get_rel_deg_ra(float yaw_deg)
{
    // 启动延时置零：默认约 1s 后把当前 yaw 作为 yaw0。
    // 注意：这里用“被调用次数”计时，默认假设外部 2ms 调用一次本函数（即 500 次≈1s）。
    // 如果你的调用频率不是 2ms，需要相应调整 500 这个阈值。

    if (!s_delay_done_ra)
    {
        if (s_delay_cnt_ra < 1500u)
        {
            s_delay_cnt_ra++;
        }
        else
        {
            // 只在“尚未手动置零”的情况下做自动置零，避免覆盖用户手动行为
            if (!s_yaw_zero_valid_ra)
            {
                ahrs_yaw_zero_set_ra(yaw_deg);
            }
            s_delay_done_ra = 1u;
        }
    }

    if (!s_yaw_zero_valid_ra)
    {
        // 未置零时：直接返回 wrap 后的 yaw
        return wrap_180_deg_ra(yaw_deg);
    }
    return wrap_180_deg_ra(yaw_deg - (float)s_yaw_zero_deg_ra);
}

void ahrs_yaw_auto_zero_update_ra(float dt_s, float gz_dps, float yaw_deg)
{
    if (s_yaw_zero_valid_ra)
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
        s_yaw_stable_cnt_ra++;
    }
    else
    {
        s_yaw_stable_cnt_ra = 0u;
    }

    uint32 need_cnt = (uint32)(AHRS_YAW_AUTO_ZERO_STABLE_TIME_S / dt_s + 0.5f);
    if (need_cnt < 1u) need_cnt = 1u;

    if (s_yaw_stable_cnt_ra >= need_cnt)
    {
        ahrs_yaw_zero_set_ra(yaw_deg);
    }
}

static float inv_sqrtf_ra(float x)
{
    return 1.0f / sqrtf(x);
}

void ahrs_mahony_init_ra(ahrs_mahony_ra_t *s, float kp, float ki)
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

void ahrs_mahony_update_imu_ra(ahrs_mahony_ra_t *s,
                               float gx, float gy, float gz,
                               float ax, float ay, float az,
                               float dt)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    if (dt <= 0.0f)
    {
        return;
    }

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        recipNorm = inv_sqrtf_ra(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        halfvx = s->q1 * s->q3 - s->q0 * s->q2;
        halfvy = s->q0 * s->q1 + s->q2 * s->q3;
        halfvz = s->q0 * s->q0 - 0.5f + s->q3 * s->q3;

        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

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

        gx += s->twoKp * halfex;
        gy += s->twoKp * halfey;
        gz += s->twoKp * halfez;
    }

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

    recipNorm = inv_sqrtf_ra(s->q0 * s->q0 + s->q1 * s->q1 + s->q2 * s->q2 + s->q3 * s->q3);
    s->q0 *= recipNorm;
    s->q1 *= recipNorm;
    s->q2 *= recipNorm;
    s->q3 *= recipNorm;
}

void ahrs_mahony_get_euler_rad_ra(const ahrs_mahony_ra_t *s, float *roll, float *pitch, float *yaw)
{
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

void ahrs_mahony_get_euler_deg_ra(const ahrs_mahony_ra_t *s, float *roll, float *pitch, float *yaw)
{
    float r, p, y;
    ahrs_mahony_get_euler_rad_ra(s, &r, &p, &y);

    const float rad2deg = (float)(180.0 / M_PI);
    if (roll)  *roll  = r * rad2deg;
    if (pitch) *pitch = p * rad2deg;
    if (yaw)   *yaw   = y * rad2deg;
}

void ahrs_mahony_update_imu660ra_euler_deg_ra(ahrs_mahony_ra_t *s,
                                              float dt_s,
                                              float *roll_deg,
                                              float *pitch_deg,
                                              float *yaw_deg)
{
    // gyro: deg/s -> rad/s
    const float deg2rad = 0.017453292519943295f;
    float gx = imu660ra_gyro_transition(imu660ra_gyro_x) * deg2rad;
    float gy = imu660ra_gyro_transition(imu660ra_gyro_y) * deg2rad;
    float gz = imu660ra_gyro_transition(imu660ra_gyro_z) * deg2rad;

    // acc: g（Mahony内部会归一化）
    float ax = imu660ra_acc_transition(imu660ra_acc_x);
    float ay = imu660ra_acc_transition(imu660ra_acc_y);
    float az = imu660ra_acc_transition(imu660ra_acc_z);

    ahrs_mahony_update_imu_ra(s, gx, gy, gz, ax, ay, az, dt_s);
    ahrs_mahony_get_euler_deg_ra(s, roll_deg, pitch_deg, yaw_deg);
}
