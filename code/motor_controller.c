/*
 * motor_controller.c
 *
 *  Created on: 2026-01-19
 *      Author: hhttiuui
 *
 *  电机速度环控制（5ms）：
 *  - 读取编码器速度（counts/5ms）
 *  - EMA 滤波
 *  - 运行简易 PID（本地实现）
 *  - 算是small_driver_uart_control.c的一个封装
 */

#include "zf_common_headfile.h"
#include "motor_controller.h"
#include "small_driver_uart_control.h"

// pitch 角（deg），由 IMU 任务在 common_flie.c 中更新；这里用于“要倒下就加速”的策略
extern float show_pitch_deg;
// pitch 角速度（deg/s），由 IMU 任务在 common_flie.c 中更新。
extern float show_pitch_rate_dps_f;
// 舵机控制量（相对舵机中心点的偏移），在 common_flie.c 中由姿态环更新。
extern float ips_show_pid_out;


// ============================ 简易本地 PID 结构体（独立于 common_flie） ============================
typedef struct {
    float kp;
    float ki;
    float kd;
    float integrator;
    float prev_error;
    float prev_measurement;
    float out_min;
    float out_max;
    float integrator_min;
    float integrator_max;
} motor_pid_t;

static motor_pid_t g_motor_pid;

// ============================ 调试/状态量 ============================
volatile float motor_pid_out = 0.0f;
volatile float motor_ff_out = 0.0f;
volatile int16 motor_speed_raw_counts = 0;
volatile float motor_speed_filtered_counts = 0.0f;
volatile uint32 count_motor_pid = 0;
volatile float motor_pitch_boost_counts = 0.0f;

// 保存上一次输出的 duty（用于斜率限幅）
static int16 s_prev_duty = 0;

// 目标速度（单位：counts per 5ms）
static volatile int16 g_motor_target_counts = 0;
// 目标速度偏置（单位：counts per 5ms）
static volatile int16 g_motor_target_offset_counts = 0;

// 基于 |servo_out|（相对舵机中心0点）的瞬时力矩补偿：
// delta = k * (|servo_out| - threshold), |servo_out|>threshold 时生效
// 兼容原菜单字段：PIT_EN/PIT_TH/PIT_K（仅语义从 pitch 改为 servo）。
static volatile uint8 g_pitch_speed_boost_enable = 0u;
static volatile float g_pitch_speed_boost_threshold_deg = 0.3f;
static volatile float g_pitch_speed_boost_k_counts_per_deg = 1.0f;
static volatile float g_pitch_speed_boost_max_counts = 300.0f;

typedef enum
{
    MOTOR_BOOST_MODE_OFF = 0u,
    MOTOR_BOOST_MODE_DUTY = 1u,
    MOTOR_BOOST_MODE_PITCH = 2u,
    MOTOR_BOOST_MODE_ACC = 3u,
} motor_boost_mode_t;

static volatile uint8 g_motor_boost_mode = (uint8)MOTOR_BOOST_MODE_OFF;
static volatile float g_motor_boost_k_duty = 1.00f;
static volatile float g_motor_boost_k_pitch = 1.00f;
static volatile float g_motor_boost_k_acc = 1.00f;

// 启动控制逻辑已简化，软起动/推动启动相关类型已移除

// 清除 PID 积分等状态，避免在强推时产生过冲
static void motor_pid_reset(void)
{
    g_motor_pid.integrator = 0.0f;
    g_motor_pid.prev_error = 0.0f;
    g_motor_pid.prev_measurement = motor_speed_filtered_counts;
}
// 软起动/推动启动相关逻辑已移除（简化控制流程）

// 速度滤波系数（EMA），范围 0..1，越大越平滑
#ifndef MOTOR_SPEED_ALPHA
#define MOTOR_SPEED_ALPHA  (0.6f)
#endif

// PID 计算周期（秒）：与主循环的 5ms 触发一致
#ifndef MOTOR_PID_DT_S
#define MOTOR_PID_DT_S (0.005f)
#endif

// 占空比斜率限制参数（单位：百分比/秒），可由外部通过 setter 修改
static float g_duty_slope_percent_per_s = 500.0f;

// 速度前馈参数：u_ff = k_v * v_ref + k_s * sign(v_ref)
static float g_motor_ff_k_s = 0.0f;
static float g_motor_ff_k_v = -4.8f;

static float motor_signf(float x)
{
    if (x > 0.0f) return 1.0f;
    if (x < 0.0f) return -1.0f;
    return 0.0f;
}

static float motor_compute_feedforward(float setpoint)
{
    const float sgn = motor_signf(setpoint);
    if (sgn == 0.0f)
    {
        return 0.0f;
    }
    return g_motor_ff_k_v * setpoint + g_motor_ff_k_s * sgn;
}

// 外部接口：设置/获取占空比斜率限制
void motor_set_duty_slope_limit_percent_per_s(float percent_per_s)
{
    g_duty_slope_percent_per_s = percent_per_s;
}

float motor_get_duty_slope_limit_percent_per_s(void)
{
    return g_duty_slope_percent_per_s;
}

void motor_set_feedforward_gains(float k_s, float k_v)
{
    g_motor_ff_k_s = k_s;
    g_motor_ff_k_v = k_v;
}

void motor_get_feedforward_gains(float *k_s, float *k_v)
{
    if (k_s) *k_s = g_motor_ff_k_s;
    if (k_v) *k_v = g_motor_ff_k_v;
}

void motor_set_pid_gains(float kp, float ki, float kd)
{
    g_motor_pid.kp = kp;
    g_motor_pid.ki = ki;
    g_motor_pid.kd = kd;
}

void motor_get_pid_gains(float *kp, float *ki, float *kd)
{
    if (kp) *kp = g_motor_pid.kp;
    if (ki) *ki = g_motor_pid.ki;
    if (kd) *kd = g_motor_pid.kd;
}

void motor_set_pitch_speed_boost_enable(uint8 enable)
{
    if (enable != 0u)
    {
        if (g_motor_boost_mode == (uint8)MOTOR_BOOST_MODE_OFF)
        {
            g_motor_boost_mode = (uint8)MOTOR_BOOST_MODE_DUTY;
        }
        g_pitch_speed_boost_enable = 1u;
    }
    else
    {
        g_motor_boost_mode = (uint8)MOTOR_BOOST_MODE_OFF;
        g_pitch_speed_boost_enable = 0u;
    }
}

uint8 motor_get_pitch_speed_boost_enable(void)
{
    return (g_motor_boost_mode != (uint8)MOTOR_BOOST_MODE_OFF) ? 1u : 0u;
}

void motor_set_pitch_speed_boost_params(float threshold_abs_deg, float k_counts_per_deg)
{
    if (threshold_abs_deg < 0.0f) threshold_abs_deg = 0.0f;
    if (k_counts_per_deg < 0.0f) k_counts_per_deg = 0.0f;
    g_pitch_speed_boost_threshold_deg = threshold_abs_deg;
    g_pitch_speed_boost_k_counts_per_deg = k_counts_per_deg;
    // 兼容旧接口：历史上的 PIT_K 对应 DUTY 映射系数
    g_motor_boost_k_duty = k_counts_per_deg;
}

void motor_get_pitch_speed_boost_params(float *threshold_abs_deg, float *k_counts_per_deg)
{
    if (threshold_abs_deg) *threshold_abs_deg = g_pitch_speed_boost_threshold_deg;
    if (k_counts_per_deg) *k_counts_per_deg = g_motor_boost_k_duty;
}

void motor_set_boost_mode(uint8 mode)
{
    if (mode > (uint8)MOTOR_BOOST_MODE_ACC) mode = (uint8)MOTOR_BOOST_MODE_OFF;
    g_motor_boost_mode = mode;
    g_pitch_speed_boost_enable = (mode != (uint8)MOTOR_BOOST_MODE_OFF) ? 1u : 0u;
}

uint8 motor_get_boost_mode(void)
{
    return g_motor_boost_mode;
}

void motor_set_boost_map_k(float duty_k, float pitch_k, float acc_k)
{
    if (duty_k < 0.0f) duty_k = 0.0f;
    if (pitch_k < 0.0f) pitch_k = 0.0f;
    if (acc_k < 0.0f) acc_k = 0.0f;

    g_motor_boost_k_duty = duty_k;
    g_motor_boost_k_pitch = pitch_k;
    g_motor_boost_k_acc = acc_k;
    // 兼容旧接口读取
    g_pitch_speed_boost_k_counts_per_deg = duty_k;
}

void motor_get_boost_map_k(float *duty_k, float *pitch_k, float *acc_k)
{
    if (duty_k) *duty_k = g_motor_boost_k_duty;
    if (pitch_k) *pitch_k = g_motor_boost_k_pitch;
    if (acc_k) *acc_k = g_motor_boost_k_acc;
}

void motor_set_pitch_speed_boost_max_counts(float max_counts)
{
    if (max_counts < 0.0f) max_counts = 0.0f;
    g_pitch_speed_boost_max_counts = max_counts;
}

float motor_get_pitch_speed_boost_max_counts(void)
{
    return g_pitch_speed_boost_max_counts;
}

float motor_get_pitch_speed_boost_counts(void)
{
    return motor_pitch_boost_counts;
}

static float motor_compute_pitch_speed_boost_counts(void)
{
    float input_abs = 0.0f;
    float k = 0.0f;

    if (g_motor_boost_mode == (uint8)MOTOR_BOOST_MODE_OFF)
    {
        return 0.0f;
    }

    if (g_motor_boost_mode == (uint8)MOTOR_BOOST_MODE_DUTY)
    {
        input_abs = fabsf(ips_show_pid_out);
        k = g_motor_boost_k_duty;
    }
    else if (g_motor_boost_mode == (uint8)MOTOR_BOOST_MODE_PITCH)
    {
        input_abs = fabsf(show_pitch_deg);
        k = g_motor_boost_k_pitch;
    }
    else
    {
        input_abs = fabsf(show_pitch_rate_dps_f);
        k = g_motor_boost_k_acc;
    }

    {
        float boost = k * input_abs;
        if (boost > g_pitch_speed_boost_max_counts) boost = g_pitch_speed_boost_max_counts;
        return boost;
    }
}

// 斜率限幅函数：根据全局参数与上次输出限制当前 duty 的变化，返回限制后的 duty
static int16 apply_duty_slope_limit(int16 duty)
{
    float max_delta_f = g_duty_slope_percent_per_s * 0.01f * 10000.0f * MOTOR_PID_DT_S;
    int16 max_delta = (int16)roundf(fabsf(max_delta_f));
    if (max_delta < 1) max_delta = 1;
    int16 delta = duty - s_prev_duty;
    if (delta > max_delta) duty = s_prev_duty + max_delta;
    else if (delta < -max_delta) duty = s_prev_duty - max_delta;
    s_prev_duty = duty;
    return duty;
}

// ============================ PID 默认参数 ============================
static void motor_pid_init_default(motor_pid_t *pid)
{
    pid->kp = 1.0f;
    pid->ki = 0.0f;
    pid->kd = 0.0f;
    pid->integrator = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->out_min = -1000.0f; // 输出映射到 duty 的范围
    pid->out_max = 1000.0f;
    pid->integrator_min = pid->out_min;
    pid->integrator_max = pid->out_max;
}

static float motor_pid_compute(motor_pid_t *pid, float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;
    float p = pid->kp * error;
    pid->integrator += 0.5f * pid->ki * dt * (error + pid->prev_error);
    if (pid->integrator > pid->integrator_max) pid->integrator = pid->integrator_max;
    if (pid->integrator < pid->integrator_min) pid->integrator = pid->integrator_min;
    float d = 0.0f;
    if (dt > 0.0f) {
        d = - pid->kd * (measurement - pid->prev_measurement) / dt;
    }
    float out = p + pid->integrator + d;
    if (out > pid->out_max) out = pid->out_max;
    if (out < pid->out_min) out = pid->out_min;
    pid->prev_error = error;
    pid->prev_measurement = measurement;
    return out;
}

void motor_pid_init(void)
{
    motor_set_target_counts_per_5ms(200);
    motor_pid_init_default(&g_motor_pid);
    // 默认增益；后续可考虑提供 setter 接口集中调参
    g_motor_pid.kp = 10.0f;
    g_motor_pid.ki = 10.0f;
    g_motor_pid.kd = 0.0f;

    // 输出限幅与积分限幅初始化（放这里以便集中配置）
    g_motor_pid.out_min = -5000.0f;  // 映射到 duty 的最小值
    g_motor_pid.out_max =  5000.0f;  // 映射到 duty 的最大值
    g_motor_pid.integrator_min = -1000.0f; // 可根据实际需要调整
    g_motor_pid.integrator_max =  1000.0f;

    motor_speed_filtered_counts = 0.0f;
    motor_pid_out = 0.0f;
    motor_ff_out = 0.0f;
    motor_pitch_boost_counts = 0.0f;
    motor_speed_raw_counts = 0;
    count_motor_pid = 0;
}

void motor_set_target_counts_per_5ms(int16 target_counts)
{
    g_motor_target_counts = target_counts;
}

void pitch_speed_change(int16 delta_counts)
{
    g_motor_target_offset_counts = delta_counts;
}

// 清除速度相关状态：原始 delta / 滤波值 / PID 积分与历史测量
// 同时调用编码器层的速度清零，确保测量端与控制端一致
void motor_clear_speed(void)
{
    motor_speed_raw_counts = 0;
    motor_speed_filtered_counts = 0.0f;
    motor_pitch_boost_counts = 0.0f;
    motor_pid_reset();
    // 清除编码器层的速度缓存并重置硬件计数器
    encoder_clear_demo5_speed();
}

void motor_acc(float threshold_deg, int16 speed_fall, int16 speed_safe)
{
    // 基于 5ms 调用周期的滞回/保持：避免在阈值附近来回跳速
    // - 进入阈值：|pitch| > threshold
    // - 退出阈值：|pitch| < threshold - hysteresis
    // - 不做保持时间：退出条件由 hysteresis 决定
	#ifndef MOTOR_ACC_HYST_DEG
	#define MOTOR_ACC_HYST_DEG  (1.0f)
	#endif

    const float abs_pitch = fabsf(show_pitch_deg);
    const float thr_on = (threshold_deg > 0.0f) ? threshold_deg : 0.0f;
    const float thr_off = (thr_on > MOTOR_ACC_HYST_DEG) ? (thr_on - MOTOR_ACC_HYST_DEG) : 0.0f;

    static uint8 s_fall_mode = 0u;

    if (!s_fall_mode)
    {
        // 安全模式：达到阈值则进入救车加速
        if (abs_pitch > thr_on)
        {
            s_fall_mode = 1u;
        }
    }
    else
    {
        // 救车模式：满足退出条件才回到安全速度（使用滞回避免抖动）
        if (abs_pitch < thr_off)
        {
            s_fall_mode = 0u;
        }
    }

    motor_set_target_counts_per_5ms(s_fall_mode ? speed_fall : speed_safe);
}

int16 motor_get_speed_counts_filtered(void)
{
    return (int16)roundf(motor_speed_filtered_counts);
}

void motor_control_task_5ms(void)
{
    // ============================ 读取编码器速度（counts/5ms） ============================
    int16 raw = encoder_get_demo5_delta_5ms();
    motor_speed_raw_counts = raw;

    // ============================ EMA 滤波 ============================
    motor_speed_filtered_counts = MOTOR_SPEED_ALPHA * motor_speed_filtered_counts + (1.0f - MOTOR_SPEED_ALPHA) * (float)raw;
    // 在获取并滤波当前速度后（软起动/推动启动逻辑已移除）
    
    // 目标与测量（单位相同）
    motor_pitch_boost_counts = motor_compute_pitch_speed_boost_counts();
    float setpoint = (float)g_motor_target_counts + (float)g_motor_target_offset_counts;
    if (setpoint < 0.0f) setpoint = 0.0f;
    float measurement = motor_speed_filtered_counts;

    // ============================ PID + 前馈 ============================
    // 不改 PI 参数，只在原有反馈控制上叠加速度前馈。
    float pid_out = -motor_pid_compute(&g_motor_pid, setpoint, measurement, MOTOR_PID_DT_S);
    float ff_out = motor_compute_feedforward(setpoint);
    float out = pid_out + ff_out;
    // 舵机补偿直接作用在最终输出端，提供瞬时力矩。
    {
        float sgn = motor_signf(out);
        if (sgn == 0.0f) sgn = motor_signf(setpoint);
        out += sgn * motor_pitch_boost_counts;
    }
    motor_pid_out = pid_out;
    motor_ff_out = ff_out;

    // ============================ 输出映射到占空比并发送 ============================
    int16 duty = (int16)roundf(out);
    // 限幅（防止意外）
    if (duty > 10000) duty = 10000;
    if (duty < -10000) duty = -10000;

    // 调用斜率限幅函数（基于全局可设置参数）
    duty = apply_duty_slope_limit(duty);

    show_duty = duty;

    // 这里将 duty 写到左电机，右电机保持 0
    small_driver_set_duty(0, duty);//左边左电机，右边右电机

    count_motor_pid++;
}
