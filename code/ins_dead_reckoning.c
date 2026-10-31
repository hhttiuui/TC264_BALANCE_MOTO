/*
 * ins_dead_reckoning.c
 *
 *  Created on: 2026-03-24
 *      Author: hhttiuui
 *
 *  惯性导航 - 航迹推算（Dead Reckoning）模块
 *
 *  原理：
 *  每 5ms 读取编码器增量（距离）和 IMU yaw 角（方向），
 *  将距离投影到 X-Y 平面坐标系上，累加得到车辆实时位置。
 *
 *  数据来源：
 *  - 编码器增量：encoder_get_demo5_delta_5ms()（counts/5ms）
 *  - 里程标定：  ENCODER_COUNTS_PER_M = 18813（counts/m）
 *  - 航向角：    show_yaw_deg（度，由 AHRS Mahony 解算，imu_task_handler_2ms 中更新）
 *
 *  调用位置：
 *    在 ins_path_record_task_5ms() / ins_path_follow_task_5ms() 里调用
 *    （即 MODE9 示教录制、MODE10 Pure Pursuit 跟踪两个模式）。
 *    此时编码器任务（5ms）和姿态解算（1ms）都已完成本周期的更新。
 *    注：历史上曾在 ttop_pid_task_5ms() 中调用，该函数已不存在。
 */

#include "zf_common_headfile.h"
#include "ins_dead_reckoning.h"

/* PI 常量 */
#ifndef INS_DR_PI
#define INS_DR_PI  (3.14159265358979f)
#endif

/* ==================== 内部状态变量 ==================== */
static float s_pos_x       = 0.0f;   /* X 坐标（米） */
static float s_pos_y       = 0.0f;   /* Y 坐标（米） */
static float s_heading_deg = 0.0f;   /* 当前航向角（度） */
static float s_odometer_m  = 0.0f;   /* 累计里程（米，只增不减） */
static float s_speed_mps   = 0.0f;   /* 瞬时速度（m/s） */

/* 航向角来源：由 imu_task_handler_2ms() 写入，定义在 Menu.c */
extern float show_yaw_deg;

/* ================================================================
 *  ins_dr_init —— 初始化
 * ================================================================ */
void ins_dr_init(void)
{
    s_pos_x       = 0.0f;
    s_pos_y       = 0.0f;
    s_heading_deg = 0.0f;
    s_odometer_m  = 0.0f;
    s_speed_mps   = 0.0f;
}

/* ================================================================
 *  ins_dr_reset —— 位置/里程归零
 * ================================================================ */
void ins_dr_reset(void)
{
    s_pos_x      = 0.0f;
    s_pos_y      = 0.0f;
    s_odometer_m = 0.0f;
    s_speed_mps  = 0.0f;
    /* 注意：不重置 s_heading_deg，它始终跟随 show_yaw_deg */
}

/* ================================================================
 *  ins_dr_update_5ms —— 5ms 周期航迹推算
 * ================================================================ */
void ins_dr_update_5ms(void)
{
    /* ---- 1) 读取编码器增量，换算为行驶距离（米）---- */
    int16 enc_delta = encoder_get_demo5_delta_5ms();
    float ds = (float)enc_delta / (float)ENCODER_COUNTS_PER_M;

    /* ---- 2) 读取当前航向角并转换为弧度 ---- */
    s_heading_deg = show_yaw_deg;
    float heading_rad = s_heading_deg * (INS_DR_PI / 180.0f);

    /* ---- 3) 将距离沿航向分解到 X/Y 轴并累加 ----
     *  X 轴 = 初始前进方向，cos(0)=1 → 前进时 X 增加
     *  Y 轴 = 左侧方向，  sin(90°)=1 → 左转时 Y 增加
     */
    s_pos_x += ds * cosf(heading_rad);
    s_pos_y += ds * sinf(heading_rad);

    /* ---- 4) 累计里程（取绝对值，只增不减）---- */
    float abs_ds = (ds >= 0.0f) ? ds : -ds;
    s_odometer_m += abs_ds;

    /* ---- 5) 计算瞬时速度（m/s）----
     *  周期 = 5ms = 0.005s
     */
    s_speed_mps = ds / 0.005f;
}

/* ================================================================
 *  状态读取函数
 * ================================================================ */
float ins_dr_get_pos_x(void)       { return s_pos_x; }
float ins_dr_get_pos_y(void)       { return s_pos_y; }
float ins_dr_get_heading_deg(void) { return s_heading_deg; }
float ins_dr_get_odometer_m(void)  { return s_odometer_m; }
float ins_dr_get_speed_mps(void)   { return s_speed_mps; }
