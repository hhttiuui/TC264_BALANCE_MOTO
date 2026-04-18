/*
 * gps_nav.c
 *
 *  Created on: 2026-03-28
 *      Author: hhttiuui
 *
 *  GPS 导航模块 —— 航向角坐标系转换 + GPS 点位跟踪导航
 *
 *  原理：
 *  小车预先存储若干 GPS 目标点（gps_save_points[]），导航时依次追踪每个点。
 *  每 100ms 更新一次（10Hz，与GPS模块输出频率匹配）：
 *    1. 用 GPS 实时位置和目标点位置，计算 GPS 坐标系下的目标航向（正北基准）
 *    2. 将 GPS 航向转换到 IMU 坐标系（上电前方基准）
 *    3. 与 IMU 实际航向 show_yaw_deg 做差，得到"需要转多少度"
 *    4. 正差 = 需要左转（逆时针），负差 = 需要右转（顺时针）
 *    5. 限速平滑后写入 g_yaw_target_deg，由 PID 链路执行转向
 */

#include "zf_common_headfile.h"
#include "gps_nav.h"
#include "gps_Azimuth.h"
#include "gps_key_led.h"
#include <math.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI (3.14159265358979323846F)
#endif

#define RAD_TO_DEG (180.0F / M_PI)
#define DEG_TO_RAD (M_PI / 180.0F)

/* ==================== 外部变量 ==================== */
extern float show_yaw_deg;              /* IMU 实测航向（IMU坐标系，±720° 连续角） */
extern volatile float g_yaw_target_deg; /* PID 目标航向（IMU坐标系） */
extern gnss_info_struct gnss;           /* GPS 实时数据（驱动层） */

/* ==================== 内部状态 ==================== */
static gps_nav_status_t s_nav;
static uint8_t s_nav_started = 0u;
static float s_yaw_cmd_smooth_deg = 0.0f; /* 限速平滑后的 yaw 输出 */
static double s_last_heading_lon = 0.0;
static double s_last_heading_lat = 0.0;
static uint8_t s_last_heading_pos_valid = 0u;

/* ================================================================
 *  wrap_180 —— 角度归一化到 [-180, +180)
 * ================================================================ */
static float wrap_180(float deg)
{
    while (deg >= 180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

/* ================================================================
 *  限速平滑（slew rate limiter）
 * ================================================================ */
static float slew_limit(float current, float target, float max_delta)
{
    float err = target - current;
    if (err > max_delta) return current + max_delta;
    if (err < -max_delta) return current - max_delta;
    return target;
}

/* ================================================================
 *  核心转换：GPS 航向 → IMU 航向
 *
 *  步骤：
 *  1. 去掉绝对参考：gps_heading - gps_heading_0  →  变成"相对上电方向"
 *  2. 翻转正方向：  取负号（GPS 顺时针正 → IMU 逆时针正）
 *  3. wrap 到 [-180, 180)
 *
 *  举例：
 *  上电朝北(gps_heading_0=0°)，GPS说当前朝东(gps_heading=90°，顺时针转了90°)
 *  → 相对角 = 90°，取负 = -90°（在IMU坐标系中 = 顺时针转了90°，与直觉一致）
 * ================================================================ */
float gps_nav_heading_to_imu(float gps_heading_deg, float gps_heading_0_deg)
{
    float rel = gps_heading_deg - gps_heading_0_deg;
    float imu = -rel;
    return wrap_180(imu);
}

/* ================================================================
 *  反向转换：IMU 航向 → GPS 航向（调试用）
 * ================================================================ */
float gps_nav_imu_to_heading(float imu_yaw_deg, float gps_heading_0_deg)
{
    float gps = gps_heading_0_deg + (-imu_yaw_deg);
    /* wrap 到 [0, 360) */
    while (gps >= 360.0f) gps -= 360.0f;
    while (gps < 0.0f)    gps += 360.0f;
    return gps;
}

/* ================================================================
 *  计算从当前 GPS 位置到目标点的 GPS 航向角（正北基准，0~360°）
 *
 *  与 gps_Azimuth.c 中 gps_calculate_heading() 类似，
 *  但此处输入是 double 类型的实时坐标和目标坐标。
 * ================================================================ */
static float calc_gps_bearing(double cur_lon, double cur_lat,
                               double tgt_lon, double tgt_lat)
{
    float dlon = (float)(tgt_lon - cur_lon) * DEG_TO_RAD;
    float lat1 = (float)cur_lat * DEG_TO_RAD;
    float lat2 = (float)tgt_lat * DEG_TO_RAD;

    float y = sinf(dlon) * cosf(lat2);
    float x = cosf(lat1) * sinf(lat2) - sinf(lat1) * cosf(lat2) * cosf(dlon);

    float bearing = atan2f(y, x) * RAD_TO_DEG;
    if (bearing < 0.0f) bearing += 360.0f;
    return bearing;
}


/* ================================================================
 *  GPS 两点间距离（米，简化平面近似，适用于短距离）
 * ================================================================ */
static float gps_distance_m(double lon1, double lat1, double lon2, double lat2)
{
    /* 经度方向：1° ≈ 111320m × cos(lat) */
    /* 纬度方向：1° ≈ 110540m */
    float avg_lat_rad = (float)((lat1 + lat2) * 0.5) * DEG_TO_RAD;
    float dx = (float)(lon2 - lon1) * 111320.0f * cosf(avg_lat_rad);
    float dy = (float)(lat2 - lat1) * 110540.0f;
    return sqrtf(dx * dx + dy * dy);
}

static uint8_t gps_pos_valid(double lon, double lat)
{
    if ((gnss.state == 0u) || (lon == 0.0) || (lat == 0.0))
    {
        return 0u;
    }
    return 1u;
}

static uint8_t gps_motion_heading_update(double cur_lon, double cur_lat, float *heading_deg)
{
    if (!gps_pos_valid(cur_lon, cur_lat))
    {
        s_last_heading_pos_valid = 0u;
        return 0u;
    }

    if (!s_last_heading_pos_valid)
    {
        s_last_heading_lon = cur_lon;
        s_last_heading_lat = cur_lat;
        s_last_heading_pos_valid = 1u;
        return 0u;
    }

    {
        float move_dist = gps_distance_m(s_last_heading_lon, s_last_heading_lat, cur_lon, cur_lat);
        if (move_dist < (float)GPS_NAV_MIN_MOVE_FOR_HEADING_M)
        {
            return 0u;
        }
    }

    *heading_deg = calc_gps_bearing(s_last_heading_lon, s_last_heading_lat, cur_lon, cur_lat);
    s_last_heading_lon = cur_lon;
    s_last_heading_lat = cur_lat;
    return 1u;
}

static void gps_nav_drift_correct_100ms(double cur_lon, double cur_lat)
{
    float gps_heading_deg;
    s_nav.gps_heading_valid = gps_motion_heading_update(cur_lon, cur_lat, &gps_heading_deg);
    s_nav.drift_error_deg = 0.0f;
    s_nav.drift_correction_deg = 0.0f;

    if (!s_nav.gps_heading_valid)
    {
        return;
    }

    s_nav.gps_motion_heading_deg = gps_heading_deg;

    if (!s_nav.heading_0_valid)
    {
        gps_nav_lock_heading_0(gps_heading_deg);
        s_nav.imu_heading_est_deg = gps_heading_deg;
        return;
    }

    s_nav.imu_heading_est_deg = gps_nav_imu_to_heading(show_yaw_deg, s_nav.gps_heading_0_deg);
    s_nav.drift_error_deg = wrap_180(gps_heading_deg - s_nav.imu_heading_est_deg);

    if (fabsf(s_nav.drift_error_deg) < (float)GPS_NAV_DRIFT_CORR_DEADZONE_DEG)
    {
        return;
    }

    s_nav.drift_correction_deg = s_nav.drift_error_deg * (float)GPS_NAV_DRIFT_CORR_GAIN_100MS;
    if (s_nav.drift_correction_deg > (float)GPS_NAV_DRIFT_CORR_MAX_STEP_DEG_100MS)
    {
        s_nav.drift_correction_deg = (float)GPS_NAV_DRIFT_CORR_MAX_STEP_DEG_100MS;
    }
    else if (s_nav.drift_correction_deg < -(float)GPS_NAV_DRIFT_CORR_MAX_STEP_DEG_100MS)
    {
        s_nav.drift_correction_deg = -(float)GPS_NAV_DRIFT_CORR_MAX_STEP_DEG_100MS;
    }

    s_nav.gps_heading_0_deg -= s_nav.drift_correction_deg;
    while (s_nav.gps_heading_0_deg >= 360.0f) s_nav.gps_heading_0_deg -= 360.0f;
    while (s_nav.gps_heading_0_deg < 0.0f)    s_nav.gps_heading_0_deg += 360.0f;
    s_nav.imu_heading_est_deg = gps_nav_imu_to_heading(show_yaw_deg, s_nav.gps_heading_0_deg);
}

/* ================================================================
 *  gps_nav_init —— 初始化
 * ================================================================ */
void gps_nav_init(void)
{
    s_nav.gps_heading_0_deg = 0.0f;
    s_nav.gps_target_heading_deg = 0.0f;
    s_nav.gps_motion_heading_deg = 0.0f;
    s_nav.imu_heading_est_deg = 0.0f;
    s_nav.imu_target_deg = 0.0f;
    s_nav.heading_delta_deg = 0.0f;
    s_nav.drift_error_deg = 0.0f;
    s_nav.drift_correction_deg = 0.0f;
    s_nav.target_point_idx = 0u;
    s_nav.nav_done = 0u;
    s_nav.heading_0_valid = 0u;
    s_nav.gps_heading_valid = 0u;

    s_nav_started = 0u;
    s_last_heading_lon = 0.0;
    s_last_heading_lat = 0.0;
    s_last_heading_pos_valid = 0u;
    s_yaw_cmd_smooth_deg = show_yaw_deg;
}

void gps_nav_start(void)
{
    s_nav.gps_target_heading_deg = 0.0f;
    s_nav.gps_motion_heading_deg = 0.0f;
    s_nav.imu_heading_est_deg = 0.0f;
    s_nav.imu_target_deg = 0.0f;
    s_nav.heading_delta_deg = 0.0f;
    s_nav.drift_error_deg = 0.0f;
    s_nav.drift_correction_deg = 0.0f;
    s_nav.target_point_idx = 0u;
    s_nav.nav_done = 0u;
    s_nav.heading_0_valid = 0u;
    s_nav.gps_heading_valid = 0u;

    s_last_heading_lon = 0.0;
    s_last_heading_lat = 0.0;
    s_last_heading_pos_valid = 0u;
    s_yaw_cmd_smooth_deg = show_yaw_deg;
    s_nav_started = 1u;
}

void gps_nav_stop(void)
{
    s_nav_started = 0u;
}

/* ================================================================
 *  gps_nav_lock_heading_0 —— 锁定上电初始航向
 *
 *  调用时机：GPS 信号有效且车辆静止时调用一次。
 *  该值是"GPS 航向 → IMU 航向"转换的旋转零点。
 * ================================================================ */
void gps_nav_lock_heading_0(float gps_heading_deg)
{
    s_nav.gps_heading_0_deg = gps_heading_deg;
    s_nav.heading_0_valid = 1u;
}

/* ================================================================
 *  gps_nav_task_100ms —— 100ms 周期导航任务（10Hz）
 *
 *  前提条件：
 *  - GPS 已存好目标点（gps_save_points[], gps_saved_point_count）
 *  - 已调用 gps_nav_lock_heading_0() 锁定初始航向
 *  - GPS 实时数据 gnss.longitude / gnss.latitude 持续更新
 *
 *  返回：0=导航中，1=所有目标点已到达
 * ================================================================ */
uint8_t gps_nav_task_100ms(void)
{
    double cur_lon = gnss.longitude;
    double cur_lat = gnss.latitude;
    float delta;
    float raw_target;
    float gps_target;
    float imu_target;
    float dist;
    uint8_t idx;

    if (!s_nav_started) return 0u;

    gps_nav_drift_correct_100ms(cur_lon, cur_lat);

    if (s_nav.nav_done) return 1u;
    if (!s_nav.heading_0_valid) return 0u;
    if (gps_saved_point_count == 0u) return 0u;
    if (!gps_pos_valid(cur_lon, cur_lat)) return 0u;
    if (s_nav.target_point_idx >= gps_saved_point_count)
    {
        s_nav.nav_done = 1u;
        return 1u;
    }

    idx = s_nav.target_point_idx;
    dist = gps_distance_m(cur_lon, cur_lat, gps_save_points[idx].lon, gps_save_points[idx].lat);
    if (dist < (float)GPS_NAV_ARRIVE_RADIUS_M)
    {
        s_nav.target_point_idx++;
        if (s_nav.target_point_idx >= gps_saved_point_count)
        {
            s_nav.nav_done = 1u;
            return 1u;
        }
        idx = s_nav.target_point_idx;
    }

    gps_target = calc_gps_bearing(cur_lon, cur_lat, gps_save_points[idx].lon, gps_save_points[idx].lat);
    s_nav.gps_target_heading_deg = gps_target;

    imu_target = gps_nav_heading_to_imu(gps_target, s_nav.gps_heading_0_deg);
    s_nav.imu_target_deg = imu_target;

    delta = wrap_180(imu_target - show_yaw_deg);
    s_nav.heading_delta_deg = delta;
    if (fabsf(delta) < (float)GPS_NAV_HEADING_DEADZONE_DEG)
    {
        delta = 0.0f;
    }

    raw_target = show_yaw_deg + delta;
    s_yaw_cmd_smooth_deg = slew_limit(s_yaw_cmd_smooth_deg, raw_target,
                                      (float)GPS_NAV_YAW_SLEW_RATE_DEG_100MS);
    g_yaw_target_deg = s_yaw_cmd_smooth_deg;
    return 0u;
}

/* ================================================================
 *  状态查询
 * ================================================================ */
const gps_nav_status_t* gps_nav_status_get(void)
{
    return &s_nav;
}

uint8_t gps_nav_is_done(void)
{
    return s_nav.nav_done;
}
