/*
 * gps_nav_circle.c
 *
 *  单点绕圈 GPS 导航模块
 *  原理：计算当前位置到中心点的 bearing，加减 90° 得到切向航向，
 *        转换到 IMU 坐标系后写入 g_yaw_target_deg。
 */

#include "zf_common_headfile.h"
#include "gps_nav_circle.h"
#include "gps_nav.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI (3.14159265358979323846F)
#endif
#define RAD_TO_DEG (180.0F / M_PI)
#define DEG_TO_RAD (M_PI / 180.0F)

/* ==================== 外部变量 ==================== */
extern float show_yaw_deg;
extern volatile float g_yaw_target_deg;
extern gnss_info_struct gnss;

/* ==================== 内部状态 ==================== */
static gps_nav_circle_status_t s_cir;
static float   s_yaw_smooth = 0.0f;
static uint8_t s_calib_tick = 0u;
static double  s_start_lon = 0.0;
static double  s_start_lat = 0.0;
static uint8_t s_start_pos_valid = 0u;

/* ==================== 工具函数 ==================== */
static float cir_wrap_180(float deg)
{
    while (deg >= 180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

static float cir_slew(float cur, float tgt, float max_d)
{
    float e = tgt - cur;
    if (e > max_d) return cur + max_d;
    if (e < -max_d) return cur - max_d;
    return tgt;
}

static float cir_bearing(double lon1, double lat1, double lon2, double lat2)
{
    float dlon = (float)(lon2 - lon1) * DEG_TO_RAD;
    float la1 = (float)lat1 * DEG_TO_RAD;
    float la2 = (float)lat2 * DEG_TO_RAD;
    float y = sinf(dlon) * cosf(la2);
    float x = cosf(la1) * sinf(la2) - sinf(la1) * cosf(la2) * cosf(dlon);
    float b = atan2f(y, x) * RAD_TO_DEG;
    if (b < 0.0f) b += 360.0f;
    return b;
}

static float cir_dist_m(double lon1, double lat1, double lon2, double lat2)
{
    float avg_lat_rad = (float)((lat1 + lat2) * 0.5) * DEG_TO_RAD;
    float dx = (float)(lon2 - lon1) * 111320.0f * cosf(avg_lat_rad);
    float dy = (float)(lat2 - lat1) * 110540.0f;
    return sqrtf(dx * dx + dy * dy);
}

static uint8_t cir_pos_valid(void)
{
    return (gnss.state != 0u && gnss.longitude != 0.0 && gnss.latitude != 0.0) ? 1u : 0u;
}

/* ==================== 接口实现 ==================== */
void gps_nav_circle_init(void)
{
    memset(&s_cir, 0, sizeof(s_cir));
    s_cir.direction = (int8_t)GPS_CIRCLE_DEFAULT_DIR;
    s_cir.desired_radius_m = (float)GPS_CIRCLE_DEFAULT_RADIUS_M;
    s_yaw_smooth = 0.0f;
    s_calib_tick = 0u;
    s_start_pos_valid = 0u;
}

void gps_nav_circle_set_radius(float r_m)
{
    if (r_m < 0.1f) r_m = 0.1f;
    s_cir.desired_radius_m = r_m;
}

void gps_nav_circle_save_center(void)
{
    if (!cir_pos_valid()) return;
    s_cir.center_lon = gnss.longitude;
    s_cir.center_lat = gnss.latitude;
    s_cir.center_valid = 1u;
}

void gps_nav_circle_start(void)
{
    if (!s_cir.center_valid) return;
    s_cir.started = 1u;
    s_cir.calib_done = 0u;
    s_cir.heading_0_valid = 0u;
    s_cir.heading_0_deg = 0.0f;
    s_calib_tick = 0u;
    s_start_pos_valid = 0u;
    s_yaw_smooth = show_yaw_deg;
    g_yaw_target_deg = 0.0f;
    /* 停止普通导航 */
    gps_nav_stop();
}

void gps_nav_circle_stop(void)
{
    s_cir.started = 0u;
    s_cir.calib_done = 0u;
}

void gps_nav_circle_toggle_dir(void)
{
    s_cir.direction = (s_cir.direction == 1) ? (int8_t)-1 : (int8_t)1;
}


/* ==================== 100ms 周期任务 ==================== */
uint8_t gps_nav_circle_task_100ms(void)
{
    if (!s_cir.started) return 0u;
    if (!cir_pos_valid()) return 0u;

    double cur_lon = gnss.longitude;
    double cur_lat = gnss.latitude;

    /* ---- 校准阶段：走直线 0.8m，用运动方向锁定 heading_0 ---- */
    if (!s_cir.calib_done)
    {
        s_calib_tick++;
        g_yaw_target_deg = 0.0f;  /* 保持直线 */

        if (!s_start_pos_valid)
        {
            s_start_lon = cur_lon;
            s_start_lat = cur_lat;
            s_start_pos_valid = 1u;
            return 0u;
        }

        float moved = cir_dist_m(s_start_lon, s_start_lat, cur_lon, cur_lat);

        if (moved >= (float)GPS_CIRCLE_CALIB_DIST_M)
        {
            /* 用起点到当前点的方向作为 heading_0 */
            float h0 = cir_bearing(s_start_lon, s_start_lat, cur_lon, cur_lat);
            s_cir.heading_0_deg = h0;
            s_cir.heading_0_valid = 1u;
            s_cir.calib_done = 1u;
            s_yaw_smooth = show_yaw_deg;
        }
        else if (s_calib_tick >= (uint8_t)GPS_CIRCLE_CALIB_TIMEOUT_TICKS)
        {
            /* 超时兜底 */
            s_cir.heading_0_deg = 0.0f;
            s_cir.heading_0_valid = 1u;
            s_cir.calib_done = 1u;
            s_yaw_smooth = show_yaw_deg;
        }
        return 0u;
    }

    /* ---- 绕圈阶段 ---- */
    /* 1. 计算当前位置到中心点的 bearing 和距离 */
    float bearing = cir_bearing(cur_lon, cur_lat, s_cir.center_lon, s_cir.center_lat);
    float dist = cir_dist_m(cur_lon, cur_lat, s_cir.center_lon, s_cir.center_lat);
    s_cir.bearing_deg = bearing;
    s_cir.dist_to_center_m = dist;

    /* 2. 径向修正：太近外推，太远内收 */
    float radius_error = dist - s_cir.desired_radius_m;
    s_cir.radius_error_m = radius_error;
    /* 符号：CW 时太远需负修正（向内），CCW 时太远需正修正（向内） */
    float radial_corr = -radius_error * (float)GPS_CIRCLE_RADIAL_KP * (float)s_cir.direction;
    /* 限幅 */
    if (radial_corr > (float)GPS_CIRCLE_RADIAL_MAX_DEG)
        radial_corr = (float)GPS_CIRCLE_RADIAL_MAX_DEG;
    if (radial_corr < -(float)GPS_CIRCLE_RADIAL_MAX_DEG)
        radial_corr = -(float)GPS_CIRCLE_RADIAL_MAX_DEG;
    s_cir.radial_correction_deg = radial_corr;

    /* 3. 切向 = bearing ± 90° + 径向修正 */
    float tangent = bearing + 90.0f * (float)s_cir.direction + radial_corr;
    if (tangent >= 360.0f) tangent -= 360.0f;
    if (tangent < 0.0f)    tangent += 360.0f;
    s_cir.tangent_heading_deg = tangent;

    /* 4. GPS 航向 → IMU 航向 */
    float imu_tgt = gps_nav_heading_to_imu(tangent, s_cir.heading_0_deg);
    s_cir.imu_target_deg = imu_tgt;

    /* 5. 航向差 + 死区 */
    float delta = cir_wrap_180(imu_tgt - show_yaw_deg);
    s_cir.heading_delta_deg = delta;
    if (fabsf(delta) < (float)GPS_CIRCLE_HEADING_DEADZONE_DEG)
        delta = 0.0f;

    /* 6. 限速平滑输出 */
    float raw = show_yaw_deg + delta;
    s_yaw_smooth = cir_slew(s_yaw_smooth, raw, (float)GPS_CIRCLE_YAW_SLEW_RATE_DEG_100MS);
    g_yaw_target_deg = s_yaw_smooth;

    return 0u;
}

uint8_t gps_nav_circle_is_started(void)
{
    return s_cir.started;
}

const gps_nav_circle_status_t* gps_nav_circle_get_status(void)
{
    return &s_cir;
}

