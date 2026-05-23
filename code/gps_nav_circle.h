/*
 * gps_nav_circle.h
 *
 *  单点绕圈 GPS 导航模块
 *  功能：保存一个中心点，小车绕该点转圈
 *  启动后前 0.8m 保持 GPS 直线校准航向角
 */

#ifndef CODE_GPS_NAV_CIRCLE_H_
#define CODE_GPS_NAV_CIRCLE_H_

#include <stdint.h>
#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== 可调参数 ===================== */

/* 校准阶段最小位移（米），走够此距离后锁定 heading_0 */
#ifndef GPS_CIRCLE_CALIB_DIST_M
#define GPS_CIRCLE_CALIB_DIST_M             (0.8f)
#endif

/* 校准阶段最大等待 tick（100ms 单位），超时兜底 */
#ifndef GPS_CIRCLE_CALIB_TIMEOUT_TICKS
#define GPS_CIRCLE_CALIB_TIMEOUT_TICKS      (150u)
#endif

/* yaw 每 100ms 最大变化量（限速平滑） */
#ifndef GPS_CIRCLE_YAW_SLEW_RATE_DEG_100MS
#define GPS_CIRCLE_YAW_SLEW_RATE_DEG_100MS  (2.0f)
#endif

/* 航向死区（deg），差值绝对值小于此值不修正 */
#ifndef GPS_CIRCLE_HEADING_DEADZONE_DEG
#define GPS_CIRCLE_HEADING_DEADZONE_DEG     (3.0f)
#endif

/* 径向修正增益（deg/m），半径偏差 1m 修正多少度航向 */
#ifndef GPS_CIRCLE_RADIAL_KP
#define GPS_CIRCLE_RADIAL_KP                (3.0f)
#endif

/* 径向修正最大角度限幅（deg） */
#ifndef GPS_CIRCLE_RADIAL_MAX_DEG
#define GPS_CIRCLE_RADIAL_MAX_DEG           (20.0f)
#endif

/* 默认绕圈半径（米） */
#ifndef GPS_CIRCLE_DEFAULT_RADIUS_M
#define GPS_CIRCLE_DEFAULT_RADIUS_M         (3.0f)
#endif

/* 绕圈方向：1=顺时针（切向 = bearing + 90），-1=逆时针 */
#ifndef GPS_CIRCLE_DEFAULT_DIR
#define GPS_CIRCLE_DEFAULT_DIR              (1)
#endif

/* ===================== 状态结构体 ===================== */
typedef struct
{
    double  center_lon;
    double  center_lat;
    uint8_t center_valid;       /* 中心点是否已保存 */
    uint8_t started;            /* 绕圈是否已启动 */
    uint8_t calib_done;         /* 校准阶段是否完成 */
    uint8_t heading_0_valid;    /* heading_0 是否已锁定 */
    float   heading_0_deg;      /* GPS→IMU 转换零点 */
    float   dist_to_center_m;   /* 当前距中心点距离 */
    float   bearing_deg;        /* 当前位置到中心点的 bearing（GPS坐标系） */
    float   tangent_heading_deg;/* 切向目标航向（GPS坐标系） */
    float   imu_target_deg;     /* 转换后的 IMU 目标航向 */
    float   heading_delta_deg;  /* 航向差 */
    float   desired_radius_m;   /* 期望半径（启动时锁定） */
    float   radius_error_m;     /* 半径偏差 = dist - desired */
    float   radial_correction_deg; /* 径向修正量 */
    int8_t  direction;          /* 1=CW, -1=CCW */
} gps_nav_circle_status_t;

/* ===================== 对外接口 ===================== */

void    gps_nav_circle_init(void);
void    gps_nav_circle_save_center(void);
void    gps_nav_circle_start(void);
void    gps_nav_circle_stop(void);
void    gps_nav_circle_set_radius(float r_m);
void    gps_nav_circle_toggle_dir(void);
uint8_t gps_nav_circle_is_started(void);
uint8_t gps_nav_circle_task_100ms(void);
const gps_nav_circle_status_t* gps_nav_circle_get_status(void);

#ifdef __cplusplus
}
#endif

#endif /* CODE_GPS_NAV_CIRCLE_H_ */

