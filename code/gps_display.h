/*
 * gps_display.h
 *
 *  Created on: 2026年1月25日
 *      Author: elle2
 */

#ifndef CODE_GPS_DISPLAY_H_
#define CODE_GPS_DISPLAY_H_
#include "zf_common_headfile.h"
#include "zf_device_ips200.h"

#define BUZZER_PIN  (P33_10)

// 最大存储点位数量
#define GPS_POINT_MAX_NUM     50
// 实时轨迹最大点位数量（1s记录1个点，1000个点=约16.7分钟）
#define GPS_REALTIME_POINT_MAX_NUM  1000
// 屏幕显示区域（IPS200：240×320）
#define SCREEN_WIDTH          240
#define SCREEN_HEIGHT         320

// GPS点位结构体（仅存储原始经纬度/屏幕坐标）
typedef struct {
    double lon;          // 原始经度
    double lat;          // 原始纬度
    uint16 screen_x;     // 缩放后的屏幕X坐标
    uint16 screen_y;     // 缩放后的屏幕Y坐标
} gps_point_t;

// 全局GPS点位数组（直接手动填写点位，无需添加函数）
extern gps_point_t gps_points[GPS_POINT_MAX_NUM];
extern uint8 gps_draw_point_count;  // 实际用于绘图的有效点位数量

// 实时轨迹点位数组（1s记录一个点）
extern gps_point_t gps_realtime_points[GPS_REALTIME_POINT_MAX_NUM];
extern uint8 gps_realtime_point_count;  // 实时轨迹点位数量
extern uint8 gps_realtime_recording;    // 实时轨迹记录标志

void gps_calc_scroll_offset(void);
void gps_realtime_start_recording(void);
void gps_realtime_stop_recording(void);
void gps_realtime_add_point(double lon, double lat);
void Buzzer_init(void);
void Buzzer_check(int time2);
void gps_draw_points_lines(uint16 point_color, uint16 line_color, uint8 point_size);
void gps_draw_realtime_line(uint16 line_color);

#endif /* CODE_GPS_DISPLAY_H_ */
