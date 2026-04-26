/*
 * gps_display.c
 *
 *  Created on: 2026年1月25日
 *      Author: elle2
 */
#include "gps_display.h"
#define IPS200_TYPE     (IPS200_TYPE_SPI)
#define DEG_TO_RAD                  (0.017453292519943295)
#define DISPLAY_MARGIN_PX           (12.0)
#define DISPLAY_MIN_RANGE_M         (2.0)
#define DISPLAY_VISUAL_ZOOM_GAIN    (1.8)

// 全局GPS点位数组（手动填写点位数据）
gps_point_t gps_points[GPS_POINT_MAX_NUM] = {0};
uint8 gps_draw_point_count = 0;  // 实际用于绘图的有效点位数量

// 实时轨迹点位数组（1s记录一个点）
gps_point_t gps_realtime_points[GPS_REALTIME_POINT_MAX_NUM] = {0};
uint8 gps_realtime_point_count = 0;  // 实时轨迹点位数量
uint8 gps_realtime_recording = 0;    // 实时轨迹记录标志

// 内部变量（缩放/偏移参数）
static double lon_min = 0, lon_max = 0;  // 经度极值
static double lat_min = 0, lat_max = 0;  // 纬度极值
static double scale_ratio = 0;           // 最终缩放系数
static int16 offset_x = 0, offset_y = 0; // 屏幕偏移量

static int16 clamp_to_screen_x(int32 value)
{
    if(value < 0) return 0;
    if(value >= (int32)SCREEN_WIDTH) return (int16)(SCREEN_WIDTH - 1);
    return (int16)value;
}

static int16 clamp_to_screen_y(int32 value)
{
    if(value < 0) return 0;
    if(value >= (int32)SCREEN_HEIGHT) return (int16)(SCREEN_HEIGHT - 1);
    return (int16)value;
}

static int32 round_to_i32(double value)
{
    return (value >= 0.0) ? (int32)(value + 0.5) : (int32)(value - 0.5);
}

void Buzzer_init(void)
{
    gpio_init(BUZZER_PIN, GPO, 0 ,GPO_PUSH_PULL);
}

void Buzzer_check(int time2)
{
    gpio_set_level(BUZZER_PIN,1);
    system_delay_ms(time2);
    gpio_set_level(BUZZER_PIN,0);

}

// 启动实时轨迹记录
void gps_realtime_start_recording(void)
{
    gps_realtime_point_count = 0;
    gps_realtime_recording = 1;
}

// 停止实时轨迹记录
void gps_realtime_stop_recording(void)
{
    gps_realtime_recording = 0;
}

// 添加实时轨迹点（1s调用一次）
// 循环缓冲区：超过200个点时，删除最早的点，保持最近200个点
void gps_realtime_add_point(double lon, double lat)
{
    if(!gps_realtime_recording) return;

    // 如果已满，将所有点向前移动一位（删除第一个点）
    if(gps_realtime_point_count >= GPS_REALTIME_POINT_MAX_NUM)
    {
        for(uint8 i = 0; i < GPS_REALTIME_POINT_MAX_NUM - 1; i++)
        {
            gps_realtime_points[i] = gps_realtime_points[i + 1];
        }
        gps_realtime_point_count = GPS_REALTIME_POINT_MAX_NUM - 1;
    }

    // 在末尾添加新点
    gps_realtime_points[gps_realtime_point_count].lon = lon;
    gps_realtime_points[gps_realtime_point_count].lat = lat;
    gps_realtime_point_count++;
}


void gps_calc_scroll_offset(void)
{
    uint8 total_count = gps_draw_point_count + gps_realtime_point_count;
    if(total_count == 0) return;

    double x_meter[GPS_POINT_MAX_NUM + GPS_REALTIME_POINT_MAX_NUM] = {0.0};
    double y_meter[GPS_POINT_MAX_NUM + GPS_REALTIME_POINT_MAX_NUM] = {0.0};

    // 使用第一个有效点作为参考点
    double ref_lon = (gps_draw_point_count > 0) ? gps_points[0].lon : gps_realtime_points[0].lon;
    double ref_lat = (gps_draw_point_count > 0) ? gps_points[0].lat : gps_realtime_points[0].lat;

    double meter_per_deg_lon = 111320.0 * cos(ref_lat * DEG_TO_RAD);
    double meter_per_deg_lat = 110540.0;

    double x_min = 0.0, x_max = 0.0, y_min = 0.0, y_max = 0.0;
    uint8 idx = 0;

    // 处理存点轨迹
    for(uint8 i = 0; i < gps_draw_point_count; i++)
    {
        x_meter[idx] = (gps_points[i].lon - ref_lon) * meter_per_deg_lon;
        y_meter[idx] = (gps_points[i].lat - ref_lat) * meter_per_deg_lat;

        if(idx == 0)
        {
            x_min = x_max = x_meter[idx];
            y_min = y_max = y_meter[idx];
        }
        else
        {
            if(x_meter[idx] < x_min) x_min = x_meter[idx];
            if(x_meter[idx] > x_max) x_max = x_meter[idx];
            if(y_meter[idx] < y_min) y_min = y_meter[idx];
            if(y_meter[idx] > y_max) y_max = y_meter[idx];
        }
        idx++;
    }

    // 处理实时轨迹
    for(uint8 i = 0; i < gps_realtime_point_count; i++)
    {
        x_meter[idx] = (gps_realtime_points[i].lon - ref_lon) * meter_per_deg_lon;
        y_meter[idx] = (gps_realtime_points[i].lat - ref_lat) * meter_per_deg_lat;

        if(idx == 0)
        {
            x_min = x_max = x_meter[idx];
            y_min = y_max = y_meter[idx];
        }
        else
        {
            if(x_meter[idx] < x_min) x_min = x_meter[idx];
            if(x_meter[idx] > x_max) x_max = x_meter[idx];
            if(y_meter[idx] < y_min) y_min = y_meter[idx];
            if(y_meter[idx] > y_max) y_max = y_meter[idx];
        }
        idx++;
    }

    lon_min = x_min;
    lon_max = x_max;
    lat_min = y_min;
    lat_max = y_max;

    double x_range = x_max - x_min;
    double y_range = y_max - y_min;
    if(x_range < DISPLAY_MIN_RANGE_M) x_range = DISPLAY_MIN_RANGE_M;
    if(y_range < DISPLAY_MIN_RANGE_M) y_range = DISPLAY_MIN_RANGE_M;

    double usable_width = (double)SCREEN_WIDTH - 2.0 * DISPLAY_MARGIN_PX;
    double usable_height = (double)SCREEN_HEIGHT - 2.0 * DISPLAY_MARGIN_PX;
    if(usable_width < 1.0) usable_width = 1.0;
    if(usable_height < 1.0) usable_height = 1.0;

    double scale_x = usable_width / x_range;
    double scale_y = usable_height / y_range;
    double base_scale = (scale_x < scale_y) ? scale_x : scale_y;

    // 应用视觉放大增益
    scale_ratio = base_scale * DISPLAY_VISUAL_ZOOM_GAIN;

    // 计算绝对最大缩放系数（仅保留1像素最小边距，防止边缘裁剪）
    double abs_max_scale_x = ((double)SCREEN_WIDTH  - 2.0) / x_range;
    double abs_max_scale_y = ((double)SCREEN_HEIGHT - 2.0) / y_range;
    double abs_max_scale = (abs_max_scale_x < abs_max_scale_y) ? abs_max_scale_x : abs_max_scale_y;
    if(scale_ratio > abs_max_scale) scale_ratio = abs_max_scale;

    double center_x_m = (x_min + x_max) * 0.5;
    double center_y_m = (y_min + y_max) * 0.5;
    double screen_center_x = ((double)SCREEN_WIDTH - 1.0) * 0.5;
    double screen_center_y = ((double)SCREEN_HEIGHT - 1.0) * 0.5;
    offset_x = (int16)round_to_i32(screen_center_x - center_x_m * scale_ratio);
    offset_y = (int16)round_to_i32(screen_center_y + center_y_m * scale_ratio);

    // 计算存点轨迹的屏幕坐标
    idx = 0;
    for(uint8 i = 0; i < gps_draw_point_count; i++)
    {
        int32 sx = round_to_i32(x_meter[idx] * scale_ratio + (double)offset_x);
        int32 sy = round_to_i32((double)offset_y - y_meter[idx] * scale_ratio);
        gps_points[i].screen_x = (uint16)clamp_to_screen_x(sx);
        gps_points[i].screen_y = (uint16)clamp_to_screen_y(sy);
        idx++;
    }

    // 计算实时轨迹的屏幕坐标
    for(uint8 i = 0; i < gps_realtime_point_count; i++)
    {
        int32 sx = round_to_i32(x_meter[idx] * scale_ratio + (double)offset_x);
        int32 sy = round_to_i32((double)offset_y - y_meter[idx] * scale_ratio);
        gps_realtime_points[i].screen_x = (uint16)clamp_to_screen_x(sx);
        gps_realtime_points[i].screen_y = (uint16)clamp_to_screen_y(sy);
        idx++;
    }
}

/**
 * @brief 绘制点位和连线
 */
void gps_draw_points_lines(uint16 point_color, uint16 line_color, uint8 point_size)
{
    if(gps_draw_point_count == 0) return;

    ips200_clear(); // 清屏

    // 绘制连线（相邻点连接）
    if(gps_draw_point_count >= 2)
    {
        for(uint8 i = 0; i < gps_draw_point_count - 1; i++)
        {
            ips200_draw_line(gps_points[i].screen_x, gps_points[i].screen_y,
                            gps_points[i+1].screen_x, gps_points[i+1].screen_y,
                            line_color);
        }
    }

    if(point_size == 0) point_size = 1;
    int16 radius = (int16)(point_size / 2);
    for(uint8 i = 0; i < gps_draw_point_count; i++)
    {
        int32 px = gps_points[i].screen_x;
        int32 py = gps_points[i].screen_y;
        for(int16 dx = -radius; dx <= radius; dx++)
        {
            for(int16 dy = -radius; dy <= radius; dy++)
            {
                int16 sx = clamp_to_screen_x((int32)(px + dx));
                int16 sy = clamp_to_screen_y((int32)(py + dy));
                ips200_draw_point((uint16)sx, (uint16)sy, point_color);
            }
        }
    }
}

/**
 * @brief 绘制实时轨迹线（绿色）
 */
void gps_draw_realtime_line(uint16 line_color)
{
    if(gps_realtime_point_count < 2) return;

    // 绘制实时轨迹连线（相邻点连接）
    for(uint8 i = 0; i < gps_realtime_point_count - 1; i++)
    {
        ips200_draw_line(gps_realtime_points[i].screen_x, gps_realtime_points[i].screen_y,
                        gps_realtime_points[i+1].screen_x, gps_realtime_points[i+1].screen_y,
                        line_color);
    }
}
