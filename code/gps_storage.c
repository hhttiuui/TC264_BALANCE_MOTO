/*
 * gps_storage.c
 * GPS实时存点模块实现
 */
#include "gps_storage.h"
#include "zf_device_gnss.h"
#include "zf_device_ips200.h"
#include <math.h>

#define SCREEN_W  240
#define SCREEN_H  320
#define DEG_TO_RAD  (0.017453292519943295)
#define M_PER_DEG_LAT  (110540.0)

static gps_storage_point_t s_buf[GPS_STORAGE_MAX_POINTS];
static uint16 s_head  = 0u;   /* 下一个写入位置 */
static uint16 s_count = 0u;   /* 当前有效点数 */
static uint8  s_recording = 0u;

void gps_storage_start(void)
{
    s_head  = 0u;
    s_count = 0u;
    s_recording = 1u;
}

void gps_storage_stop(void)
{
    s_recording = 0u;
}

uint8 gps_storage_is_recording(void)
{
    return s_recording;
}

/* 每100ms由定时中断调用 */
void gps_storage_task_100ms(void)
{
    double lon, lat, dist;
    uint16 prev_idx;

    if (!s_recording) return;
    if (gnss.state != 1) return;

    lon = gnss.longitude;
    lat = gnss.latitude;
    if (lon == 0.0 && lat == 0.0) return;

    /* 跳变过滤 */
    if (s_count > 0u)
    {
        prev_idx = (s_head + GPS_STORAGE_MAX_POINTS - 1u) % GPS_STORAGE_MAX_POINTS;
        dist = get_two_points_distance(s_buf[prev_idx].lat, s_buf[prev_idx].lon, lat, lon);
        if (dist > GPS_STORAGE_JUMP_FILTER_M) return;
    }

    /* 写入环形缓冲区 */
    s_buf[s_head].lon = lon;
    s_buf[s_head].lat = lat;
    s_head = (s_head + 1u) % GPS_STORAGE_MAX_POINTS;
    if (s_count < GPS_STORAGE_MAX_POINTS) s_count++;
}

/* 绘制GPS轨迹到屏幕 (米制坐标等比缩放) */
void gps_storage_draw(void)
{
    uint16 i, idx, start;
    double ref_lon, ref_lat, m_per_deg_lon;
    double xm, ym, x_min, x_max, y_min, y_max;
    double x_range, y_range, scale, scale_x, scale_y, off_x, off_y;
    int16 x1, y1, x2, y2;

    if (s_count == 0u)
    {
        ips200_show_string(60, 150, "Waiting GPS...");
        return;
    }

    /* 以第一个有效点为参考原点 */
    start = (s_head + GPS_STORAGE_MAX_POINTS - s_count) % GPS_STORAGE_MAX_POINTS;
    ref_lon = s_buf[start].lon;
    ref_lat = s_buf[start].lat;
    m_per_deg_lon = 111320.0 * cos(ref_lat * DEG_TO_RAD);

    /* 计算米制坐标范围 */
    x_min = x_max = 0.0;
    y_min = y_max = 0.0;
    for (i = 1u; i < s_count; i++)
    {
        idx = (start + i) % GPS_STORAGE_MAX_POINTS;
        xm = (s_buf[idx].lon - ref_lon) * m_per_deg_lon;
        ym = (s_buf[idx].lat - ref_lat) * M_PER_DEG_LAT;
        if (xm < x_min) x_min = xm;
        if (xm > x_max) x_max = xm;
        if (ym < y_min) y_min = ym;
        if (ym > y_max) y_max = ym;
    }

    x_range = x_max - x_min;
    y_range = y_max - y_min;
    if (x_range < 2.0) x_range = 2.0;
    if (y_range < 2.0) y_range = 2.0;

    /* 等比缩放，取较小scale，居中显示 */
    scale_x = (double)(SCREEN_W - 20) / x_range;
    scale_y = (double)(SCREEN_H - 20) / y_range;
    scale = (scale_x < scale_y) ? scale_x : scale_y;
    off_x = ((double)(SCREEN_W - 20) - x_range * scale) / 2.0 + 10.0;
    off_y = ((double)(SCREEN_H - 20) - y_range * scale) / 2.0 + 10.0;

    ips200_clear();

    /* 画蓝色连线 */
    xm = 0.0;
    ym = 0.0;
    x1 = (int16)(off_x + (xm - x_min) * scale);
    y1 = (int16)(off_y + (y_max - ym) * scale);

    for (i = 1u; i < s_count; i++)
    {
        idx = (start + i) % GPS_STORAGE_MAX_POINTS;
        xm = (s_buf[idx].lon - ref_lon) * m_per_deg_lon;
        ym = (s_buf[idx].lat - ref_lat) * M_PER_DEG_LAT;
        x2 = (int16)(off_x + (xm - x_min) * scale);
        y2 = (int16)(off_y + (y_max - ym) * scale);

        if (x1 >= 0 && x1 < SCREEN_W && y1 >= 0 && y1 < SCREEN_H &&
            x2 >= 0 && x2 < SCREEN_W && y2 >= 0 && y2 < SCREEN_H)
        {
            ips200_draw_line((uint16)x1, (uint16)y1, (uint16)x2, (uint16)y2, RGB565_BLUE);
        }
        x1 = x2;
        y1 = y2;
    }

    /* 画红色大点 (3x3) */
    for (i = 0u; i < s_count; i++)
    {
        idx = (start + i) % GPS_STORAGE_MAX_POINTS;
        xm = (s_buf[idx].lon - ref_lon) * m_per_deg_lon;
        ym = (s_buf[idx].lat - ref_lat) * M_PER_DEG_LAT;
        x1 = (int16)(off_x + (xm - x_min) * scale);
        y1 = (int16)(off_y + (y_max - ym) * scale);
        if (x1 >= 1 && x1 < (SCREEN_W - 1) && y1 >= 1 && y1 < (SCREEN_H - 1))
        {
            int16 dx, dy;
            for (dy = -1; dy <= 1; dy++)
                for (dx = -1; dx <= 1; dx++)
                    ips200_draw_point((uint16)(x1 + dx), (uint16)(y1 + dy), RGB565_RED);
        }
    }

    /* 显示点数 */
    ips200_show_string(0, 0, "pts:");
    ips200_show_uint(40, 0, s_count, 4);
}

