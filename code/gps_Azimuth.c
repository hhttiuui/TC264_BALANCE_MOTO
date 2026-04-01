/*
 * gps_Azimuth.c
 *
 *  Created on: 2026年3月7日
 *      Author: elle2
 */
#include "gps_Azimuth.h"
#include "zf_common_headfile.h"
#include <math.h>


/* 适配TC264的圆周率定义（避免编译器未定义M_PI的问题） */
#ifndef M_PI
#define M_PI        (3.14159265358979323846F)
#endif

/* 弧度/角度转换常量（TC264单精度浮点优化） */
#define RAD_TO_DEG  (180.0F / M_PI)
#define DEG_TO_RAD  (M_PI / 180.0F)


//        gps_points[0] = (gps_point_t){gps_save_points[1].lon, gps_save_points[1].lat}; // 点位1
//        gps_points[1] = (gps_point_t){gps_save_points[2].lon, gps_save_points[2].lat}; // 点位2
//        gps_points[2] = (gps_point_t){gps_save_points[3].lon, gps_save_points[3].lat}; // 点位3
//        gps_points[3] = (gps_point_t){gps_save_points[4].lon, gps_save_points[4].lat}; // 点位4
//        gps_points[4] = (gps_point_t){gps_save_points[1].lon, gps_save_points[1].lat}; // 点位5
//        gps_points[5] = (gps_point_t){gps_save_points[1].lon, gps_save_points[1].lat}; // 点位6
//        gps_points[6] = (gps_point_t){gps_save_points[1].lon, gps_save_points[1].lat}; // 点位7
//
/**

 * @brief 计算两个GPS点之间的航向角（核心实现，适配TC264）
 */
FLOAT32 gps_calculate_heading(FLOAT32 gps_points[2][2])
{
    /* 提取你的十进制度经纬度（直接用，无需转换） */
    FLOAT32 lon1 = gps_points[0][GPS_POINT_LON_INDEX];  // 起点经度（如118.09332）
    FLOAT32 lat1 = gps_points[0][GPS_POINT_LAT_INDEX];  // 起点纬度（如24.58390）
    FLOAT32 lon2 = gps_points[1][GPS_POINT_LON_INDEX];  // 目标点经度
    FLOAT32 lat2 = gps_points[1][GPS_POINT_LAT_INDEX];  // 目标点纬度

    /* 1. 计算经度差（转弧度，TC264三角函数仅支持弧度） */
    FLOAT32 delta_lon = (lon2 - lon1) * DEG_TO_RAD;

    /* 2. 纬度转弧度 */
    FLOAT32 lat1_rad = lat1 * DEG_TO_RAD;
    FLOAT32 lat2_rad = lat2 * DEG_TO_RAD;

    /* 3. 航向角核心公式（WGS84坐标系，无冗余计算） */
    FLOAT32 y = sinf(delta_lon) * cosf(lat2_rad);  // 单精度三角函数，适配TC264
    FLOAT32 x = (cosf(lat1_rad) * sinf(lat2_rad)) -
                (sinf(lat1_rad) * cosf(lat2_rad) * cosf(delta_lon));

    /* 4. 反正切求航向角（弧度），再转角度 */
    FLOAT32 heading_rad = atan2f(y, x);
    FLOAT32 heading_deg = heading_rad * RAD_TO_DEG;

    /* 5. 修正为0~360°（避免负角度，适配小车转向逻辑） */
    if (heading_deg < 0.0F)
    {
        heading_deg += 360.0F;
    }

    return heading_deg;
}
/**
 * @brief 批量计算所有已保存GPS点的相邻航向角（0→1、1→2…）
 * @param heading_result 输出参数：存储航向角的数组（需提前分配空间）
 * @param max_count      数组最大容量（防越界）
 * @return 有效航向角数量（0 ~ gps_saved_point_count-1）
 */
uint8 gps_calc_multi_heading(FLOAT32 heading_result[], uint8 max_count)
{
    if (gps_saved_point_count < 2U)
    {
        return 0U;
    }

    uint8 count = gps_saved_point_count - 1;
    if (count > max_count)
    {
        count = max_count;
    }

    FLOAT32 temp_2points[2][2] = {0};
    uint8 i;

    for (i = 0; i < count; i++)
    {
        /* 将 double 类型的保存点转为 FLOAT32 传入计算函数 */
        temp_2points[0][GPS_POINT_LON_INDEX] = (FLOAT32)gps_save_points[i].lon;
        temp_2points[0][GPS_POINT_LAT_INDEX] = (FLOAT32)gps_save_points[i].lat;
        temp_2points[1][GPS_POINT_LON_INDEX] = (FLOAT32)gps_save_points[i + 1].lon;
        temp_2points[1][GPS_POINT_LAT_INDEX] = (FLOAT32)gps_save_points[i + 1].lat;

        heading_result[i] = gps_calculate_heading(temp_2points);
    }

    return count;
}
