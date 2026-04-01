/*
 * gps_Azimuth.h
 *
 *  Created on: 2026年3月7日
 *      Author: elle2
 */

#ifndef CODE_GPS_AZIMUTH_H_
#define CODE_GPS_AZIMUTH_H_
#include "zf_common_headfile.h"
#include <math.h>

#ifndef FLOAT32
#define FLOAT32 float
//#define MAX_HEADING_NUM        (MAX_GPS_POINT_NUM - 1)  // 最多6个航向角
/**
 * @brief GPS坐标点数组索引定义
 * 说明：gps_points[点序号][0] = 经度(lon)，gps_points[点序号][1] = 纬度(lat)
 */
#endif
#define GPS_POINT_LON_INDEX    (0U)  // 经度索引（无符号数适配TC264规范）
#define GPS_POINT_LAT_INDEX    (1U)  // 纬度索引

//typedef struct {
//    FLOAT32 lon;
//    FLOAT32 lat;
//    uint8_t valid;
//} GPS_Point;
//uint8_t gps_point_count = 0U;                        // 已存点数量（0~7）

FLOAT32 gps_calculate_heading(FLOAT32 gps_points[2][2]);
uint8 gps_calc_multi_heading(FLOAT32 heading_result[], uint8 max_count);

#endif /* CODE_GPS_AZIMUTH_H_ */
