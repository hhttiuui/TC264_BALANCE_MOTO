/*
 * gps_storage.h
 * GPS实时存点模块 - 环形缓冲区存储GPS轨迹并绘制蓝色路径
 */
#ifndef CODE_GPS_STORAGE_H_
#define CODE_GPS_STORAGE_H_

#include "zf_common_headfile.h"

#define GPS_STORAGE_MAX_POINTS      1000u
#define GPS_STORAGE_JUMP_FILTER_M   5.0   /* 100ms内跳变超过此距离则丢弃 */

typedef struct {
    double lon;
    double lat;
} gps_storage_point_t;

void   gps_storage_start(void);
void   gps_storage_stop(void);
uint8  gps_storage_is_recording(void);
void   gps_storage_task_100ms(void);
void   gps_storage_draw(void);

#endif /* CODE_GPS_STORAGE_H_ */

