/*
 * GPS.c
 *
 *  Created on: 2026年1月26日
 *      Author: elle2
 */

#include "GPS.h"

#define IPS200_TYPE     (IPS200_TYPE_SPI)
void gps_display(void)
{
    ips200_init(IPS200_TYPE);
    if(gnss_flag)
           {
               gnss_flag = 0;

               gnss_data_parse();          //开始解析数据

               ips200_show_uint(   0, 16*0, gnss.time.year, 4);
               ips200_show_uint(  80, 16*0, gnss.time.month, 2);
               ips200_show_uint( 160, 16*0, gnss.time.day, 2);
               ips200_show_uint(   0, 16*1, gnss.time.hour, 2);
               ips200_show_uint(  80, 16*1, gnss.time.minute, 2);
               ips200_show_uint( 160, 16*1, gnss.time.second, 2);

               ips200_show_uint(   0, 16*2, gnss.state, 5);
               ips200_show_float(120, 16*2, gnss.latitude, 4, 6);
               ips200_show_float(  0, 16*3, gnss.longitude, 4, 6);
               ips200_show_float(120, 16*3, gnss.speed, 4, 6);
               ips200_show_float(  0, 16*4, gnss.direction, 4, 6);
               ips200_show_uint( 120, 16*4, gnss.satellite_used, 5);
               ips200_show_float(  0, 16*5, gnss.height, 4, 6);
           }
}

