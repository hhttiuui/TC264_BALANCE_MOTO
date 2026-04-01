/*
 * gps_key_led.h
 *
 *  Created on: 2026年1月28日
 *      Author: elle2
 */

#ifndef CODE_GPS_KEY_LED_H_
#define CODE_GPS_KEY_LED_H_

#include "zf_common_headfile.h"
#include "zf_device_ips200.h"
#include "zf_driver_gpio.h"

#define LED1                    (P20_9)
#define LED2                    (P20_8)
#define LED3                    (P21_5)
#define LED4                    (P21_4)
// 按键定义
// #define KEY1                    (P20_6)
// #define KEY2                    (P20_7)
// #define KEY3                    (P11_2)
// #define KEY4                    (P11_3)

// ==================== 功能宏定义 ====================
#define MAX_GPS_POINT_NUM       50      // 最多保存50个GPS定位
#define IPS_DISPLAY_ROW_HEIGHT  16      // IPS200每行显示高度（像素）
#define GPS_BAUDRATE            115200    // GPS串口波特率

// GPS定位结构体（存储经纬度）
typedef struct {
    double lon;          // 经度
    double lat;          // 纬度
    uint8  valid;        // 定位有效标志（1-有效，0-无效）
}gps_point_t_save;


void all_periph_init(void);
uint8 key_scan(gpio_pin_enum key_pin);
void save_current_gps_point(void);
void display_gps_points(void);
void led_toggle_control(void);

// ==================== 全局变量声明 ====================
extern gps_point_t_save gps_save_points[MAX_GPS_POINT_NUM]; // 保存的GPS定位
extern uint8      gps_saved_point_count;               // 已保存定位数量
extern uint8      led1_state, led2_state;              // LED状态（闪烁用）




#endif /* CODE_GPS_KEY_LED_H_ */
