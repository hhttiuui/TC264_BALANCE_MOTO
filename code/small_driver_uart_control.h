/*
 * small_driver_uart_control.h
 *
 *  Created on: 2026-01-19
 *      Author: hhttiuui
 *
 *  小驱动板 UART 控制接口声明。
 */

#ifndef CODE_SMALL_DRIVER_UART_CONTROL_H_
#define CODE_SMALL_DRIVER_UART_CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "zf_common_headfile.h"


#define SMALL_DRIVER_UART                       (UART_2        )
#define SMALL_DRIVER_BAUDRATE                   (460800        )
#define SMALL_DRIVER_RX                         (UART2_TX_P10_5)//UART2_TX_P10_5,UART3_TX_P15_6
#define SMALL_DRIVER_TX                         (UART2_RX_P10_6)//UART2_RX_P10_6,UART3_RX_P15_7

typedef struct
{
    uint8 send_data_buffer[7];                  // 发送缓冲区

    uint8 receive_data_buffer[7];               // 接收缓冲区

    uint8 receive_data_count;                   // 接收计数

    uint8 sum_check_data;                       // 校验和

    int16 receive_left_speed_data;              // 接收到的左轮速度

    int16 receive_right_speed_data;             // 接收到的右轮速度

}small_device_value_struct;

extern small_device_value_struct motor_value;



void uart_control_callback(void);                                   // 串口接收回调：解析速度帧

void small_driver_set_duty(int16 left_duty, int16 right_duty);      // 设置左右轮占空比

void small_driver_get_speed(void);                                  // 请求速度回传

void small_driver_uart_init(void);                                  // 初始化串口并清零状态

#ifdef __cplusplus
}
#endif

#endif /* CODE_SMALL_DRIVER_UART_CONTROL_H_ */
