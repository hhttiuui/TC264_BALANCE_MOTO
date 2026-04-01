/*
 * small_driver_uart_control.c
 *
 *  Created on: 2026-01-19
 *      Author: hhttiuui
 *
 *  小驱动板 UART 控制：
 *  - 发送占空比（左右轮）
 *  - 接收/解析回传速度
 *
 *  说明：本文件为串口协议解析与发送封装，上层请按周期调用 get_speed/回调。
 */

#include "zf_common_headfile.h"
#include "small_driver_uart_control.h"

small_device_value_struct motor_value;      // 小驱动板通信用结构体


// ============================ 接收：串口中断回调解析 ============================
// 说明：
// - 从串口接收 7 字节数据帧，含帧头/类型/数据/校验
// - 解析到 motor_value.receive_*_speed_data
// - 本函数需在对应的串口接收中断/回调中调用
void uart_control_callback(void)
{
    uint8 receive_data;                                                                     // 临时接收字节

    if(uart_query_byte(SMALL_DRIVER_UART, &receive_data))                                   // 读取串口字节
    {
        if(receive_data == 0xA5 && motor_value.receive_data_buffer[0] != 0xA5)              // 帧头对齐（A5）
        {
            motor_value.receive_data_count = 0;                                             // 重新从帧头开始收
        }

        motor_value.receive_data_buffer[motor_value.receive_data_count ++] = receive_data;  // 保存到缓冲

        if(motor_value.receive_data_count >= 7)                                             // 收满一帧
        {
            if(motor_value.receive_data_buffer[0] == 0xA5)                                  // 校验帧头
            {

                motor_value.sum_check_data = 0;                                             // 计算校验和

                for(int i = 0; i < 6; i ++)
                {
                    motor_value.sum_check_data += motor_value.receive_data_buffer[i];       // 累加校验
                }

                if(motor_value.sum_check_data == motor_value.receive_data_buffer[6])        // 校验通过
                {

                    if(motor_value.receive_data_buffer[1] == 0x02)                          // 速度回传帧
                    {
                        motor_value.receive_left_speed_data  = (((int)motor_value.receive_data_buffer[2] << 8) | (int)motor_value.receive_data_buffer[3]);  // 左轮速度

                        motor_value.receive_right_speed_data = (((int)motor_value.receive_data_buffer[4] << 8) | (int)motor_value.receive_data_buffer[5]);  // 右轮速度
                    }

                    motor_value.receive_data_count = 0;                                     // 清计数

                    memset(motor_value.receive_data_buffer, 0, 7);                          // 清缓冲
                }
                else
                {
                    motor_value.receive_data_count = 0;                                     // 校验失败清计数

                    memset(motor_value.receive_data_buffer, 0, 7);                          // 清缓冲
                }
            }
            else
            {
                motor_value.receive_data_count = 0;                                         // 帧头错误清计数

                memset(motor_value.receive_data_buffer, 0, 7);                              // 清缓冲
            }
        }
    }
}

// ============================ 发送：设置左右轮占空比 ============================
// left_duty/right_duty：范围 -10000 ~ 10000（正负表示方向，具体以驱动板固件为准）
void small_driver_set_duty(int16 left_duty, int16 right_duty)
{
    motor_value.send_data_buffer[0] = 0xA5;                                         // 帧头

    motor_value.send_data_buffer[1] = 0X01;                                         // 功能码：设置占空比

    motor_value.send_data_buffer[2] = (uint8)((left_duty & 0xFF00) >> 8);           // 左占空比高字节

    motor_value.send_data_buffer[3] = (uint8)(left_duty & 0x00FF);                  // 左占空比低字节

    motor_value.send_data_buffer[4] = (uint8)((right_duty & 0xFF00) >> 8);          // 右占空比高字节

    motor_value.send_data_buffer[5] = (uint8)(right_duty & 0x00FF);                 // 右占空比低字节

    motor_value.send_data_buffer[6] = 0;                                            // 校验和

    for(int i = 0; i < 6; i ++)
    {
        motor_value.send_data_buffer[6] += motor_value.send_data_buffer[i];         // 累加校验
    }

    uart_write_buffer(SMALL_DRIVER_UART, motor_value.send_data_buffer, 7);          // 发送 7 字节指令帧
}

// ============================ 发送：请求速度回传 ============================
// 说明：需要驱动板固件周期回传速度数据（默认 10ms），本函数用于触发请求。
void small_driver_get_speed(void)
{
    motor_value.send_data_buffer[0] = 0xA5;                                         // 帧头

    motor_value.send_data_buffer[1] = 0X02;                                         // 功能码：请求速度

    motor_value.send_data_buffer[2] = 0x00;

    motor_value.send_data_buffer[3] = 0x00;

    motor_value.send_data_buffer[4] = 0x00;

    motor_value.send_data_buffer[5] = 0x00;

    motor_value.send_data_buffer[6] = 0xA7;                                         // 固定校验（A5+02= A7）

    uart_write_buffer(SMALL_DRIVER_UART, motor_value.send_data_buffer, 7);          // 发送请求帧
}


// ============================ 内部：结构体初始化 ============================
void small_driver_init(void)
{
    memset(motor_value.send_data_buffer, 0, 7);                             // 清发送缓冲

    memset(motor_value.receive_data_buffer, 0, 7);                          // 清接收缓冲

    motor_value.receive_data_count          = 0;

    motor_value.sum_check_data              = 0;

    motor_value.receive_right_speed_data    = 0;

    motor_value.receive_left_speed_data     = 0;
}


// ============================ 对外：串口初始化 ============================
void small_driver_uart_init(void)
{
    uart_init(SMALL_DRIVER_UART, SMALL_DRIVER_BAUDRATE, SMALL_DRIVER_RX, SMALL_DRIVER_TX);      // 串口初始化

    uart_rx_interrupt(SMALL_DRIVER_UART, 1);                                                    // 使能串口接收中断

    small_driver_init();                                                                        // 结构体清零初始化

    small_driver_set_duty(0, 0);                                                                // 上电先发 0 占空比

    small_driver_get_speed();                                                                   // 请求实时速度回传
}














