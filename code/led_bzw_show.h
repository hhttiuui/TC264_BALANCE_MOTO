/*
 * led_bzw_show.h
 *
 *  Created on: 2026-01-20
 *      Author: hhttiuui
 *
 *  LED/蜂鸣器/状态显示模块对外接口。
 */

#ifndef CODE_LED_BZW_SHOW_H_
#define CODE_LED_BZW_SHOW_H_

#include "zf_common_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

// 说明：
// - 用于“LED/蜂鸣器/屏幕”等方式显示当前状态与关键标志位。
// - 具体显示策略由 led_bzw_show_task_xxms() 内实现，可在主循环按固定周期调用。

typedef struct
{
    // 常用节拍/任务标志
    uint8 imu_flag_2ms;
    uint8 pid_flag;
    uint8 encoder_flag;

    // AHRS：yaw 延时置零完成标志（来自 ahrs_yaw_get_rel_deg 内部计数）
    // - 1：已完成；0：未完成
    uint8 yaw_delay_done;

    // 连接/通信状态（按你工程里已有的全局变量对接）
    uint8 wifi_ok;
    uint8 tcp_ok;

    // 计数器类（可用于观察丢帧/频率）
    uint32 count_imu;
    uint32 count_pid;
    uint32 count_encoder;

    // 比赛功能选择状态（来自 Ttop_funtion.c）
    // - 0：未选择/默认
    // - 1：转弯任务（funtion_kemu=1）
    // - 2：8字任务（funtion_kemu=2）
    uint8 funtion_kemu;
} led_bzw_show_status_t;

// 初始化（可选）：若不需要硬件初始化可留空实现
void led_bzw_show_init(void);

// 任务：建议 20ms/50ms 调用一次
// - status: 由调用方填充要显示的状态快照
void led_bzw_show_task_20ms(const led_bzw_show_status_t *status);

// 任务（封装版）：内部自行采集关键标志位并显示
// - 调用者只需按固定周期调用这个函数即可
void led_bzw_show_task_20ms_auto(void);

// 获取 LED 模块内部保存的“最新状态快照”（只读）
// - 该快照在 led_bzw_show_task_20ms_auto() 被调用时更新
const led_bzw_show_status_t* led_bzw_show_status_get(void);

#ifdef __cplusplus
}
#endif

#endif /* CODE_LED_BZW_SHOW_H_ */
