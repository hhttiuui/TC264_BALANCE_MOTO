/*
 * led_bzw_show.c
 *
 *  Created on: 2026-01-20
 *      Author: hhttiuui
 *
 *  LED/蜂鸣器/状态显示模块：
 *  - 通过 LED 显示关键状态（yaw_delay_done、功能选择等）
 *  - 内部保存最近一次状态快照，供其它模块查询
 */

#include "led_bzw_show.h"
#include "zf_common_headfile.h"
#include "quaternion_ahrs.h"

// 统一从顶层头文件获取 funtion_kemu / funtion_ready_flag 的声明，避免类型不一致(E289)
#include "Ttop_funtion.h"

// LED 模块内部保存的“最新状态快照”
static led_bzw_show_status_t s_led_status;

#define LED1                    (P20_9)
#define LED2                    (P20_8)
#define LED3                    (P21_5)
#define LED4                    (P21_4)

// 可按需要修改：例如用 LED 闪烁表示不同状态。
// 这里先给一个“占位实现”，避免影响现有逻辑。

void led_bzw_show_init(void)
{
    // 初始化 LED1~LED4 输出
    // 备注：部分板子 LED 为“低电平点亮”(active-low)，此处按现有工程逻辑配置。
    gpio_init(LED1, GPO, GPIO_LOW,  GPO_PUSH_PULL);
    gpio_init(LED2, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(LED3, GPO, GPIO_LOW,  GPO_PUSH_PULL);
    gpio_init(LED4, GPO, GPIO_HIGH, GPO_PUSH_PULL);
}

void led_bzw_show_task_20ms(const led_bzw_show_status_t *status)
{
    // 保护：空指针直接返回
    if (status == (void*)0)
    {
        return;
    }

    // 注意：多数板子 LED 是“低电平点亮”(active-low)。
    // 这里统一按“1=亮、0=灭”的语义写逻辑，再在输出时取反。
    // LED1：亮起表示 yaw 延时置零完成（yaw_delay_done=1）
    gpio_set_level(LED1, (uint8)(status->yaw_delay_done ? 0u : 1u));

    // LED2/LED3：显示 funtion_kemu
    // - funtion_kemu=1：亮 LED2
    // - funtion_kemu=2：亮 LED3
    // 其余：都灭
    gpio_set_level(LED2, (uint8)((status->funtion_kemu == 1u) ? 0u : 1u));
    gpio_set_level(LED3, (uint8)((status->funtion_kemu == 2u) ? 0u : 1u));
    gpio_set_level(LED4, (uint8)((status->funtion_kemu == 3u) ? 0u : 1u));

}

void led_bzw_show_task_20ms_auto(void)
{
    // 更新内部快照（目前按你的需求：只填 yaw_delay_done，其它置 0）
    s_led_status.imu_flag_2ms   = 0u;
    s_led_status.pid_flag       = 0u;
    s_led_status.encoder_flag   = 0u;
    s_led_status.yaw_delay_done = ahrs_yaw_delay_done_get();
    s_led_status.wifi_ok        = 0u;
    s_led_status.tcp_ok         = 0u;
    s_led_status.count_imu      = 0u;
    s_led_status.count_pid      = 0u;
    s_led_status.count_encoder  = 0u;
    s_led_status.funtion_kemu   = funtion_kemu;

    led_bzw_show_task_20ms(&s_led_status);
}

const led_bzw_show_status_t* led_bzw_show_status_get(void)
{
    return &s_led_status;
}
