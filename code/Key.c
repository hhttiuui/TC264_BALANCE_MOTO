/*
 * Key.c
 *
 *  Created on: 2026-01-17
 *      Author: hhttiuui
 *
 *  按键驱动模块：
 *  - 5 次采样为一个周期（建议 5ms 调一次 Key_Sample -> 25ms 周期）
 *  - 周期内统计“按下次数”，周期末计算稳定状态（>=4 次按下 -> 认为按下）
 */

#include "zf_common_headfile.h"
#include "Key.h"

volatile uint8 key_flag;

// 5 次采样为一个周期：周期内统计“按下次数”，周期末计算稳定状态（>=4 次按下 -> 认为按下）
static uint8 s_sample_tick = 0u;      // 0~4
static uint8 s_k1_cnt = 0u;
static uint8 s_k2_cnt = 0u;
static uint8 s_k3_cnt = 0u;
static uint8 s_k4_cnt = 0u;

static uint8 s_k1_down = 0u;
static uint8 s_k2_down = 0u;
static uint8 s_k3_down = 0u;
static uint8 s_k4_down = 0u;

void Key_init(void)
{
    gpio_init(key1, GPI, GPIO_HIGH, GPI_PULL_UP);               // 初始化 KEY1 输入 默认高电平 上拉输入
    gpio_init(key2, GPI, GPIO_HIGH, GPI_PULL_UP);               // 初始化 KEY2 输入 默认高电平 上拉输入
    gpio_init(key3, GPI, GPIO_HIGH, GPI_PULL_UP);               // 初始化 KEY3 输入 默认高电平 上拉输入
    gpio_init(key4, GPI, GPIO_HIGH, GPI_PULL_UP);               // 初始化 KEY4 输入 默认高电平 上拉输入

    // 初始化为“松开”状态
    s_sample_tick = 0u;
    s_k1_cnt = 0u;
    s_k2_cnt = 0u;
    s_k3_cnt = 0u;
    s_k4_cnt = 0u;
    s_k1_down = 0u;
    s_k2_down = 0u;
    s_k3_down = 0u;
    s_k4_down = 0u;
}

void Key_Sample(void)
{
    // KEYx==0 表示按下：周期内累计“按下次数”
    if (KEY1 == 0) s_k1_cnt++;
    if (KEY2 == 0) s_k2_cnt++;
    if (KEY3 == 0) s_k3_cnt++;
    if (KEY4 == 0) s_k4_cnt++;

    // 5 次采样为一个周期：第 5 次采样后更新稳定状态
    s_sample_tick++;
    if (s_sample_tick >= 5u)
    {
        s_sample_tick = 0u;

        // >=4/5 认为按下（抗抖）
        s_k1_down = (uint8)(s_k1_cnt >= 4u);
        s_k2_down = (uint8)(s_k2_cnt >= 4u);
        s_k3_down = (uint8)(s_k3_cnt >= 4u);
    s_k4_down = (uint8)(s_k4_cnt >= 4u);

        // 清零计数，进入下一周期
        s_k1_cnt = 0u;
        s_k2_cnt = 0u;
        s_k3_cnt = 0u;
        s_k4_cnt = 0u;
    }
}

// 读取指定按键是否“按下”（去抖后的稳定状态）
// i: 1/2/3 -> KEY1/KEY2/KEY3
// return: 1=按下 0=未按下
uint8 Key_Read(uint8 i)
{
    switch (i)
    {
        case 1u: return s_k1_down;
        case 2u: return s_k2_down;
        case 3u: return s_k3_down;
        case 4u: return s_k4_down;
        default: return 0u;
    }
}
