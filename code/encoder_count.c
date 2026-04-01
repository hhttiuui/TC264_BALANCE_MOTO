/*
 * encoder_count.c
 *
 *  Created on: 2026-01-18
 *      Author: hhttiuui
 *
 *  编码器计数示例（TIM/GPT12 编码器模式）：
 *  - 采用“间隔计数”方式：每 5ms 读取硬件计数并清零
 *  - 同时维护 delta_5ms 与 total_count，供上层里程/速度计算使用
 */


#include "zf_common_headfile.h"
#include "encoder_count.h"

// 正交编码器示例（GPT12）：TIM5
// A 相 -> P10_3，对应 TIM5_ENCODER_CH1_P10_3
// B 相 -> P10_1，对应 TIM5_ENCODER_CH2_P10_1
// 使用间隔计数器方法：每 5ms 读取硬件计数并清零，避免使用总计数差分导致的移除/复位问题
static volatile int16 s_enc5_delta_5ms = 0;
static volatile int32 s_enc5_speed_cps = 0;   // counts per second（以 encoder_get_count() 的“计数单位”为准）
static volatile int64 s_enc5_count_total = 0; // 总计数（与 encoder_get_demo5_delta_5ms() 符号约定一致）
volatile uint8 encoder_flag;

void encoder_tim5_demo_init(void)
{
    encoder_quad_init(TIM2_ENCODER, TIM2_ENCODER_CH1_P33_7, TIM2_ENCODER_CH2_P33_6);
    // 将硬件计数器清零，启动间隔计数模式
    encoder_clear_count(TIM2_ENCODER);
    s_enc5_delta_5ms = 0;
    s_enc5_speed_cps = 0;
    s_enc5_count_total = 0;
}

void encoder_tim5_demo_update_5ms(void)
{
    // 读取硬件计数（该计数是自上次清零以来的脉冲数），然后清零硬件计数器
    // 这样每次返回的就是“本 5ms 周期内的脉冲数”，无需维护总计数差分，避免复位/移除问题
    int16 cnt = encoder_get_count(TIM2_ENCODER);

    // 把本周期的脉冲数写入 delta（保留原来符号约定：使用 -delta 保持向后兼容）
    s_enc5_delta_5ms = -cnt;

    // 累加总计数（保持与 delta 一致的符号约定）
    s_enc5_count_total += (int64)s_enc5_delta_5ms;

    // 计算瞬时速度（counts per second）
    s_enc5_speed_cps = (int32)cnt * 200;

    // 清零硬件计数器，为下一周期统计做准备
    encoder_clear_count(TIM2_ENCODER);
}

void encoder_clear_demo5_count_total(void)
{
    s_enc5_count_total = 0;

}

void encoder_clear_demo5_speed(void)
{
    // 清除本地缓存的速度/增量变量，并重置硬件计数器以避免残留值
    s_enc5_delta_5ms = 0;
    s_enc5_speed_cps = 0;
    //encoder_clear_count(TIM2_ENCODER);
}

int16 encoder_get_demo5_delta_5ms(void)
{
    return s_enc5_delta_5ms;
}

int64 encoder_get_demo5_count_total(void)
{
    return s_enc5_count_total;
}

