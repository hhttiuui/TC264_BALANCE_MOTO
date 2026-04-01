/*
 * Key.h
 *
 *  Created on: 2026-01-17
 *      Author: hhttiuui
 *
 *  按键驱动模块（消抖/采样）。
 */

#ifndef CODE_KEY_H_
#define CODE_KEY_H_

#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C" {
#endif
#define key1            (P13_0)//P11_3  P11_2  P20_7  P20_6
#define key2            (P13_3)
#define key3            (P13_1)
#define key4            (P13_2)

#define  KEY1           (gpio_get_level(key1))
#define  KEY2           (gpio_get_level(key2))
#define  KEY3           (gpio_get_level(key3))
#define  KEY4           (gpio_get_level(key4))

#define KEY1_PRES  1
#define KEY2_PRES  2
#define KEY3_PRES  3
#define KEY4_PRES  4

// 按键索引（用于 Key_Read(i)）
#define KEY_ID_1   (1u)
#define KEY_ID_2   (2u)
#define KEY_ID_3   (3u)
#define KEY_ID_4   (4u)

void  Key_init(void);        // 初始化

// 采样/读取分离
// - Key_Sample(): 每 5ms 调一次（或 1~5ms 均可）
// - Key_Read(i): 读取指定按键是否按下（稳定状态 0/1）
void  Key_Sample(void);
uint8 Key_Read(uint8 i);

extern volatile uint8 key_flag;

#ifdef __cplusplus
}
#endif

#endif
