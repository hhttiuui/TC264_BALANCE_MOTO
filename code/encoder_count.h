/*
 * encoder_count.h
 *
 *  Created on: 2026-01-18
 *      Author: hhttiuui
 *
 *  编码器计数接口（示例：TIM5 demo）。
 */

#ifndef CODE_ENCODER_COUNT_H_
#define CODE_ENCODER_COUNT_H_

#include <stdint.h>

void encoder_tim5_demo_init(void);
void encoder_tim5_demo_update_5ms(void);
int16 encoder_get_demo5_delta_5ms(void);
int64 encoder_get_demo5_count_total(void);
void encoder_clear_demo5_count_total(void);
void encoder_clear_demo5_speed(void);


#endif /* CODE_ENCODER_COUNT_H_ */
