/*
 * kkf_font.h
 *
 *  Created on: 2026-03-13
 *      Author: Copilot
 *
 *  用途：为“扣扣飞车”菜单提供固定中文词条的 16x16 点阵数据。
 *  说明：ips200_show_chinese() 不做编码转换，要求传入已经取模好的点阵。
 *       这里先提供接口与占位字模；你可以用 PCtoLCD2002 按 16x16 / 阴码 / 逐行 / 高位在前 取模后替换。
 */

#ifndef _KKF_FONT_H_
#define _KKF_FONT_H_

#include "zf_common_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

// 每个汉字 16x16：16 行 * 2 字节/行 = 32 字节
#define KKF_FONT_HZ16_BYTES_PER_CHAR   (32u)

// 词条字模：按“汉字顺序”连续存放
extern const uint8 kkf_hz16_koukoufeiche[];      // "扣扣飞车" 4字
extern const uint8 kkf_hz16_canshutiaoshi[];     // "参数调试" 4字
extern const uint8 kkf_hz16_qingling[];          // "清零" 2字
extern const uint8 kkf_hz16_moshi[];             // "模式" 2字
extern const uint8 kkf_hz16_moshiyi[];           // "模式一" 3字
extern const uint8 kkf_hz16_moshier[];           // "模式二" 3字
extern const uint8 kkf_hz16_moshisan[];          // "模式三" 3字
extern const uint8 kkf_hz16_sudu[];              // "速度" 2字
extern const uint8 kkf_hz16_mubiao_sudu[];       // "目标速度" 4字
extern const uint8 kkf_hz16_xuanze[];            // "选择" 2字

#ifdef __cplusplus
}
#endif

#endif
