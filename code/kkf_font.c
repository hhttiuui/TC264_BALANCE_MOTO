/*
 * kkf_font.c
 *
 *  Created on: 2026-03-13
 *      Author: Copilot
 *
 *  重要说明：
 *  - ips200_show_chinese(x,y,16,buf,n,color) 需要 buf 为“已取模”的点阵数据。
 *  - 本文件当前提供的是“占位字模(全 0)”，用于完成工程接线与编译。
 *  - 你需要用 PCtoLCD2002 把对应汉字取模后，替换这些数组内容。
 *
 *  推荐取模参数（和 ips200_show_chinese() 注释一致）：
 *  - 16x16
 *  - 阴码
 *  - 逐行
 *  - 高位在前
 */

#include "kkf_font.h"

// 下面宏用来快速生成 N 个 16x16 汉字的“空白字模”占位：N * 32 字节
#define KKF_HZ16_BLANK(N) \
    { \
        /* 32*(N) bytes, all 0 */ \
    }

// 为了让编译器真的生成指定大小的数组，这里用显式长度 + 0 初始化。
// 你替换成真实字模时，可以把长度保留不变。

const uint8 kkf_hz16_koukoufeiche[KKF_FONT_HZ16_BYTES_PER_CHAR * 4u]    = {0};
const uint8 kkf_hz16_canshutiaoshi[KKF_FONT_HZ16_BYTES_PER_CHAR * 4u]   = {0};
const uint8 kkf_hz16_qingling[KKF_FONT_HZ16_BYTES_PER_CHAR * 2u]        = {0};
const uint8 kkf_hz16_moshi[KKF_FONT_HZ16_BYTES_PER_CHAR * 2u]           = {0};
const uint8 kkf_hz16_moshiyi[KKF_FONT_HZ16_BYTES_PER_CHAR * 3u]         = {0};
const uint8 kkf_hz16_moshier[KKF_FONT_HZ16_BYTES_PER_CHAR * 3u]         = {0};
const uint8 kkf_hz16_moshisan[KKF_FONT_HZ16_BYTES_PER_CHAR * 3u]        = {0};
const uint8 kkf_hz16_sudu[KKF_FONT_HZ16_BYTES_PER_CHAR * 2u]            = {0};
const uint8 kkf_hz16_mubiao_sudu[KKF_FONT_HZ16_BYTES_PER_CHAR * 4u]     = {0};
const uint8 kkf_hz16_xuanze[KKF_FONT_HZ16_BYTES_PER_CHAR * 2u]          = {0};

#undef KKF_HZ16_BLANK
