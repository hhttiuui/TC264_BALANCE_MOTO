/*
 * flash_storage.h
 *
 *  Created on: 2026-03-18
 *      Author: hhttiuui
 *
 *  Flash 存储读写模块（预留空接口）
 *
 *  说明：
 *  - 该文件用于后续实现 Flash 参数存储/读取（如 PID 参数、段表等）。
 *  - 当前仅提供空文件骨架，避免影响现有工程。
 */

#ifndef CODE_FLASH_STORAGE_H_
#define CODE_FLASH_STORAGE_H_

#include "zf_common_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

// TODO: 后续在此处声明 flash 存储初始化/读/写接口
// 例如：flash_storage_init(), flash_storage_load(), flash_storage_save() 等。

// ============================================================================
// Flash 参数存储（DFLASH）
//
// 说明：
// - 当前用于持久化两类数据：
//   1) `s_pid_mode_cfg`（PID/AHRS/阻尼等配置表）
//   2) `g_eight2_prog`（八字绕行段表）
// - 采用“整页读写 + 头部 + CRC 校验”的方式，避免误读与掉电/未初始化问题。
// - 写入前若检测到非擦除态，会执行擦除；请勿在高频循环中频繁调用 save。
// ============================================================================

// 加载：成功返回 1；失败（未初始化/校验失败）返回 0（保持 RAM 默认值不变）
uint8 flash_storage_load_pid_mode_cfg(void);
uint8 flash_storage_load_eight2_prog(void);
uint8 flash_storage_load_eight2_prog_ex1(void);
uint8 flash_storage_load_eight2_prog_ex2(void);

// 保存：成功返回 1；失败返回 0
uint8 flash_storage_save_pid_mode_cfg(void);
uint8 flash_storage_save_eight2_prog(void);
uint8 flash_storage_save_eight2_prog_ex1(void);
uint8 flash_storage_save_eight2_prog_ex2(void);

// ============================================================================
// Flash 参数存储 —— kemu1 固定 5 段配置
//
// 持久化内容：
// - g_ttop_kemu1_counts_per_meter
// - g_ttop_kemu1_seg_table[5]（每段 meters/speed/turn_prog_id）
// ============================================================================
uint8 flash_storage_load_kemu1_cfg(void);
uint8 flash_storage_save_kemu1_cfg(void);

// ============================================================================
// Flash 参数存储 —— 惯导航点表（INS Path）
//
// 说明：
// - 用于持久化示教录制的航点表 g_ins_path[] + g_ins_path_count。
// - 每个航点 = 2 × float (x, y) = 8 字节；最多 500 个 = 4000 字节。
// - 单页可容纳（1024 uint32 = 4096 字节，减去 16 字节 header = 4080 字节 > 4000）。
// - 写入时同时保存航点数 count 和 CRC 校验。
// ============================================================================

// 加载航点表：Flash → g_ins_path[] + g_ins_path_count
// 成功返回 1；失败（未保存/校验失败）返回 0（不覆盖 RAM）
uint8 flash_storage_load_ins_path(void);

// 保存航点表：g_ins_path[] + g_ins_path_count → Flash
// 成功返回 1；失败返回 0
uint8 flash_storage_save_ins_path(void);

// ============================================================================
// Flash 参数存储 —— 坡道速度参数（roll_slope_speed_cfg）
// ============================================================================
uint8 flash_storage_load_slope_cfg(void);
uint8 flash_storage_save_slope_cfg(void);

// ============================================================================
// Flash 参数存储 —— KEMU3 段表（g_ttop_kemu3_seg_table）
// ============================================================================
uint8 flash_storage_load_kemu3_seg(void);
uint8 flash_storage_save_kemu3_seg(void);

#ifdef __cplusplus
}
#endif

#endif /* CODE_FLASH_STORAGE_H_ */
