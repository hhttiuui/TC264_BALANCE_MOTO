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
// - page 11 专用于 KM3 两段录制轨迹：slot 1=掉头，slot 2=S弯。
// - 其余配置使用各自的独立 Flash 页。
// - 采用“整页读写 + 头部 + CRC 校验”的方式，避免误读与掉电/未初始化问题。
// - 写入前若检测到非擦除态，会执行擦除；请勿在高频循环中频繁调用 save。
// ============================================================================

// 加载：成功返回 1；失败（未初始化/校验失败）返回 0（保持 RAM 默认值不变）
uint8 flash_storage_load_pid_mode_cfg(void);

// 保存：成功返回 1；失败返回 0
uint8 flash_storage_save_pid_mode_cfg(void);

// KM3 录制页：slot 1=掉头（最多400点），slot 2=S弯（最多500点）
uint8 flash_storage_save_kemu3_record(uint8 slot, const float *points, uint16 count);
uint8 flash_storage_load_kemu3_record(uint8 slot, float *points, uint16 max_points, uint16 *out_count);

// ============================================================================
// Flash 参数存储 —— 统一段表（KM3 + KM1 + MAIN + EX1 + EX2）page 6
// ============================================================================
uint8 flash_storage_load_kemu_all(void);
uint8 flash_storage_save_kemu_all(void);

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

// old KMU3 seg APIs removed — merged into flash_storage_save/load_kemu_all

// ============================================================================
// Flash 参数存储 —— RC Yaw 录制轨迹（50ms采样数组）
//
// 说明：
// - 使用 DFLASH 第 2 页存储。
// - yaw_points 为外部提供的 float 数组，count 为有效点数。
// - load 时按 max_points 做上限保护，实际点数通过 out_count 返回。
// ============================================================================
uint8 flash_storage_save_rc_yaw_record(const float *yaw_points, uint16 count, uint8 slot);
uint8 flash_storage_load_rc_yaw_record(float *yaw_points, uint16 max_points, uint16 *out_count, uint8 slot);

// ============================================================================
// Flash 参数存储 —— 杂项配置（MISC）
//
// 说明：
// - 集中存放目前未单独建页的“菜单可调但未持久化”的参数。
// - 后续新增杂项数据应优先塞入 reserved[]，无需新建 flash 页。
// - 结构体按 4 字节对齐，reserved 预留 128 字节以供扩展。
// ============================================================================

#ifndef FLASH_STORAGE_SECTOR_MISC
#define FLASH_STORAGE_SECTOR_MISC   (0u)
#endif
#ifndef FLASH_STORAGE_PAGE_MISC
#define FLASH_STORAGE_PAGE_MISC     (3u)   // 页号范围 1~11，3 目前空闲
#endif

typedef struct
{
    // ---- SPCH_DBG（3 项） ----
    uint8  fit_mode;                       // g_ttop_speed_change_fit_mode
    uint8  _pad0[3];
    float  lpf_alpha;                      // g_ttop_speed_change_lpf_alpha
    float  pitch_angle_lpf_alpha;          // g_pitch_angle_lpf_alpha
    float  pitch_zero_offset_deg;          // g_pitch_zero_offset_deg

    // ---- KEMU3_DBG 额外参数（5 项） ----
    uint8  ex2_loop_count;                 // g_ttop_kemu3_ex2_loop_count
    uint8  _pad1[3];
    float  kemu3_end_target_rel_deg;       // g_ttop_kemu3_end_seg.target_rel_deg
    float  kemu3_end_rate_dps;             // g_ttop_kemu3_end_seg.rate_dps
    float  yaw_zero_offset_deg;            // g_yaw_zero_offset_deg
    float  kemu3_yaw_ki_path;              // g_kemu3_yaw_ki_path
    int    kemu3_counts_per_meter;         // g_ttop_kemu3_counts_per_meter

    // ---- MOTOR_PID ----
    float  motor_kp;
    float  motor_ki;
    float  motor_kd;
    float  motor_pitch_boost_threshold;
    float  motor_pitch_boost_k;
    float  motor_pitch_boost_max;
    uint8  motor_pitch_boost_enable;
    uint8  _pad2[3];
    uint8  motor_boost_mode;
    uint8  _pad3[3];
    float  motor_boost_k_duty;
    float  motor_boost_k_pitch;
    float  motor_boost_k_acc;

    // ---- 舵机零点校准 ----
    float  servo_zero_offset_deg;          // g_servo_zero_offset_deg

    // ---- 预留扩展区（后续新增杂项直接使用此区域） ----
    uint8  reserved[124];

} flash_misc_cfg_t;

// 加载杂项配置：Flash → 全局变量
// 成功返回 1，失败（未保存/校验失败）返回 0
uint8 flash_storage_load_misc_cfg(void);

// 保存杂项配置：全局变量 → Flash
// 成功返回 1，失败返回 0
uint8 flash_storage_save_misc_cfg(void);

#ifdef __cplusplus
}
#endif

#endif /* CODE_FLASH_STORAGE_H_ */
