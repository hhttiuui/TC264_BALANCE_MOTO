/*
 * flash_storage.c
 *
 *  Created on: 2026-03-18
 *      Author: hhttiuui
 *
 *  Flash 存储读写模块（DFLASH 参数持久化）
 *
 *  当前持久化两类数据：
 *  - `s_pid_mode_cfg`  : PID/AHRS/阻尼等配置表
 *  - `g_eight2_prog`   : 八字绕行段表（12 段）
 *
 *  设计要点：
 *  - 使用 zf_driver_flash 提供的“整页缓存 flash_union_buffer + 整页读写”接口。
 *  - 页内布局：Header + Payload + CRC32（IEEE/ZIP，多项式 0x04C11DB7，反射形式 0xEDB88320）。
 *  - 读取时校验 magic/version/长度/CRC，失败则不覆盖 RAM 默认值。
 *  - 写入前若检测到非擦除态则擦除，随后写入。
 */

#include "zf_common_headfile.h"
#include "flash_storage.h"

#include "common_flie.h"      // s_pid_mode_cfg, g_pitch_zero_offset_deg, g_yaw_zero_offset_deg
#include "top_8_turn_2.h"     // g_eight2_prog, ttop_eight2_seg_t
#include "ttop_kemu1.h"       // g_ttop_kemu1_*
#include "ttop_kemu3.h"       // g_ttop_kemu3_seg_table, g_ttop_kemu3_* (extras)
#include "ttop_speed_change.h"// g_ttop_speed_change_fit_mode, g_ttop_speed_change_lpf_alpha
#include "motor_controller.h" // motor_get_pid_gains, motor_set_pid_gains, etc.
#include "roll_slope_speed.h" // roll_slope_speed_cfg_t
#include "ins_path_follow.h"  // g_ins_path, g_ins_path_count, ins_waypoint_t
#include "zf_driver_flash.h"

// ============================
// 用户可根据需要调整存储位置
// ============================

// 建议：两类数据分开占用不同 page，互不影响
#ifndef FLASH_STORAGE_SECTOR_PID
#define FLASH_STORAGE_SECTOR_PID    (0u)
#endif
#ifndef FLASH_STORAGE_PAGE_PID
#define FLASH_STORAGE_PAGE_PID      (10u)
#endif

#ifndef FLASH_STORAGE_SECTOR_EIGHT2
#define FLASH_STORAGE_SECTOR_EIGHT2 (0u)
#endif
#ifndef FLASH_STORAGE_PAGE_EIGHT2
#define FLASH_STORAGE_PAGE_EIGHT2   (11u)
#endif

#ifndef FLASH_STORAGE_SECTOR_EIGHT2_EX1
#define FLASH_STORAGE_SECTOR_EIGHT2_EX1 (FLASH_STORAGE_SECTOR_EIGHT2)
#endif
#ifndef FLASH_STORAGE_PAGE_EIGHT2_EX1
#define FLASH_STORAGE_PAGE_EIGHT2_EX1   (8u)
#endif

#ifndef FLASH_STORAGE_SECTOR_EIGHT2_EX2
#define FLASH_STORAGE_SECTOR_EIGHT2_EX2 (FLASH_STORAGE_SECTOR_EIGHT2)
#endif
#ifndef FLASH_STORAGE_PAGE_EIGHT2_EX2
#define FLASH_STORAGE_PAGE_EIGHT2_EX2   (9u)
#endif

#ifndef FLASH_STORAGE_SECTOR_KEMU1
#define FLASH_STORAGE_SECTOR_KEMU1  (0u)
#endif
#ifndef FLASH_STORAGE_PAGE_KEMU1
#define FLASH_STORAGE_PAGE_KEMU1    (7u)
#endif

// 惯导航点表存储位置（使用 page 1，与其它数据分开）
#ifndef FLASH_STORAGE_SECTOR_INS
#define FLASH_STORAGE_SECTOR_INS    (0u)
#endif
#ifndef FLASH_STORAGE_PAGE_INS
#define FLASH_STORAGE_PAGE_INS      (1u)
#endif

// 坡道速度参数存储位置
#ifndef FLASH_STORAGE_SECTOR_SLOPE
#define FLASH_STORAGE_SECTOR_SLOPE  (0u)
#endif
#ifndef FLASH_STORAGE_PAGE_SLOPE
#define FLASH_STORAGE_PAGE_SLOPE    (5u)
#endif

// KEMU3 段表存储位置
#ifndef FLASH_STORAGE_SECTOR_KEMU3
#define FLASH_STORAGE_SECTOR_KEMU3  (0u)
#endif
#ifndef FLASH_STORAGE_PAGE_KEMU3
#define FLASH_STORAGE_PAGE_KEMU3    (6u)
#endif

// RC Yaw 录制轨迹存储位置（固定使用第 2 页）
#ifndef FLASH_STORAGE_SECTOR_RC_YAW
#define FLASH_STORAGE_SECTOR_RC_YAW (0u)
#endif
#ifndef FLASH_STORAGE_PAGE_RC_YAW
#define FLASH_STORAGE_PAGE_RC_YAW   (2u)
#endif
#ifndef FLASH_STORAGE_PAGE_RC_YAW_EX
#define FLASH_STORAGE_PAGE_RC_YAW_EX (4u)
#endif

typedef struct
{
	uint32 magic;
	uint16 version;
	uint16 payload_len_bytes;
	uint32 crc32;
} flash_storage_hdr_t;

#define FLASH_STORAGE_MAGIC_PID     (0x50494443u) // 'PIDC'
#define FLASH_STORAGE_MAGIC_EIGHT2  (0x45383250u) // 'E82P'
#define FLASH_STORAGE_MAGIC_EIGHT2_EX1 (0x45383231u) // 'E821'
#define FLASH_STORAGE_MAGIC_EIGHT2_EX2 (0x45383232u) // 'E822'
#define FLASH_STORAGE_MAGIC_KEMU1   (0x4B4D5531u) // 'KMU1'
#define FLASH_STORAGE_MAGIC_INS     (0x494E5350u) // 'INSP' (INS Path)
#define FLASH_STORAGE_MAGIC_SLOPE   (0x534C5043u) // 'SLPC'
#define FLASH_STORAGE_MAGIC_KEMU3   (0x4B4D5533u) // 'KMU3'
#define FLASH_STORAGE_MAGIC_MISC    (0x4D495343u) // 'MISC'
#define FLASH_STORAGE_MAGIC_RC_YAW  (0x52594346u) // 'RYCF'
#define FLASH_STORAGE_MAGIC_RC_YAW_P0 (0x52594330u) // 'RYC0'
#define FLASH_STORAGE_MAGIC_RC_YAW_P1 (0x52594331u) // 'RYC1'
#define FLASH_STORAGE_VERSION       (1u)

// Header 按 uint32 槽位存储：magic/version+len（打包）/crc
#define FLASH_STORAGE_HDR_WORDS     (4u)

static uint32 flash_storage_crc32_update(uint32 crc, uint8 data)
{
	crc ^= (uint32)data;
	for (uint8 i = 0u; i < 8u; i++)
	{
		if (crc & 1u) crc = (crc >> 1u) ^ 0xEDB88320u;
		else          crc = (crc >> 1u);
	}
	return crc;
}

static uint32 flash_storage_crc32(const void *data, uint32 len)
{
	const uint8 *p = (const uint8 *)data;
	uint32 crc = 0xFFFFFFFFu;

	for (uint32 i = 0u; i < len; i++)
	{
		crc = flash_storage_crc32_update(crc, p[i]);
	}

	return ~crc;
}

static uint8 flash_storage_write_blob(uint32 sector, uint32 page, uint32 magic, const void *payload, uint16 payload_len)
{
	const uint32 payload_words = (uint32)((payload_len + 3u) / 4u);
	const uint32 total_words = (uint32)FLASH_STORAGE_HDR_WORDS + payload_words;

	if (total_words > (uint32)EEPROM_PAGE_LENGTH)
	{
		return 0u;
	}

	// 若页已写入过数据，需要擦除
	if (flash_check(sector, page))
	{
		flash_erase_page(sector, page);
	}

	flash_buffer_clear();

	// Header
	flash_union_buffer[0].uint32_type = magic;
	flash_union_buffer[1].uint32_type = ((uint32)FLASH_STORAGE_VERSION << 16) | (uint32)payload_len;

	// Payload（按 32bit 拷贝；尾部不足 4 字节的部分用 0xFF 填充）
	{
		uint32 *dst = (uint32 *)&flash_union_buffer[FLASH_STORAGE_HDR_WORDS];
		const uint8 *src = (const uint8 *)payload;
		uint32 i;

		for (i = 0u; i < payload_words; i++)
		{
			uint32 w = 0xFFFFFFFFu;
			const uint32 base = i * 4u;
			if (base < payload_len)
			{
				uint8 b0 = (base + 0u < payload_len) ? src[base + 0u] : 0xFFu;
				uint8 b1 = (base + 1u < payload_len) ? src[base + 1u] : 0xFFu;
				uint8 b2 = (base + 2u < payload_len) ? src[base + 2u] : 0xFFu;
				uint8 b3 = (base + 3u < payload_len) ? src[base + 3u] : 0xFFu;
				w = ((uint32)b0) | ((uint32)b1 << 8) | ((uint32)b2 << 16) | ((uint32)b3 << 24);
			}
			dst[i] = w;
		}
	}

	// CRC32：对 payload 做 CRC
	{
		uint32 crc = flash_storage_crc32(payload, (uint32)payload_len);
		flash_union_buffer[2].uint32_type = crc;
	}

	// 预留：flash_union_buffer[3] 可用于 flags/seq/备用
	flash_union_buffer[3].uint32_type = 0u;

	// zf_driver_flash 约定：flash_write_page_from_buffer 返回 0 表示成功，1 表示失败
	return (uint8)(flash_write_page_from_buffer(sector, page) == 0u);
}

static uint8 flash_storage_read_blob(uint32 sector, uint32 page, uint32 magic, void *payload, uint16 payload_len)
{
	flash_read_page_to_buffer(sector, page);

	if (flash_union_buffer[0].uint32_type != magic)
	{
		return 0u;
	}

	{
		uint32 vlen = flash_union_buffer[1].uint32_type;
		uint16 ver = (uint16)(vlen >> 16);
		uint16 len = (uint16)(vlen & 0xFFFFu);
		if (ver != (uint16)FLASH_STORAGE_VERSION) return 0u;
		if (len != payload_len) return 0u;
	}

	{
		// 先按字节拷贝还原 payload
		uint8 *dst = (uint8 *)payload;
		const uint32 *src = (const uint32 *)&flash_union_buffer[FLASH_STORAGE_HDR_WORDS];
		const uint32 payload_words = (uint32)((payload_len + 3u) / 4u);
		uint32 i;
		for (i = 0u; i < payload_words; i++)
		{
			uint32 w = src[i];
			uint32 base = i * 4u;
			if (base + 0u < payload_len) dst[base + 0u] = (uint8)(w & 0xFFu);
			if (base + 1u < payload_len) dst[base + 1u] = (uint8)((w >> 8) & 0xFFu);
			if (base + 2u < payload_len) dst[base + 2u] = (uint8)((w >> 16) & 0xFFu);
			if (base + 3u < payload_len) dst[base + 3u] = (uint8)((w >> 24) & 0xFFu);
		}
	}

	// CRC 校验
	{
		uint32 crc_expect = flash_union_buffer[2].uint32_type;
		uint32 crc_actual = flash_storage_crc32(payload, (uint32)payload_len);
		if (crc_expect != crc_actual) return 0u;
	}

	return 1u;
}

uint8 flash_storage_save_pid_mode_cfg(void)
{
	return flash_storage_write_blob(
		FLASH_STORAGE_SECTOR_PID,
		FLASH_STORAGE_PAGE_PID,
		FLASH_STORAGE_MAGIC_PID,
		(const void *)s_pid_mode_cfg,
		(uint16)sizeof(s_pid_mode_cfg));
}

uint8 flash_storage_load_pid_mode_cfg(void)
{
	pid_mode_cfg_t tmp_new[PID_MODE_CFG_COUNT];

	// 1) 先尝试按“新长度”读取（当前固件版本）
	if (flash_storage_read_blob(
			FLASH_STORAGE_SECTOR_PID,
			FLASH_STORAGE_PAGE_PID,
			FLASH_STORAGE_MAGIC_PID,
			(void *)tmp_new,
			(uint16)sizeof(tmp_new)))
	{
		memcpy((void *)s_pid_mode_cfg, (const void *)tmp_new, sizeof(tmp_new));
		return 1u;
	}

	return 0u;
}

uint8 flash_storage_save_eight2_prog(void)
{
	return flash_storage_write_blob(
		FLASH_STORAGE_SECTOR_EIGHT2,
		FLASH_STORAGE_PAGE_EIGHT2,
		FLASH_STORAGE_MAGIC_EIGHT2,
		(const void *)g_eight2_prog,
		(uint16)sizeof(g_eight2_prog));
}

uint8 flash_storage_load_eight2_prog(void)
{
	ttop_eight2_seg_t tmp[TTOP_EIGHT2_PROG_LEN];
	if (!flash_storage_read_blob(
			FLASH_STORAGE_SECTOR_EIGHT2,
			FLASH_STORAGE_PAGE_EIGHT2,
			FLASH_STORAGE_MAGIC_EIGHT2,
			(void *)tmp,
			(uint16)sizeof(tmp)))
	{
		return 0u;
	}

	memcpy((void *)g_eight2_prog, (const void *)tmp, sizeof(tmp));
	return 1u;
}

uint8 flash_storage_save_eight2_prog_ex1(void)
{
	return flash_storage_write_blob(
		FLASH_STORAGE_SECTOR_EIGHT2_EX1,
		FLASH_STORAGE_PAGE_EIGHT2_EX1,
		FLASH_STORAGE_MAGIC_EIGHT2_EX1,
		(const void *)g_eight2_prog_ex1,
		(uint16)sizeof(g_eight2_prog_ex1));
}

uint8 flash_storage_load_eight2_prog_ex1(void)
{
	ttop_eight2_seg_t tmp[TTOP_EIGHT2_PROG_LEN];
	if (!flash_storage_read_blob(
			FLASH_STORAGE_SECTOR_EIGHT2_EX1,
			FLASH_STORAGE_PAGE_EIGHT2_EX1,
			FLASH_STORAGE_MAGIC_EIGHT2_EX1,
			(void *)tmp,
			(uint16)sizeof(tmp)))
	{
		return 0u;
	}

	memcpy((void *)g_eight2_prog_ex1, (const void *)tmp, sizeof(tmp));
	return 1u;
}

uint8 flash_storage_save_eight2_prog_ex2(void)
{
	return flash_storage_write_blob(
		FLASH_STORAGE_SECTOR_EIGHT2_EX2,
		FLASH_STORAGE_PAGE_EIGHT2_EX2,
		FLASH_STORAGE_MAGIC_EIGHT2_EX2,
		(const void *)g_eight2_prog_ex2,
		(uint16)sizeof(g_eight2_prog_ex2));
}

uint8 flash_storage_load_eight2_prog_ex2(void)
{
	ttop_eight2_seg_t tmp[TTOP_EIGHT2_PROG_LEN];
	if (!flash_storage_read_blob(
			FLASH_STORAGE_SECTOR_EIGHT2_EX2,
			FLASH_STORAGE_PAGE_EIGHT2_EX2,
			FLASH_STORAGE_MAGIC_EIGHT2_EX2,
			(void *)tmp,
			(uint16)sizeof(tmp)))
	{
		return 0u;
	}

	memcpy((void *)g_eight2_prog_ex2, (const void *)tmp, sizeof(tmp));
	return 1u;
}

typedef struct
{
	int32 counts_per_meter;
	ttop_kemu1_seg_t seg_table[TTOP_KEMU1_SEG_COUNT];
} flash_kemu1_cfg_t;

typedef struct
{
	uint16 count;
	uint16 _pad;
	float yaw_points[900];
} flash_rc_yaw_cfg_t;

typedef struct
{
	uint16 total_count;
	uint16 part_count;
	float yaw_points[900];
} flash_rc_yaw_part_t;

static flash_rc_yaw_cfg_t s_flash_rc_yaw_cfg;
static flash_rc_yaw_part_t s_flash_rc_yaw_part0;
static flash_rc_yaw_part_t s_flash_rc_yaw_part1;

uint8 flash_storage_save_rc_yaw_record(const float *yaw_points, uint16 count)
{
	if ((yaw_points == NULL) || (count == 0u) || (count > (uint16)1800u))
	{
		return 0u;
	}

	{
		uint16 count0 = (count > (uint16)900u) ? (uint16)900u : count;
		uint16 count1 = (uint16)(count - count0);

		memset((void *)&s_flash_rc_yaw_part0, 0, sizeof(s_flash_rc_yaw_part0));
		s_flash_rc_yaw_part0.total_count = count;
		s_flash_rc_yaw_part0.part_count = count0;
		memcpy((void *)s_flash_rc_yaw_part0.yaw_points, (const void *)yaw_points, (size_t)count0 * sizeof(float));

		if (!flash_storage_write_blob(
				FLASH_STORAGE_SECTOR_RC_YAW,
				FLASH_STORAGE_PAGE_RC_YAW,
				FLASH_STORAGE_MAGIC_RC_YAW_P0,
				(const void *)&s_flash_rc_yaw_part0,
				(uint16)sizeof(s_flash_rc_yaw_part0)))
		{
			return 0u;
		}

		memset((void *)&s_flash_rc_yaw_part1, 0, sizeof(s_flash_rc_yaw_part1));
		s_flash_rc_yaw_part1.total_count = count;
		s_flash_rc_yaw_part1.part_count = count1;
		if (count1 > 0u)
		{
			memcpy((void *)s_flash_rc_yaw_part1.yaw_points,
			       (const void *)(yaw_points + count0),
			       (size_t)count1 * sizeof(float));
		}

		return flash_storage_write_blob(
			FLASH_STORAGE_SECTOR_RC_YAW,
			FLASH_STORAGE_PAGE_RC_YAW_EX,
			FLASH_STORAGE_MAGIC_RC_YAW_P1,
			(const void *)&s_flash_rc_yaw_part1,
			(uint16)sizeof(s_flash_rc_yaw_part1));
	}
}

uint8 flash_storage_load_rc_yaw_record(float *yaw_points, uint16 max_points, uint16 *out_count)
{
	if ((yaw_points == NULL) || (out_count == NULL) || (max_points == 0u))
	{
		return 0u;
	}

	// 新格式：两页（最多 1800 点）
	if (flash_storage_read_blob(
			FLASH_STORAGE_SECTOR_RC_YAW,
			FLASH_STORAGE_PAGE_RC_YAW,
			FLASH_STORAGE_MAGIC_RC_YAW_P0,
			(void *)&s_flash_rc_yaw_part0,
			(uint16)sizeof(s_flash_rc_yaw_part0)))
	{
		uint16 total = s_flash_rc_yaw_part0.total_count;
		uint16 count0 = s_flash_rc_yaw_part0.part_count;
		uint16 count1;

		if ((total == 0u) || (total > (uint16)1800u)) return 0u;
		if ((count0 == 0u) || (count0 > (uint16)900u) || (count0 > total)) return 0u;
		if (total > max_points) return 0u;

		count1 = (uint16)(total - count0);
		if (count1 > (uint16)900u) return 0u;

		memcpy((void *)yaw_points,
		       (const void *)s_flash_rc_yaw_part0.yaw_points,
		       (size_t)count0 * sizeof(float));

		if (count1 > 0u)
		{
			if (!flash_storage_read_blob(
					FLASH_STORAGE_SECTOR_RC_YAW,
					FLASH_STORAGE_PAGE_RC_YAW_EX,
					FLASH_STORAGE_MAGIC_RC_YAW_P1,
					(void *)&s_flash_rc_yaw_part1,
					(uint16)sizeof(s_flash_rc_yaw_part1)))
			{
				return 0u;
			}

			if (s_flash_rc_yaw_part1.total_count != total) return 0u;
			if (s_flash_rc_yaw_part1.part_count != count1) return 0u;

			memcpy((void *)(yaw_points + count0),
			       (const void *)s_flash_rc_yaw_part1.yaw_points,
			       (size_t)count1 * sizeof(float));
		}

		*out_count = total;
		return 1u;
	}

	// 兼容旧格式：单页（最多 900 点）
	if (!flash_storage_read_blob(
			FLASH_STORAGE_SECTOR_RC_YAW,
			FLASH_STORAGE_PAGE_RC_YAW,
			FLASH_STORAGE_MAGIC_RC_YAW,
			(void *)&s_flash_rc_yaw_cfg,
			(uint16)sizeof(s_flash_rc_yaw_cfg)))
	{
		return 0u;
	}

	if ((s_flash_rc_yaw_cfg.count == 0u) || (s_flash_rc_yaw_cfg.count > (uint16)900u))
	{
		return 0u;
	}

	if (s_flash_rc_yaw_cfg.count > max_points)
	{
		return 0u;
	}

	memcpy((void *)yaw_points, (const void *)s_flash_rc_yaw_cfg.yaw_points, (size_t)s_flash_rc_yaw_cfg.count * sizeof(float));
	*out_count = s_flash_rc_yaw_cfg.count;
	return 1u;
}

uint8 flash_storage_save_kemu1_cfg(void)
{
	flash_kemu1_cfg_t cfg;
	cfg.counts_per_meter = (int32)g_ttop_kemu1_counts_per_meter;
	memcpy((void *)cfg.seg_table, (const void *)g_ttop_kemu1_seg_table, sizeof(cfg.seg_table));

	return flash_storage_write_blob(
		FLASH_STORAGE_SECTOR_KEMU1,
		FLASH_STORAGE_PAGE_KEMU1,
		FLASH_STORAGE_MAGIC_KEMU1,
		(const void *)&cfg,
		(uint16)sizeof(cfg));
}

uint8 flash_storage_load_kemu1_cfg(void)
{
	flash_kemu1_cfg_t cfg;
	if (!flash_storage_read_blob(
			FLASH_STORAGE_SECTOR_KEMU1,
			FLASH_STORAGE_PAGE_KEMU1,
			FLASH_STORAGE_MAGIC_KEMU1,
			(void *)&cfg,
			(uint16)sizeof(cfg)))
	{
		return 0u;
	}

	g_ttop_kemu1_counts_per_meter = (int)cfg.counts_per_meter;
	memcpy((void *)g_ttop_kemu1_seg_table, (const void *)cfg.seg_table, sizeof(cfg.seg_table));
	ttop_kemu1_rebuild_targets();
	return 1u;
}

/* ============================================================================
 *  惯导航点表（INS Path）Flash 存储
 *
 *  说明：
 *  - 航点表较大（最多 500 × 8 = 4000 字节），不适合在栈上分配临时缓冲区。
 *  - 因此采用"直接操作 flash_union_buffer"的方式读写，不经过 write_blob。
 *  - 页内布局：
 *    [0] magic        : FLASH_STORAGE_MAGIC_INS
 *    [1] version<<16 | count  : 高16位=版本号，低16位=航点数
 *    [2] crc32        : 对航点数据的 CRC 校验
 *    [3] reserved     : 保留（0）
 *    [4..] payload    : 航点数据，每个航点占 2 个 uint32（float x, float y）
 * ============================================================================ */

uint8 flash_storage_save_ins_path(void)
{
	uint16 n = g_ins_path_count;
	if (n > (uint16)INS_PATH_MAX_POINTS) n = (uint16)INS_PATH_MAX_POINTS;

	/* 计算所需空间并检查是否超出一页 */
	const uint32 data_words = (uint32)n * 2u;  /* 每个航点 = 2 个 float = 2 个 uint32 */
	const uint32 total_words = (uint32)FLASH_STORAGE_HDR_WORDS + data_words;

	if (total_words > (uint32)EEPROM_PAGE_LENGTH)
	{
		return 0u;  /* 航点太多，一页放不下 */
	}

	/* 若页已写入过数据，先擦除 */
	if (flash_check(FLASH_STORAGE_SECTOR_INS, FLASH_STORAGE_PAGE_INS))
	{
		flash_erase_page(FLASH_STORAGE_SECTOR_INS, FLASH_STORAGE_PAGE_INS);
	}

	flash_buffer_clear();

	/* ---- Header ---- */
	flash_union_buffer[0].uint32_type = FLASH_STORAGE_MAGIC_INS;
	flash_union_buffer[1].uint32_type = ((uint32)FLASH_STORAGE_VERSION << 16) | (uint32)n;

	/* ---- Payload：逐个航点写入 ---- */
	for (uint16 i = 0u; i < n; i++)
	{
		uint32 off = (uint32)FLASH_STORAGE_HDR_WORDS + (uint32)i * 2u;
		float x = g_ins_path[i].x;
		float y = g_ins_path[i].y;
		memcpy(&flash_union_buffer[off].uint32_type, &x, sizeof(float));
		memcpy(&flash_union_buffer[off + 1u].uint32_type, &y, sizeof(float));
	}

	/* ---- CRC32：对航点原始数据做校验 ---- */
	{
		uint32 crc = flash_storage_crc32((const void *)g_ins_path,
		                                 (uint32)n * (uint32)sizeof(ins_waypoint_t));
		flash_union_buffer[2].uint32_type = crc;
	}
	flash_union_buffer[3].uint32_type = 0u;

	return (uint8)(flash_write_page_from_buffer(FLASH_STORAGE_SECTOR_INS,
	                                            FLASH_STORAGE_PAGE_INS) == 0u);
}

uint8 flash_storage_load_ins_path(void)
{
	flash_read_page_to_buffer(FLASH_STORAGE_SECTOR_INS, FLASH_STORAGE_PAGE_INS);

	/* ---- 校验 magic ---- */
	if (flash_union_buffer[0].uint32_type != FLASH_STORAGE_MAGIC_INS)
	{
		return 0u;
	}

	/* ---- 校验 version + 读取 count ---- */
	{
		uint32 vlen = flash_union_buffer[1].uint32_type;
		uint16 ver   = (uint16)(vlen >> 16);
		uint16 count = (uint16)(vlen & 0xFFFFu);

		if (ver != (uint16)FLASH_STORAGE_VERSION) return 0u;
		if (count > (uint16)INS_PATH_MAX_POINTS)  return 0u;

		/* ---- 读取航点数据 ---- */
		for (uint16 i = 0u; i < count; i++)
		{
			uint32 off = (uint32)FLASH_STORAGE_HDR_WORDS + (uint32)i * 2u;
			float x, y;
			memcpy(&x, &flash_union_buffer[off].uint32_type, sizeof(float));
			memcpy(&y, &flash_union_buffer[off + 1u].uint32_type, sizeof(float));
			g_ins_path[i].x = x;
			g_ins_path[i].y = y;
		}

		/* ---- CRC 校验 ---- */
		{
			uint32 crc_expect = flash_union_buffer[2].uint32_type;
			uint32 crc_actual = flash_storage_crc32(
				(const void *)g_ins_path,
				(uint32)count * (uint32)sizeof(ins_waypoint_t));

			if (crc_expect != crc_actual) return 0u;
		}

		g_ins_path_count = count;
	}

	return 1u;
}


// ============================================================================
// Flash 存储 —— roll_slope_speed_cfg（坡道速度参数）
// ============================================================================

uint8 flash_storage_save_slope_cfg(void)
{
	const roll_slope_speed_cfg_t *cfg = roll_slope_speed_get_cfg();
	return flash_storage_write_blob(
		FLASH_STORAGE_SECTOR_SLOPE,
		FLASH_STORAGE_PAGE_SLOPE,
		FLASH_STORAGE_MAGIC_SLOPE,
		(const void *)cfg,
		(uint16)sizeof(roll_slope_speed_cfg_t));
}

uint8 flash_storage_load_slope_cfg(void)
{
	roll_slope_speed_cfg_t tmp;
	if (!flash_storage_read_blob(
			FLASH_STORAGE_SECTOR_SLOPE,
			FLASH_STORAGE_PAGE_SLOPE,
			FLASH_STORAGE_MAGIC_SLOPE,
			(void *)&tmp,
			(uint16)sizeof(roll_slope_speed_cfg_t)))
	{
		return 0u;
	}
	roll_slope_speed_set_cfg(&tmp);
	return 1u;
}

// ============================================================================
// Flash 存储 —— g_ttop_kemu3_seg_table（KEMU3 段表）
// ============================================================================

uint8 flash_storage_save_kemu3_seg(void)
{
	return flash_storage_write_blob(
		FLASH_STORAGE_SECTOR_KEMU3,
		FLASH_STORAGE_PAGE_KEMU3,
		FLASH_STORAGE_MAGIC_KEMU3,
		(const void *)g_ttop_kemu3_seg_table,
		(uint16)sizeof(ttop_kemu3_seg_t) * TTOP_KEMU3_SEG_COUNT);
}

uint8 flash_storage_load_kemu3_seg(void)
{
	ttop_kemu3_seg_t tmp[TTOP_KEMU3_SEG_COUNT];
	if (!flash_storage_read_blob(
			FLASH_STORAGE_SECTOR_KEMU3,
			FLASH_STORAGE_PAGE_KEMU3,
			FLASH_STORAGE_MAGIC_KEMU3,
			(void *)tmp,
			(uint16)sizeof(ttop_kemu3_seg_t) * TTOP_KEMU3_SEG_COUNT))
	{
		return 0u;
	}
	memcpy((void *)g_ttop_kemu3_seg_table, (const void *)tmp, sizeof(tmp));
	return 1u;
}

// ============================================================================
// Flash 存储 —— 杂项配置（flash_misc_cfg_t）
// ============================================================================

uint8 flash_storage_save_misc_cfg(void)
{
	flash_misc_cfg_t cfg;

	// ---- SPCH_DBG ----
	cfg.fit_mode              = g_ttop_speed_change_fit_mode;
	cfg.lpf_alpha             = g_ttop_speed_change_lpf_alpha;
	cfg.pitch_angle_lpf_alpha = g_pitch_angle_lpf_alpha;
	cfg.pitch_zero_offset_deg = g_pitch_zero_offset_deg;

	// ---- KEMU3 额外参数 ----
	cfg.ex2_loop_count           = g_ttop_kemu3_ex2_loop_count;
	cfg.kemu3_end_target_rel_deg = g_ttop_kemu3_end_seg.target_rel_deg;
	cfg.kemu3_end_rate_dps       = g_ttop_kemu3_end_seg.rate_dps;
	cfg.yaw_zero_offset_deg      = g_yaw_zero_offset_deg;
	cfg.kemu3_yaw_ki_path        = g_kemu3_yaw_ki_path;
	cfg.kemu3_counts_per_meter   = g_ttop_kemu3_counts_per_meter;

	// ---- MOTOR_PID ----
	motor_get_pid_gains(&cfg.motor_kp, &cfg.motor_ki, &cfg.motor_kd);
	motor_get_pitch_speed_boost_params(&cfg.motor_pitch_boost_threshold, &cfg.motor_pitch_boost_k);
	cfg.motor_pitch_boost_max = motor_get_pitch_speed_boost_max_counts();
	cfg.motor_pitch_boost_enable = motor_get_pitch_speed_boost_enable();

	// reserved 填充 0（后续新增字段会覆盖对应位置）
	memset((void *)cfg.reserved, 0, sizeof(cfg.reserved));
	// _pad 字段在栈上默认未初始化，显式清零
	cfg._pad0[0] = 0; cfg._pad0[1] = 0; cfg._pad0[2] = 0;
	cfg._pad1[0] = 0; cfg._pad1[1] = 0; cfg._pad1[2] = 0;
	cfg._pad2[0] = 0; cfg._pad2[1] = 0; cfg._pad2[2] = 0;

	return flash_storage_write_blob(
		FLASH_STORAGE_SECTOR_MISC,
		FLASH_STORAGE_PAGE_MISC,
		FLASH_STORAGE_MAGIC_MISC,
		(const void *)&cfg,
		(uint16)sizeof(flash_misc_cfg_t));
}

uint8 flash_storage_load_misc_cfg(void)
{
	flash_misc_cfg_t cfg;

	if (!flash_storage_read_blob(
			FLASH_STORAGE_SECTOR_MISC,
			FLASH_STORAGE_PAGE_MISC,
			FLASH_STORAGE_MAGIC_MISC,
			(void *)&cfg,
			(uint16)sizeof(flash_misc_cfg_t)))
	{
		return 0u;
	}

	// ---- SPCH_DBG ----
	g_ttop_speed_change_fit_mode = cfg.fit_mode;
	g_ttop_speed_change_lpf_alpha = cfg.lpf_alpha;
	g_pitch_angle_lpf_alpha      = cfg.pitch_angle_lpf_alpha;
	if (g_pitch_angle_lpf_alpha < 0.0f) g_pitch_angle_lpf_alpha = 0.0f;
	if (g_pitch_angle_lpf_alpha > 0.99f) g_pitch_angle_lpf_alpha = 0.99f;
	g_pitch_zero_offset_deg      = cfg.pitch_zero_offset_deg;

	// ---- KEMU3 额外参数 ----
	g_ttop_kemu3_ex2_loop_count           = cfg.ex2_loop_count;
	g_ttop_kemu3_end_seg.target_rel_deg   = cfg.kemu3_end_target_rel_deg;
	g_ttop_kemu3_end_seg.rate_dps         = cfg.kemu3_end_rate_dps;
	g_yaw_zero_offset_deg                 = cfg.yaw_zero_offset_deg;
	g_kemu3_yaw_ki_path                   = cfg.kemu3_yaw_ki_path;
	g_ttop_kemu3_counts_per_meter         = cfg.kemu3_counts_per_meter;
	ttop_kemu3_rebuild_targets();

	// ---- MOTOR_PID ----
	motor_set_pid_gains(cfg.motor_kp, cfg.motor_ki, cfg.motor_kd);
	motor_set_pitch_speed_boost_params(cfg.motor_pitch_boost_threshold, cfg.motor_pitch_boost_k);
	motor_set_pitch_speed_boost_max_counts(cfg.motor_pitch_boost_max);
	motor_set_pitch_speed_boost_enable(cfg.motor_pitch_boost_enable);

	return 1u;
}
