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

#include "common_flie.h"      // s_pid_mode_cfg
#include "top_8_turn_2.h"     // g_eight2_prog
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

// 惯导航点表存储位置（使用 page 1，与其它数据分开）
#ifndef FLASH_STORAGE_SECTOR_INS
#define FLASH_STORAGE_SECTOR_INS    (0u)
#endif
#ifndef FLASH_STORAGE_PAGE_INS
#define FLASH_STORAGE_PAGE_INS      (1u)
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
#define FLASH_STORAGE_MAGIC_INS     (0x494E5350u) // 'INSP' (INS Path)
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
