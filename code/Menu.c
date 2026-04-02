/*
 * Menu.c
 *
 *  Created on: 2026-01-17
 *      Author: hhttiuui
 *
 *  菜单/界面显示模块：
 *  - IPS200 屏幕菜单显示
 *  - 按键切换菜单层级与选项
 *  - 显示 IMU / PID / 速度等调试信息
 */

#include "zf_common_headfile.h"
#include "Menu.h"
#include "Key.h"
#include "motor_controller.h"
#include "Ttop_funtion.h"
#include "top_8_turn_2.h"
#include "flash_storage.h"
#include "common_flie.h"
#include "ins_dead_reckoning.h"
#include "ins_path_follow.h"
#include "gps_display.h"

// =========================
// 菜单：扣扣飞车
// =========================
typedef enum
{
    UI_PAGE_LEGACY = 0,          // 保留原来的 main/second 菜单
    UI_PAGE_KKF_MAIN = 1,        // 界面1：扣扣飞车
    UI_PAGE_KKF_PARAM = 2,       // 界面1_1：参数调试（含模式选择入口+参数列表）
    UI_PAGE_KKF_MODE_SELECT = 3, // 界面1_1_2：模式选择
    UI_PAGE_KKF_EIGHT_TURN = 4,  // 界面1_1_3：8字段表（只读显示）

    UI_PAGE_KKF_INS_PATH_VIEW = 13, // 界面1：航点查看（g_ins_path 分页显示）

    UI_PAGE_KKF_EIGHT2_PROG_SELECT = 10, // 界面1_1_3_1：段表选择（main/ex1/ex2）

    UI_PAGE_KKF_PID_MODE_SELECT = 14, // 界面1_1：PID_MODE 选择页（选择要编辑哪套 PID 参数）
    UI_PAGE_KKF_GPS_DEBUG = 15,      // 界面1_1_9：GPS 调试入口（四项空菜单）
    UI_PAGE_KKF_GPS_ROUTE = 16,      // 专门界面：显示已保存路线（点/线）
    UI_PAGE_KKF_GPS_HEADINGS = 17,   // 专门界面：显示多段航向角

    // 界面1_1_4 ~ 1_1_8：PID 参数（直接操作 s_pid_mode_cfg）
    UI_PAGE_KKF_PID_CASCADE = 5,
    UI_PAGE_KKF_PID_CASCADE2 = 11,
    UI_PAGE_KKF_PID_CASCADE3 = 12,
    UI_PAGE_KKF_PID_ANGLE = 6,
    UI_PAGE_KKF_PID_DAMPED_TWO = 7,
    UI_PAGE_KKF_PID_DAMPED_THREE = 8,
    UI_PAGE_KKF_PID_DAMPED_FOUR = 9,
} ui_page_t;

// 上电默认进入“扣扣飞车”主界面
static volatile ui_page_t s_ui_page = UI_PAGE_KKF_MAIN;

// 扣扣飞车：主界面光标（行号从1开始，对齐你当前的“return在第1行”的习惯）
static volatile uint8 s_kkf_main_item = 1u;
static volatile uint8 s_kkf_param_item = 1u;
static volatile uint8 s_kkf_mode_item = 1u;
static volatile uint8 s_kkf_eight_turn_item = 1u;
static volatile uint8 s_kkf_pid_item = 1u;

// GPS 调试子菜单当前选项（1..4），用于 kkf_gps_debug_draw / Menu_Select
static volatile uint8 s_kkf_gps_item = 1u;

// 菜单中用于选择 motor_request_start 的临时状态（已移除）

// PID 选择：当前选中的 PID 模式（用于 PID_CHANGE 进入编辑）
static volatile pid_mode_t s_kkf_pid_mode_id = PID_MODE_CASCADE;
static volatile uint8 s_kkf_pid_mode_item = 1u; // 1..7

// 航点查看页：当前页（从 0 开始）
static volatile uint16 s_kkf_ins_path_page = 0u;

// eight2 段表选择：当前选择（用于显示/编辑哪一套段表）
static volatile ttop_eight2_prog_id_t s_kkf_eight2_prog_id = TTOP_EIGHT2_PROG_MAIN;
static volatile uint8 s_kkf_eight2_prog_item = 1u; // 1..3

static volatile ttop_eight2_seg_t *kkf_eight2_prog_ptr(ttop_eight2_prog_id_t id)
{
    switch (id)
    {
        case TTOP_EIGHT2_PROG_EX1: return (volatile ttop_eight2_seg_t *)g_eight2_prog_ex1;
        case TTOP_EIGHT2_PROG_EX2: return (volatile ttop_eight2_seg_t *)g_eight2_prog_ex2;
        case TTOP_EIGHT2_PROG_MAIN:
        default: return (volatile ttop_eight2_seg_t *)g_eight2_prog;
    }
}

static const char *kkf_eight2_prog_str(ttop_eight2_prog_id_t id)
{
    switch (id)
    {
        case TTOP_EIGHT2_PROG_EX1: return "EX1";
        case TTOP_EIGHT2_PROG_EX2: return "EX2";
        case TTOP_EIGHT2_PROG_MAIN:
        default: return "MAIN";
    }
}

// MT_ST 显示字符串：当被选中时返回带 '#' 的高亮样式
// moto start menu removed

static const char *kkf_pid_mode_str(pid_mode_t mode)
{
    switch (mode)
    {
        case PID_MODE_CASCADE: return "CASCADE";
        case PID_MODE_CASCADE_EX1: return "CASCADE2";
        case PID_MODE_CASCADE_EX2: return "CASCADE3";
        case PID_MODE_ANGLE: return "ANGLE";
        case PID_MODE_DAMPED_TWO: return "DAMPED2";
        case PID_MODE_DAMPED_THREE: return "DAMPED3";
        case PID_MODE_DAMPED_FOUR: return "DAMPED4";
        default: return "CASCADE";
    }
}

static uint8 kkf_pid_mode_to_item(pid_mode_t mode)
{
    switch (mode)
    {
        case PID_MODE_CASCADE: return 1u;
        case PID_MODE_CASCADE_EX1: return 2u;
        case PID_MODE_CASCADE_EX2: return 3u;
        case PID_MODE_ANGLE: return 4u;
        case PID_MODE_DAMPED_TWO: return 5u;
        case PID_MODE_DAMPED_THREE: return 6u;
        case PID_MODE_DAMPED_FOUR: return 7u;
        default: return 1u;
    }
}

static pid_mode_t kkf_item_to_pid_mode(uint8 item)
{
    switch (item)
    {
        case 2u: return PID_MODE_CASCADE_EX1;
        case 3u: return PID_MODE_CASCADE_EX2;
        case 4u: return PID_MODE_ANGLE;
        case 5u: return PID_MODE_DAMPED_TWO;
        case 6u: return PID_MODE_DAMPED_THREE;
        case 7u: return PID_MODE_DAMPED_FOUR;
        case 1u:
        default: return PID_MODE_CASCADE;
    }
}

static ui_page_t kkf_pid_mode_to_page(pid_mode_t mode)
{
    switch (mode)
    {
        case PID_MODE_CASCADE: return UI_PAGE_KKF_PID_CASCADE;
        case PID_MODE_CASCADE_EX1: return UI_PAGE_KKF_PID_CASCADE2;
        case PID_MODE_CASCADE_EX2: return UI_PAGE_KKF_PID_CASCADE3;
        case PID_MODE_ANGLE: return UI_PAGE_KKF_PID_ANGLE;
        case PID_MODE_DAMPED_TWO: return UI_PAGE_KKF_PID_DAMPED_TWO;
        case PID_MODE_DAMPED_THREE: return UI_PAGE_KKF_PID_DAMPED_THREE;
        case PID_MODE_DAMPED_FOUR: return UI_PAGE_KKF_PID_DAMPED_FOUR;
        default: return UI_PAGE_KKF_PID_CASCADE;
    }
}

typedef enum
{
    KKF_EIGHT_TURN_SEL = 0u,   // '>' 选择行
    KKF_EIGHT_TURN_EDIT_DEG,  // '-' 编辑偏移量
    KKF_EIGHT_TURN_EDIT_DPS,  // '*' 编辑限速
} kkf_eight_turn_state_t;

static volatile kkf_eight_turn_state_t s_kkf_eight_turn_state = KKF_EIGHT_TURN_SEL;

// 模式：1~12（与顶层 funtion_kemu 对齐）
static volatile uint8 s_kkf_mode = 1u;

// 与顶层调度融合：菜单直接接管模式/ready/清零等动作
extern volatile uint8 eight_flag;
extern volatile uint16 eight_count;
void ttop_funtion_mode_once_apply(void);

static const char *kkf_mode_str(uint8 mode)
{
    switch (mode)
    {
        case 1u: return "MODE 1 200";
        case 2u: return "MODE 2 SPCHD";
        case 3u: return "MODE 3 55";
        case 4u: return "MODE 4 45";
        case 5u: return "MODE 5 200+T";
        case 6u: return "MODE 6 8-TURN";
        case 7u: return "MODE 7";
        case 8u: return "MODE 8";
        case 9u: return "MODE 9 INS-REC";
        case 10u: return "MODE10 INS-TRK";
        case 11u: return "MODE11";
        case 12u: return "MODE12";
        default: return "MODE 1";
    }
}

static void kkf_draw_signature_bottom(void)
{
    // 在所有页面底部显示署名：MADE_BY -> hhttiuui
    // 使用 ips200_height_max 动态计算最后一行的 y，避免不同旋转/分辨率下越界。
    uint16 y;

    // 8x16 字体高度约 16 像素，这里把基线放在倒数第 16 像素处。
    if (ips200_height_max > 16u)
    {
        y = (uint16)(ips200_height_max - 16u);
    }
    else
    {
        y = 0u;
    }

    ips200_show_string(0, y, "MADE_BY -> hhttiuui");
}

static void kkf_main_draw(void)
{
    // 界面1：
    // 扣扣飞车
    //   参数调试
    //   START
    //   RESET
    //   STATE(显示是否开始跑)
    //   清零
    //   模式：（显示当前模式）
    //   yaw：
    //   pitch：
    // > 速度：
    ips200_show_string(0, 0 * 15, "KOU_KOU_FLY_CAR");
    ips200_show_string(16, 1 * 15, "PID_CONFIG");
    ips200_show_string(16, 2 * 15, "START");
    ips200_show_string(16, 3 * 15, "RESET");
    ips200_show_string(16, 4 * 15, "CLEAR");
    ips200_show_string(16, 5 * 15, "GPS_DBG");
    ips200_show_string(16, 6 * 15, "STATE:");
    ips200_show_string(16 * 6, 6 * 15, funtion_ready_flag ? "RUN" : "STOP");
    ips200_show_string(16, 7 * 15, "MODE:");
    ips200_show_string(16 * 6, 7 * 15, (char *)kkf_mode_str(s_kkf_mode));
    ips200_show_string(16, 8 * 15, "yaw:");
    ips200_show_float(16 * 6, 8 * 15, show_yaw_deg, 8, 2);//g_yaw_target_deg，show_yaw_deg
    ips200_show_string(16, 9 * 15, "t_yaw:");
    ips200_show_float(16 * 6, 9 * 15, g_yaw_target_deg, 8, 2);//eight_count
    ips200_show_string(16, 10 * 15, "pitch:");
    ips200_show_float(16 * 6, 10 * 15, show_pitch_deg, 8, 2);
    ips200_show_string(16, 11 * 15, "SPD:");
    ips200_show_int(16 * 6, 11 * 15, motor_speed_raw_counts, 6);

    ips200_show_string(16, 12 * 15, "xy:");
    ips200_show_float(16 * 3, 12 * 15, ins_dr_get_pos_x(), 2, 3);
    ips200_show_float(16 * 8, 12 * 15, ins_dr_get_pos_y(), 2, 3);

    ips200_show_string(16, 14 * 15, "g_ins_path");

    ips200_show_string(0, s_kkf_main_item * 15, "> ");

    kkf_draw_signature_bottom();
}

static inline uint16 kkf_ui_rows_available(void)
{
    // 以 15px 行高为准，并为底部签名预留 16px。
    uint16 usable_h = (ips200_height_max > 16u) ? (uint16)(ips200_height_max - 16u) : ips200_height_max;
    uint16 rows = (usable_h / 15u);
    return (rows > 0u) ? rows : 1u;
}

static void kkf_ins_path_draw(void)
{
    // 航点查看页：按屏幕可显示行数分页
    // - KEY1/KEY3：翻页
    // - KEY4：返回
    // 显示：索引 + x + y

    uint16 rows = kkf_ui_rows_available();
    uint16 count = g_ins_path_count;
    if (count > (uint16)INS_PATH_MAX_POINTS) count = (uint16)INS_PATH_MAX_POINTS;

    // 预留 2 行标题/信息
    uint16 points_per_page = (rows > 2u) ? (uint16)(rows - 2u) : 1u;
    if (points_per_page == 0u) points_per_page = 1u;

    uint16 page_count = 1u;
    if (count > 0u)
    {
        page_count = (uint16)((count + points_per_page - 1u) / points_per_page);
        if (page_count == 0u) page_count = 1u;
    }

    uint16 page = s_kkf_ins_path_page;
    if (page >= page_count) page = (uint16)(page_count - 1u);
    s_kkf_ins_path_page = page;

    uint16 start = (uint16)(page * points_per_page);
    uint16 end = (uint16)(start + points_per_page);
    if (end > count) end = count;

    ips200_show_string(0, 0, "G_INS_PATH");

    // 第 1 行：N + page
    ips200_show_string(0, 1 * 15, "N:");
    ips200_show_int(16 * 2, 1 * 15, (int32)count, 5);
    ips200_show_string(16 * 6, 1 * 15, "P:");
    ips200_show_int(16 * 8, 1 * 15, (int32)(page + 1u), 3);
    ips200_show_string(16 * 10, 1 * 15, "/");
    ips200_show_int(16 * 11, 1 * 15, (int32)page_count, 3);

    // 航点列表：从第 2 行开始
    if (count == 0u)
    {
        ips200_show_string(0, 2 * 15, "EMPTY");
        kkf_draw_signature_bottom();
        return;
    }

    for (uint16 i = start; i < end; i++)
    {
        uint16 row = (uint16)(2u + (i - start));
        uint16 y = (uint16)(row * 15u);

        // idx
        ips200_show_int(0, y, (int32)i, 4);
        // x / y
        ips200_show_float(16 * 3, y, g_ins_path[i].x, 7, 3);
        ips200_show_float(16 * 8, y, g_ins_path[i].y, 7, 3);
    }

    kkf_draw_signature_bottom();
}

static void kkf_param_draw(void)
{
    // 界面1_1：
    // 扣扣飞车
    //   模式选择：（当前模式）
    //   eight_turn
    //   PID_CHANGE（进入编辑）
    //   PID_MODE（选择要改哪套 PID）
    ips200_show_string(0, 0 * 15, "KOU_KOU_FLY_CAR");
    ips200_show_string(16, 1 * 15, "MODE SEL:");
    ips200_show_string(16 * 8, 1 * 15, (char *)kkf_mode_str(s_kkf_mode));
    ips200_show_string(16, 2 * 15, "EIGHT_TURN");
    ips200_show_string(16, 3 * 15, "E8_PROG:");
    ips200_show_string(16 * 8, 3 * 15, (char *)kkf_eight2_prog_str(s_kkf_eight2_prog_id));
    ips200_show_string(16, 4 * 15, "PID_CHANGE");
    ips200_show_string(16, 5 * 15, "PID_MODE:");
    ips200_show_string(16 * 8, 5 * 15, (char *)kkf_pid_mode_str((pid_mode_t)s_kkf_pid_mode_id));

    ips200_show_string(0, s_kkf_param_item * 15, "> ");

    kkf_draw_signature_bottom();
}

static void kkf_pid_mode_select_draw(void)
{
    // PID_MODE 选择页：KEY1/KEY3 移动；KEY2 确认；KEY4 返回
    ips200_show_string(0, 0 * 15, "KOU_KOU_FLY_CAR");
    ips200_show_string(16, 1 * 15, "PID_MODE");

    ips200_show_string(16, 2 * 15, "CASCADE");
    ips200_show_string(16, 3 * 15, "CASCADE2");
    ips200_show_string(16, 4 * 15, "CASCADE3");
    ips200_show_string(16, 5 * 15, "ANGLE");
    ips200_show_string(16, 6 * 15, "DAMPED2");
    ips200_show_string(16, 7 * 15, "DAMPED3");
    ips200_show_string(16, 8 * 15, "DAMPED4");

    // 光标：从第 2 行开始
    ips200_show_string(0, (uint16)((1u + (uint16)s_kkf_pid_mode_item) * 15u), "> ");

    kkf_draw_signature_bottom();
}

static void kkf_eight_turn_draw(void)
{
    // 界面1_1_3：EIGHT_TURN（只显示，不做修改）
    // 期望显示方式：直接显示 0~11 行
    // 55.0    20.0
    //  0.0    20.0
    //  ...
    //
    // 说明：这里每行显示两列数值：
    // - 左列：target_rel_deg（偏移量，deg）
    // - 右列：rate_dps（限速，deg/s）
    uint16 i;
    char cursor[3] = ">";

    ips200_show_string(0, 0 * 15, "KOU_KOU_FLY_CAR");
    ips200_show_string(16, 1 * 15, "EIGHT_TURN");

    // 光标符号：选择/编辑偏移/编辑限速
    if (s_kkf_eight_turn_state == KKF_EIGHT_TURN_SEL)
    {
        cursor[0] = '>';
    }
    else if (s_kkf_eight_turn_state == KKF_EIGHT_TURN_EDIT_DEG)
    {
        cursor[0] = '-';
    }
    else
    {
        cursor[0] = '*';
    }

    volatile ttop_eight2_seg_t *prog = kkf_eight2_prog_ptr((ttop_eight2_prog_id_t)s_kkf_eight2_prog_id);

    for (i = 0u; i < (uint16)TTOP_EIGHT2_PROG_LEN; i++)
    {
        // 从第 2 行开始显示 12 行
        uint16 y = (uint16)((2u + i) * 15u);

        // 选中行显示光标
        if (i == (uint16)s_kkf_eight_turn_item)
        {
            ips200_show_string(0, y, cursor);
        }
        else
        {
            ips200_show_string(0, y, " ");
        }

        ips200_show_float(16 * 1, y, prog[i].target_rel_deg, 6, 1);
        ips200_show_float(16 * 6, y, prog[i].rate_dps, 6, 1);
    }

    kkf_draw_signature_bottom();
}

static void kkf_eight2_prog_select_draw(void)
{
    // 段表选择页：
    // > MAIN
    //   EX1
    //   EX2
    ips200_show_string(0, 0 * 15, "KOU_KOU_FLY_CAR");
    ips200_show_string(16, 1 * 15, "E8_PROG SEL");

    ips200_show_string(16, 2 * 15, "MAIN");
    ips200_show_string(16, 3 * 15, "EX1");
    ips200_show_string(16, 4 * 15, "EX2");

    ips200_show_string(0, (uint16)((1u + (uint16)s_kkf_eight2_prog_item) * 15u), "> ");

    kkf_draw_signature_bottom();
}

// GPS 调试空菜单（4 项，占位）
static void kkf_gps_debug_draw(void)
{
    ips200_show_string(0, 0 * 15, "KOU_KOU_FLY_CAR");

    ips200_show_string(16, 1 * 15, "SAVE_PT");
    ips200_show_string(16, 2 * 15, "SHOW_ROUTE");
    ips200_show_string(16, 3 * 15, "SHOW_HEADS");
    ips200_show_string(16, 4 * 15, "CLEAR");

    // 光标：从第1行开始显示（顶标题占第0行）
    ips200_show_string(0, (uint16)((uint16)s_kkf_gps_item * 15u), "> ");

    kkf_draw_signature_bottom();
}

static void kkf_mode_select_draw(void)
{
    // 界面1_1_2：
    // 扣扣飞车
    // > MODE 1
    //   MODE 2
    //   MODE 3
    //   MODE 4
    //   MODE 5
    //   MODE 6
    ips200_show_string(0, 0 * 15, "KOU_KOU_FLY_CAR");
    ips200_show_string(16, 1 * 15, "MODE 1 200");
    ips200_show_string(16, 2 * 15, "MODE 2 SPCHD");
    ips200_show_string(16, 3 * 15, "MODE 3 55");
    ips200_show_string(16, 4 * 15, "MODE 4 45");
    ips200_show_string(16, 5 * 15, "MODE 5 200+T");
    ips200_show_string(16, 6 * 15, "MODE 6 8-TURN");
    ips200_show_string(16, 7 * 15, "MODE 7");
    ips200_show_string(16, 8 * 15, "MODE 8");
    ips200_show_string(16, 9 * 15, "MODE 9 INS-REC");
    ips200_show_string(16, 10 * 15, "MODE10 INS-TRK");
    ips200_show_string(16, 11 * 15, "MODE11");
    ips200_show_string(16, 12 * 15, "MODE12");
    ips200_show_string(0, s_kkf_mode_item * 15, "> ");

    kkf_draw_signature_bottom();
}

// =========================
// PID 参数编辑页（1_1_4 ~ 1_1_8）
// =========================

typedef enum
{
    KKF_PID_FIELD_AHRS_KP = 1u,
    KKF_PID_FIELD_YAW_KP,
    KKF_PID_FIELD_ANGLE_KP,
    KKF_PID_FIELD_RATE_KP,
    KKF_PID_FIELD_DAMP_K,
    KKF_PID_FIELD_DAMP_MAX,
    KKF_PID_FIELD_MOTOR_TGT,
    KKF_PID_FIELD_COUNT = KKF_PID_FIELD_MOTOR_TGT,
} kkf_pid_field_t;

typedef enum
{
    KKF_PID_STATE_SEL = 0u,   // '>' 选择字段
    KKF_PID_STATE_EDIT = 1u,  // '-' 编辑当前字段
} kkf_pid_state_t;

static volatile kkf_pid_state_t s_kkf_pid_state = KKF_PID_STATE_SEL;

static inline pid_mode_t kkf_page_to_pid_mode(ui_page_t page)
{
    switch (page)
    {
        case UI_PAGE_KKF_PID_CASCADE: return PID_MODE_CASCADE;
        case UI_PAGE_KKF_PID_CASCADE2: return PID_MODE_CASCADE_EX1;
        case UI_PAGE_KKF_PID_CASCADE3: return PID_MODE_CASCADE_EX2;
        case UI_PAGE_KKF_PID_ANGLE: return PID_MODE_ANGLE;
        case UI_PAGE_KKF_PID_DAMPED_TWO: return PID_MODE_DAMPED_TWO;
        case UI_PAGE_KKF_PID_DAMPED_THREE: return PID_MODE_DAMPED_THREE;
        case UI_PAGE_KKF_PID_DAMPED_FOUR: return PID_MODE_DAMPED_FOUR;
        default: return PID_MODE_CASCADE;
    }
}

static void kkf_pid_draw_for_mode(pid_mode_t mode)
{
    // 显示字段：
    // 1 ahrs_kp
    // 2 yaw.kp
    // 3 angle.kp
    // 4 rate.kp（无则 0）
    // 5 damp_k
    // 6 damp_max
    // 7 motor_target_counts_5ms
    const pid_mode_cfg_t *cfg = &s_pid_mode_cfg[(uint16)mode];

    char cursor[3] = ">";
    if (s_kkf_pid_state == KKF_PID_STATE_EDIT) cursor[0] = '-';

    ips200_show_string(0, 0 * 15, "KOU_KOU_FLY_CAR");

    // 顶部显示当前 mode 名
    if (mode == PID_MODE_CASCADE) ips200_show_string(16, 1 * 15, "PID_CASCADE");
    else if (mode == PID_MODE_CASCADE_EX1) ips200_show_string(16, 1 * 15, "PID_CASCADE2");
    else if (mode == PID_MODE_CASCADE_EX2) ips200_show_string(16, 1 * 15, "PID_CASCADE3");
    else if (mode == PID_MODE_ANGLE) ips200_show_string(16, 1 * 15, "PID_ANGLE");
    else if (mode == PID_MODE_DAMPED_TWO) ips200_show_string(16, 1 * 15, "PID_DAMPED2");
    else if (mode == PID_MODE_DAMPED_THREE) ips200_show_string(16, 1 * 15, "PID_DAMPED3");
    else ips200_show_string(16, 1 * 15, "PID_DAMPED4");

    // 每行: label + value
    ips200_show_string(16, 2 * 15, "AHRS_KP:");
    ips200_show_float(16 * 9, 2 * 15, cfg->ahrs_kp, 5, 2);

    ips200_show_string(16, 3 * 15, "YAW_KP :");
    ips200_show_float(16 * 9, 3 * 15, cfg->yaw.kp, 5, 2);

    ips200_show_string(16, 4 * 15, "ANG_KP :");
    ips200_show_float(16 * 9, 4 * 15, cfg->angle.kp, 5, 2);
    
    ips200_show_string(16, 5 * 15, "RATE_KP:");
    ips200_show_float(16 * 9, 5 * 15, cfg->rate.kp, 5, 2);

    ips200_show_string(16, 6 * 15, "DAMP_K :");
    ips200_show_float(16 * 9, 6 * 15, cfg->damp_k, 5, 2);

    ips200_show_string(16, 7 * 15, "DAMP_MX:");
    ips200_show_float(16 * 9, 7 * 15, cfg->damp_max, 5, 2);

    ips200_show_string(16, 8 * 15, "MOT_TGT:");
    ips200_show_int(16 * 9, 8 * 15, cfg->motor_target_counts_5ms, 6);

    // 光标：行号 = 字段序号 + 1（因为从第2行开始显示字段）
    ips200_show_string(0, (uint16)((1u + (uint16)s_kkf_pid_item) * 15u), cursor);

    kkf_draw_signature_bottom();
}

static void kkf_pid_adjust_value(pid_mode_t mode, kkf_pid_field_t field, int8 dir)
{
    // dir: +1 / -1
    pid_mode_cfg_t *cfg = &s_pid_mode_cfg[(uint16)mode];

    // 步进：kp 0.01；damp 0.1；motor 1
    if (field == KKF_PID_FIELD_AHRS_KP)
    {
        cfg->ahrs_kp += (dir > 0) ? 0.01f : -0.01f;
        if (cfg->ahrs_kp < 0.0f) cfg->ahrs_kp = 0.0f;
    }
    else if (field == KKF_PID_FIELD_YAW_KP)
    {
        cfg->yaw.kp += (dir > 0) ? 0.01f : -0.01f;
        if (cfg->yaw.kp < 0.0f) cfg->yaw.kp = 0.0f;
    }
    else if (field == KKF_PID_FIELD_ANGLE_KP)
    {
        cfg->angle.kp += (dir > 0) ? 0.1f : -0.1f;
        if (cfg->angle.kp < 0.0f) cfg->angle.kp = 0.0f;
    }
    else if (field == KKF_PID_FIELD_RATE_KP)
    {
        cfg->rate.kp += (dir > 0) ? 0.1f : -0.1f;
        if (cfg->rate.kp < 0.0f) cfg->rate.kp = 0.0f;
    }
    else if (field == KKF_PID_FIELD_DAMP_K)
    {
        cfg->damp_k += (dir > 0) ? 0.1f : -0.1f;
        if (cfg->damp_k < 0.0f) cfg->damp_k = 0.0f;
    }
    else if (field == KKF_PID_FIELD_DAMP_MAX)
    {
        cfg->damp_max += (dir > 0) ? 1.0f : -1.0f;
        if (cfg->damp_max < 0.0f) cfg->damp_max = 0.0f;
    }
    else // MOTOR_TGT
    {
        int32 v = (int32)cfg->motor_target_counts_5ms + ((dir > 0) ? 1 : -1);
        if (v < 0) v = 0;
        if (v > 2000) v = 2000;
        cfg->motor_target_counts_5ms = (int16)v;
    }
}

// // 旧版菜单显示控制/状态信息
// volatile uint8 menu_deep=1;     
// volatile uint8 sec_menu_item=1; 
// volatile uint8 main_menu_item=1;
 volatile uint8 menu_show_flag=1;
// volatile uint8 data_change_flag=0;

float ips_show_pid_out;
uint16 count_pid_main;
int16 show_duty;

float show_roll_deg;
float show_pitch_deg;
float show_yaw_deg;
float show_pitch_rate_dps_f;

extern volatile int16 motor_speed_raw_counts;

// IPS200 的 16x16 字体在驱动里标注为“暂不支持”，因此这里通过：
// 1) 强制使用 8x16 字体；2) 使用 16 的行距
// 来实现“比原来(15行距)更大一号/更清晰”的显示。
#define KKF_LINE_H      (16u)

static void kkf_ui_prepare_font(void)
{
    ips200_set_font(IPS200_8X16_FONT);
}

void User_Interface(void)//UI显示
{

    if(menu_show_flag==1)
    {
        kkf_ui_prepare_font();
        if (s_ui_page == UI_PAGE_KKF_MAIN)
        {
            kkf_main_draw();
        }
        else if (s_ui_page == UI_PAGE_KKF_PARAM)
        {
            kkf_param_draw();
        }
        else if (s_ui_page == UI_PAGE_KKF_MODE_SELECT)
        {
            kkf_mode_select_draw();
        }
        else if (s_ui_page == UI_PAGE_KKF_EIGHT2_PROG_SELECT)
        {
            kkf_eight2_prog_select_draw();
        }
        else if (s_ui_page == UI_PAGE_KKF_PID_MODE_SELECT)
        {
            kkf_pid_mode_select_draw();
        }
        else if (s_ui_page == UI_PAGE_KKF_GPS_DEBUG)
        {
            kkf_gps_debug_draw();
        }

        else if (s_ui_page == UI_PAGE_KKF_GPS_ROUTE)
        {
            // 专门界面：绘制已保存路线（点/线）
            // 使用 gps_display 提供的绘制函数
//            ips200_show_string(0, 0 * 15, "KOU_KOU_FLY_CAR");
            ttop_gps_display_route();//            kkf_draw_signature_bottom();
        }
        else if (s_ui_page == UI_PAGE_KKF_GPS_HEADINGS)
        {
            // 专门界面：计算并显示多段航向角
            ttop_gps_calc_and_show_headings();
        }
        else if (s_ui_page == UI_PAGE_KKF_INS_PATH_VIEW)
        {
            kkf_ins_path_draw();
        }
        else if (s_ui_page == UI_PAGE_KKF_PID_CASCADE ||
                 s_ui_page == UI_PAGE_KKF_PID_CASCADE2 ||
                 s_ui_page == UI_PAGE_KKF_PID_CASCADE3 ||
                 s_ui_page == UI_PAGE_KKF_PID_ANGLE ||
                 s_ui_page == UI_PAGE_KKF_PID_DAMPED_TWO ||
                 s_ui_page == UI_PAGE_KKF_PID_DAMPED_THREE ||
                 s_ui_page == UI_PAGE_KKF_PID_DAMPED_FOUR)
        {
            kkf_pid_draw_for_mode(kkf_page_to_pid_mode(s_ui_page));
        }
        else
        {
            kkf_eight_turn_draw();
        }
    }
    else
    {
        return;
    }
}

void Menu_Select(void)
{
    uint8 key = 0;
    static uint8 last_key = 0; // 上一次的“事件键值”

    // 组合键：KEY2(确认) + KEY4(返回) 同时按下 -> 切换屏幕显示开关
    // 用锁存避免长按导致反复翻转：必须先松开组合键才能再次触发。
    static uint8 k24_latched = 0u;

    // 组合键：KEY1(上) + KEY3(下) 同时按下 -> 保存参数到 Flash
    // 用锁存避免长按导致反复保存：必须先松开组合键才能再次触发。
    static uint8 k13_latched = 0u;

    // Key 模块现在只提供稳定按下状态 Key_Read(i)。
    // 这里用“边沿检测”把稳定状态转换为单次触发事件，保持原菜单逻辑不变。
    static uint8 k1_last = 0u;
    static uint8 k2_last = 0u;
    static uint8 k3_last = 0u;
    static uint8 k4_last = 0u;

    uint8 k1_now = Key_Read(KEY_ID_1);
    uint8 k2_now = Key_Read(KEY_ID_2);
    uint8 k3_now = Key_Read(KEY_ID_3);
    uint8 k4_now = Key_Read(KEY_ID_4);

    // 检测组合键（稳定态）：先于单键事件处理，避免冲突。
    if (k2_now && k4_now)
    {
        if (!k24_latched)
        {
            k24_latched = 1u;
            menu_show_flag = (uint8)(!menu_show_flag);
            ips200_clear();
        }
        // 组合键按下时，不再生成单键事件
        k1_last = k1_now;
        k2_last = k2_now;
        k3_last = k3_now;
        k4_last = k4_now;
        last_key = 0u;
        return;
    }
    else
    {
        // 组合键已松开，允许下次触发
        k24_latched = 0u;
    }

    if (k1_now && k3_now)
    {
        if (!k13_latched)
        {
            uint8 ok1;
            uint8 ok2;
            uint8 ok3;
            uint8 ok4;
            k13_latched = 1u;

            ok1 = flash_storage_save_pid_mode_cfg();
            ok2 = flash_storage_save_eight2_prog();
            ok3 = flash_storage_save_eight2_prog_ex1();
            ok4 = flash_storage_save_eight2_prog_ex2();
            flash_storage_save_ins_path(); 
            
            // 轻量提示：显示 SAVE OK/FAIL
            if (ok1 && ok2 && ok3 && ok4)
            {
                ips200_show_string(16 * 8, 0, "SAVE OK");
            }
            else
            {
                ips200_show_string(16 * 8, 0, "SAVE FAIL");
                // 显示单项结果，便于定位是哪一类保存失败
                // ips200_show_int(16 * 8, 16, (int32)ok1, 1);
                // ips200_show_int(16 * 9, 16, (int32)ok2, 1);
                // ips200_show_int(16 * 10, 16, (int32)ok3, 1);
                // ips200_show_int(16 * 11, 16, (int32)ok4, 1);
            }
            ttop_funtion_mode_once_apply();
        }

        // 组合键按下时，不再生成单键事件
        k1_last = k1_now;
        k2_last = k2_now;
        k3_last = k3_now;
        k4_last = k4_now;
        last_key = 0u;
        return;
    }
    else
    {
        // 组合键已松开，允许下次触发
        k13_latched = 0u;
    }

    if (k1_now && !k1_last) key = KEY1_PRES;
    else if (k2_now && !k2_last) key = KEY2_PRES;
    else if (k3_now && !k3_last) key = KEY3_PRES;
    else if (k4_now && !k4_last) key = KEY4_PRES;

    k1_last = k1_now;
    k2_last = k2_now;
    k3_last = k3_now;
    k4_last = k4_now;
    if (key && !last_key)
    {
        // =========================
        // 按键语义：KEY1上 / KEY2确认 / KEY3下 / KEY4返回
        // =========================
        if (s_ui_page == UI_PAGE_KKF_MAIN)
        {
            if (KEY3_PRES == key)
            {
                if (s_kkf_main_item > 1u) s_kkf_main_item--;
                else s_kkf_main_item = 14u;
                ips200_clear();
            }
            else if (KEY1_PRES == key)
            {
                if (s_kkf_main_item < 14u) s_kkf_main_item++;
                else s_kkf_main_item = 1u;
                ips200_clear();
            }
            else if (KEY2_PRES == key)
            {
                // 1:PID_CONFIG 2:START 3:RESET 4:CLEAR 5:MT_ST 6:STATE 7:MODE 8:yaw 9:t_yaw 10:pitch 11:SPD 12:xy 13: (reserved) 14:g_ins_path
                if (s_kkf_main_item == 1u)
                {
                    s_ui_page = UI_PAGE_KKF_PARAM;
                    s_kkf_param_item = 1u;
                    ips200_clear();
                }
                else if (s_kkf_main_item == 2u)
                {
                    // START：开始跑（使能 PID/ENC 等周期任务）
                    funtion_ready_flag = 1u;
                   //motor_request_start(s_menu_moto_mode);
                    ips200_clear();
                }
                else if (s_kkf_main_item == 3u)
                {
                    // RESET：停止跑 + 同时进行与 CLEAR 类似的“状态复位”
                    
                    encoder_clear_demo5_count_total();
                    ahrs_yaw_zero_request_set();//四元数重置
                    ttop_eight2_reset_now();//八字步复位
                    small_driver_set_duty(0, 0);//电机停止
                    servo_zero();//舵机归零
                    ins_dr_init();//位置重置
                    ins_path_follow_init();//航点跟随状态重置
                    //motor_request_stop(true);// 电机停止
                    //motor_clear_speed();// 电机速度相关状态清零
                    eight_flag = 0u;
                    eight_count = 0u;
                    funtion_ready_flag = 0u;
                    ips200_clear();
                }
                else if (s_kkf_main_item == 4u)
                {
                    // 清零：对齐原 ttop_key_task_5ms 的行为
                    ahrs_yaw_zero_request_set();
                    ttop_eight2_reset_now();
                    eight_flag = 0u;
                    eight_count = 0u;
                    ips200_clear();
                }
                else if (s_kkf_main_item == 5u)
                {
                    // 进入 GPS 调试入口（原 MT_ST 插槽）
                    s_ui_page = UI_PAGE_KKF_GPS_DEBUG;
                    s_kkf_gps_item = 1u;
                    ips200_clear();
                }
                else if (s_kkf_main_item == 14u)
                {
                    // 航点查看
                    s_ui_page = UI_PAGE_KKF_INS_PATH_VIEW;
                    s_kkf_ins_path_page = 0u;
                    ips200_clear();
                }
                else
                {
                    // 其它项：暂不进入编辑
                    ips200_clear();

                }
            }
            else if (KEY4_PRES == key)
            {
                // 主界面按返回：保持在主界面
                ips200_clear();
            }
        }
        else if (s_ui_page == UI_PAGE_KKF_GPS_ROUTE)
        {
            // 路线显示页：按 KEY4 返回到 GPS 菜单
            if (KEY4_PRES == key)
            {
                s_ui_page = UI_PAGE_KKF_GPS_DEBUG;
                ips200_clear();
            }
            else
            {
                // 其它键暂不处理（可在此扩展缩放/翻页功能）
            }
        }
        else if (s_ui_page == UI_PAGE_KKF_GPS_HEADINGS)
        {
            // 航向显示页：按 KEY4 返回到 GPS 菜单
            if (KEY4_PRES == key)
            {
                s_ui_page = UI_PAGE_KKF_GPS_DEBUG;
                ips200_clear();
            }
            else
            {
                // 其它键暂不处理
            }
        }
        else if (s_ui_page == UI_PAGE_KKF_INS_PATH_VIEW)
        {
            // 航点查看页：KEY1/KEY3 翻页；KEY4 返回
            uint16 count = g_ins_path_count;
            if (count > (uint16)INS_PATH_MAX_POINTS) count = (uint16)INS_PATH_MAX_POINTS;

            uint16 rows = kkf_ui_rows_available();
            uint16 points_per_page = (rows > 2u) ? (uint16)(rows - 2u) : 1u;
            if (points_per_page == 0u) points_per_page = 1u;

            uint16 page_count = 1u;
            if (count > 0u)
            {
                page_count = (uint16)((count + points_per_page - 1u) / points_per_page);
                if (page_count == 0u) page_count = 1u;
            }

            if (KEY4_PRES == key)
            {
                s_ui_page = UI_PAGE_KKF_MAIN;
                ips200_clear();
            }
            else if (KEY3_PRES == key)
            {
                // 上一页
                if (s_kkf_ins_path_page > 0u) s_kkf_ins_path_page--;
                else s_kkf_ins_path_page = (uint16)(page_count - 1u);
                ips200_clear();
            }
            else if (KEY1_PRES == key)
            {
                // 下一页
                if (s_kkf_ins_path_page + 1u < page_count) s_kkf_ins_path_page++;
                else s_kkf_ins_path_page = 0u;
                ips200_clear();
            }
            else
            {
                // KEY2：不处理
            }
        }
        else if (s_ui_page == UI_PAGE_KKF_GPS_DEBUG)
        {
            // GPS 调试菜单：KEY1 上 / KEY3 下 / KEY2 确认（暂为空动作） / KEY4 返回
            if (KEY4_PRES == key)
            {
                s_ui_page = UI_PAGE_KKF_MAIN;
                ips200_clear();
            }
            else if (KEY3_PRES == key)
            {
                if (s_kkf_gps_item > 1u) s_kkf_gps_item--;
                else s_kkf_gps_item = 4u;
                ips200_clear();
            }
            else if (KEY1_PRES == key)
            {
                if (s_kkf_gps_item < 4u) s_kkf_gps_item++;
                else s_kkf_gps_item = 1u;
                ips200_clear();
            }
            else if (KEY2_PRES == key)
            {
                // GPS 选项确认：1=保存点；2=显示路线（切到专门界面）；3=显示航向（切到专门界面）；4=清屏
                if (s_kkf_gps_item == 1u)
                {
                    // 保存点并在同一行显示确认
                    ttop_gps_save_point();
                    ips200_show_uint(16 * 8, (uint16)((uint16)s_kkf_gps_item * 15u), gps_saved_point_count, 3);//gps_test_count,gps_saved_point_count


                }
                else if (s_kkf_gps_item == 2u)
                {
                    // 进入路线显示界面，由专门的绘制函数负责绘制
                    s_ui_page = UI_PAGE_KKF_GPS_ROUTE;
                    ips200_clear();
                }
                else if (s_kkf_gps_item == 3u)
                {
                    // 进入航向显示界面
                    s_ui_page = UI_PAGE_KKF_GPS_HEADINGS;
                    ips200_clear();
                }
                else
                {
                    // 第4项：清屏并显示帮助
                    ttop_gps_clear_and_show_help();
                }
            }
        }
        else if (s_ui_page == UI_PAGE_KKF_PARAM)
        {
            // 1:MODE_SEL 2:EIGHT_TURN 3:E8_PROG 4:PID_CHANGE 5:PID_MODE
            if (KEY3_PRES == key)
            {
                if (s_kkf_param_item > 1u) s_kkf_param_item--;
                else s_kkf_param_item = 5u;
                ips200_clear();
            }
            else if (KEY1_PRES == key)
            {
                if (s_kkf_param_item < 5u) s_kkf_param_item++;
                else s_kkf_param_item = 1u;
                ips200_clear();
            }
            else if (KEY2_PRES == key)
            {
                if (s_kkf_param_item == 1u)
                {
                    s_ui_page = UI_PAGE_KKF_MODE_SELECT;
                    s_kkf_mode_item = s_kkf_mode; // 默认指到当前模式
                    ips200_clear();
                }
                else if (s_kkf_param_item == 2u)
                {
                    s_ui_page = UI_PAGE_KKF_EIGHT_TURN;
                    s_kkf_eight_turn_item = 1u;
                    ips200_clear();
                }
                else if (s_kkf_param_item == 3u)
                {
                    s_ui_page = UI_PAGE_KKF_EIGHT2_PROG_SELECT;
                    // 进入时光标对齐当前选择
                    if (s_kkf_eight2_prog_id == TTOP_EIGHT2_PROG_EX1) s_kkf_eight2_prog_item = 2u;
                    else if (s_kkf_eight2_prog_id == TTOP_EIGHT2_PROG_EX2) s_kkf_eight2_prog_item = 3u;
                    else s_kkf_eight2_prog_item = 1u;
                    ips200_clear();
                }
                else if (s_kkf_param_item == 4u)
                {
                    // PID_CHANGE：进入 PID 编辑页（按当前 PID_MODE）
                    s_ui_page = kkf_pid_mode_to_page((pid_mode_t)s_kkf_pid_mode_id);
                    s_kkf_pid_state = KKF_PID_STATE_SEL;
                    s_kkf_pid_item = 1u;
                    ips200_clear();
                }
                else if (s_kkf_param_item == 5u)
                {
                    // PID_MODE：选择要编辑哪套 PID 参数
                    s_ui_page = UI_PAGE_KKF_PID_MODE_SELECT;
                    s_kkf_pid_mode_item = kkf_pid_mode_to_item((pid_mode_t)s_kkf_pid_mode_id);
                    ips200_clear();
                }
                else
                {
                    ips200_clear();
                }
            }
            else if (KEY4_PRES == key)
            {
                s_ui_page = UI_PAGE_KKF_MAIN;
                ips200_clear();
            }
        }
        else if (s_ui_page == UI_PAGE_KKF_EIGHT_TURN)
        {
            // eight_turn：三段式状态机
            // 1) 选择行：'>'
            //    - KEY1/KEY3：移动行
            //    - KEY2：进入 编辑deg（'-'）
            // 2) 编辑deg：'-'
            //    - KEY1：减一；KEY3：加一
            //    - KEY2：保存并进入 编辑degs（'*'）
            // 3) 编辑degs：'*'
            //    - KEY1：减一；KEY3：加一
            //    - KEY2：保存并回到 选择行（'>'）
            // KEY4：返回 pid_config（从任何状态返回都会回到选择态）

            uint16 idx = (uint16)s_kkf_eight_turn_item;
            volatile ttop_eight2_seg_t *prog = kkf_eight2_prog_ptr((ttop_eight2_prog_id_t)s_kkf_eight2_prog_id);

            if (KEY4_PRES == key)
            {
                s_ui_page = UI_PAGE_KKF_PARAM;
                s_kkf_eight_turn_state = KKF_EIGHT_TURN_SEL;
                ips200_clear();
            }
            else if (s_kkf_eight_turn_state == KKF_EIGHT_TURN_SEL)
            {
                if (KEY3_PRES == key)
                {
                    if (s_kkf_eight_turn_item > 0u) s_kkf_eight_turn_item--;
                    else s_kkf_eight_turn_item = (uint8)(TTOP_EIGHT2_PROG_LEN - 1u);
                    ips200_clear();
                }
                else if (KEY1_PRES == key)
                {
                    if (s_kkf_eight_turn_item < (uint8)(TTOP_EIGHT2_PROG_LEN - 1u)) s_kkf_eight_turn_item++;
                    else s_kkf_eight_turn_item = 0u;
                    ips200_clear();
                }
                else if (KEY2_PRES == key)
                {
                    s_kkf_eight_turn_state = KKF_EIGHT_TURN_EDIT_DEG;
                    ips200_clear();
                }
            }
            else if (s_kkf_eight_turn_state == KKF_EIGHT_TURN_EDIT_DEG)
            {
                if (KEY1_PRES == key)
                {
                    prog[idx].target_rel_deg -= 1.0f;
                    ips200_clear();
                }
                else if (KEY3_PRES == key)
                {
                    prog[idx].target_rel_deg += 1.0f;
                    ips200_clear();
                }
                else if (KEY2_PRES == key)
                {
                    // 保存（已直接写回表），进入下一步
                    s_kkf_eight_turn_state = KKF_EIGHT_TURN_EDIT_DPS;
                    ips200_clear();
                }
            }
            else // KKF_EIGHT_TURN_EDIT_DPS
            {
                if (KEY1_PRES == key)
                {
                    prog[idx].rate_dps -= 1.0f;
                    ips200_clear();
                }
                else if (KEY3_PRES == key)
                {
                    prog[idx].rate_dps += 1.0f;
                    ips200_clear();
                }
                else if (KEY2_PRES == key)
                {
                    // 保存（已直接写回表），回到选择态
                    s_kkf_eight_turn_state = KKF_EIGHT_TURN_SEL;
                    ips200_clear();
                }
            }
        }
        else if (s_ui_page == UI_PAGE_KKF_EIGHT2_PROG_SELECT)
        {
            // 段表选择页：KEY1/KEY3 移动；KEY2 确认并切换；KEY4 返回
            if (KEY4_PRES == key)
            {
                s_ui_page = UI_PAGE_KKF_PARAM;
                ips200_clear();
            }
            else if (KEY3_PRES == key)
            {
                if (s_kkf_eight2_prog_item > 1u) s_kkf_eight2_prog_item--;
                else s_kkf_eight2_prog_item = 3u;
                ips200_clear();
            }
            else if (KEY1_PRES == key)
            {
                if (s_kkf_eight2_prog_item < 3u) s_kkf_eight2_prog_item++;
                else s_kkf_eight2_prog_item = 1u;
                ips200_clear();
            }
            else if (KEY2_PRES == key)
            {
                if (s_kkf_eight2_prog_item == 2u) s_kkf_eight2_prog_id = TTOP_EIGHT2_PROG_EX1;
                else if (s_kkf_eight2_prog_item == 3u) s_kkf_eight2_prog_id = TTOP_EIGHT2_PROG_EX2;
                else s_kkf_eight2_prog_id = TTOP_EIGHT2_PROG_MAIN;

                // 同步到八字模块：运行时从选中段表输出 g_yaw_target_deg
                ttop_eight2_select_program((ttop_eight2_prog_id_t)s_kkf_eight2_prog_id);

                s_ui_page = UI_PAGE_KKF_PARAM;
                ips200_clear();
            }
        }
        else if (s_ui_page == UI_PAGE_KKF_PID_MODE_SELECT)
        {
            // PID_MODE 选择页：KEY1/KEY3 移动；KEY2 确认；KEY4 返回
            if (KEY4_PRES == key)
            {
                s_ui_page = UI_PAGE_KKF_PARAM;
                ips200_clear();
            }
            else if (KEY3_PRES == key)
            {
                if (s_kkf_pid_mode_item > 1u) s_kkf_pid_mode_item--;
                else s_kkf_pid_mode_item = 7u;
                ips200_clear();
            }
            else if (KEY1_PRES == key)
            {
                if (s_kkf_pid_mode_item < 7u) s_kkf_pid_mode_item++;
                else s_kkf_pid_mode_item = 1u;
                ips200_clear();
            }
            else if (KEY2_PRES == key)
            {
                s_kkf_pid_mode_id = kkf_item_to_pid_mode(s_kkf_pid_mode_item);
                s_ui_page = UI_PAGE_KKF_PARAM;
                ips200_clear();
            }
        }
        else if (s_ui_page == UI_PAGE_KKF_MODE_SELECT)
        {
            // 1~12: MODE 1..12（与 funtion_kemu 对齐）
            if (KEY3_PRES == key)
            {
                if (s_kkf_mode_item > 1u) s_kkf_mode_item--;
                else s_kkf_mode_item = 12u;
                ips200_clear();
            }
            else if (KEY1_PRES == key)
            {
                if (s_kkf_mode_item < 12u) s_kkf_mode_item++;
                else s_kkf_mode_item = 1u;
                ips200_clear();
            }
            else if (KEY2_PRES == key)
            {
                // 确认选择
                s_kkf_mode = s_kkf_mode_item;

                // 关键：在“确认键”这一刻立刻把模式应用到顶层，否则会出现
                // “光标选中了但需要再进一次模式选择界面才生效”的体验。
                // 顶层 funtion_kemu：1/2/3 对应菜单 MODE 1/2/3
                funtion_kemu = (uint8)(s_kkf_mode);
                ttop_funtion_mode_once_apply();

                s_ui_page = UI_PAGE_KKF_PARAM;
                ips200_clear();
            }
            else if (KEY4_PRES == key)
            {
                // 返回不更改模式
                s_ui_page = UI_PAGE_KKF_PARAM;
                ips200_clear();
            }
        }

    else if (s_ui_page == UI_PAGE_KKF_PID_CASCADE ||
         s_ui_page == UI_PAGE_KKF_PID_CASCADE2 ||
         s_ui_page == UI_PAGE_KKF_PID_CASCADE3 ||
                 s_ui_page == UI_PAGE_KKF_PID_ANGLE ||
                 s_ui_page == UI_PAGE_KKF_PID_DAMPED_TWO ||
                 s_ui_page == UI_PAGE_KKF_PID_DAMPED_THREE ||
                 s_ui_page == UI_PAGE_KKF_PID_DAMPED_FOUR)
        {
            // PID 参数编辑页：
            // - 选择态('>')：KEY1/KEY3 移动字段；KEY2 进入编辑态('-')
            // - 编辑态('-')：KEY1/KEY3 调值；KEY2 保存并调用一次 apply；回到选择态
            // - KEY4：返回参数页（并复位到选择态）

            pid_mode_t mode = kkf_page_to_pid_mode(s_ui_page);

            if (KEY4_PRES == key)
            {
                s_ui_page = UI_PAGE_KKF_PARAM;
                s_kkf_pid_state = KKF_PID_STATE_SEL;
                s_kkf_pid_item = 1u;
                ips200_clear();
            }
            else if (s_kkf_pid_state == KKF_PID_STATE_SEL)
            {
                if (KEY3_PRES == key)
                {
                    if (s_kkf_pid_item > 1u) s_kkf_pid_item--;
                    else s_kkf_pid_item = (uint8)KKF_PID_FIELD_COUNT;
                    ips200_clear();
                }
                else if (KEY1_PRES == key)
                {
                    if (s_kkf_pid_item < (uint8)KKF_PID_FIELD_COUNT) s_kkf_pid_item++;
                    else s_kkf_pid_item = 1u;
                    ips200_clear();
                }
                else if (KEY2_PRES == key)
                {
                    s_kkf_pid_state = KKF_PID_STATE_EDIT;
                    ips200_clear();
                }
            }
            else // EDIT
            {
                if (KEY1_PRES == key)
                {
                    kkf_pid_adjust_value(mode, (kkf_pid_field_t)s_kkf_pid_item, -1);
                    ips200_clear();
                }
                else if (KEY3_PRES == key)
                {
                    kkf_pid_adjust_value(mode, (kkf_pid_field_t)s_kkf_pid_item, +1);
                    ips200_clear();
                }
                else if (KEY2_PRES == key)
                {
                    // 保存/应用：按你的约定，调用一次模式 apply 触发 PID_INIT_BY_MODE
                    ttop_funtion_mode_once_apply();
                    s_kkf_pid_state = KKF_PID_STATE_SEL;
                    ips200_clear();
                }
            }
        }

    }
    last_key = key;
}
