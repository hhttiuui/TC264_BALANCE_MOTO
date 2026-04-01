/*
 * ins_path_follow.c
 *
 *  Created on: 2026-03-24
 *      Author: hhttiuui
 *
 *  惯性导航 - 路径跟踪与示教录制模块
 *
 *  ===================== Pure Pursuit 算法简介 =====================
 *  Pure Pursuit（纯追踪）是一种经典的路径跟踪算法：
 *    1. 在目标路径上找到距车辆最近的点（nearest）
 *    2. 从 nearest 沿路径往前找一个"预瞄点"（look-ahead point），
 *       该点与车辆的距离 ≈ 预瞄距离 L_d
 *    3. 计算车辆当前位置到预瞄点的航向角
 *    4. 将该航向角作为 g_yaw_target_deg，由 PID 链路执行转向
 *
 *  预瞄距离 L_d 的作用：
 *    L_d 越大 → 转弯越提前、越平滑，但跟踪精度略低
 *    L_d 越小 → 跟踪越紧凑，但容易产生振荡（尤其弯道处）
 *    经验值：车速(m/s) × (0.5~1.5)，低速小车建议 0.15~0.50m
 */

#include "zf_common_headfile.h"
#include "ins_path_follow.h"
#include "ins_dead_reckoning.h"

/* PI 常量 */
#ifndef INS_PF_PI
#define INS_PF_PI  (3.14159265358979f)
#endif

/* ================================================================
 *  航点表（全局，对外暴露）
 * ================================================================ */
volatile ins_waypoint_t g_ins_path[INS_PATH_MAX_POINTS];
volatile uint16         g_ins_path_count = 0u;

/* ================================================================
 *  内部状态：录制
 * ================================================================ */
static uint8  s_record_active      = 0u;     /* 1=录制中 0=未录制 */
static float  s_record_last_odo_m  = 0.0f;   /* 上次录制点的里程 */
static uint8  s_straight_mode      = 0u;     /* 迟滞状态：1=当前判定为直线 */

/* ================================================================
 *  内部状态：跟踪
 * ================================================================ */
static uint8  s_follow_done     = 0u;   /* 1=已到终点 */
static uint16 s_nearest_idx     = 0u;   /* 最近航点索引 */
static uint16 s_lookahead_idx   = 0u;   /* 预瞄航点索引 */
static float  s_cte             = 0.0f; /* 横向偏差（米） */

/* ================================================================
 *  辅助函数
 * ================================================================ */

/* 两点距离的平方（避免 sqrt，用于比较大小时更快） */
static inline float ins_dist2(float x1, float y1, float x2, float y2)
{
    float dx = x2 - x1;
    float dy = y2 - y1;
    return dx * dx + dy * dy;
}

/* 两点距离 */
static inline float ins_distf(float x1, float y1, float x2, float y2)
{
    return sqrtf(ins_dist2(x1, y1, x2, y2));
}

/* 角度归一化到 [-180, +180) */
static inline float ins_wrap_deg(float deg)
{
    while (deg >  180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

/* ================================================================
 *  录制滤波：滑动窗口直线判定 + 迟滞 + 弯道不过滤
 *
 *  算法流程：
 *    1) 取窗口内最近 N 个点（含新点）的连续线段
 *    2) 计算每段航向角，再计算相邻航向角之差
 *    3) 用迟滞判定当前是否"直线模式"：
 *       - 未进入直线模式时：所有角差 < ENTER 阈值 → 进入
 *       - 已在直线模式时：任一角差 > EXIT  阈值 → 退出
 *    4) 如果是直线模式：
 *       - 取最后一段 (P[n-1] → P_raw) 的方向为参考线
 *       - 计算新点到参考线的横向偏差
 *       - 若偏差小 → 投影到参考线
 *       - 若偏差大 → 保持原点（异常跳点保护）
 *    5) 如果不是直线模式：不做任何修改（保留弯道原始点）
 * ================================================================ */
static void ins_record_filter_point(float *px, float *py)
{
    if ((float *)0 == px || (float *)0 == py) return;

    /* 窗口内需要 (WINDOW_SIZE - 1) 个已有点 + 1 个新点 */
    uint16 win = (uint16)INS_FILTER_WINDOW_SIZE;
    if (g_ins_path_count < (uint16)(win - 1u)) return;   /* 点太少，跳过 */

    /* ---- 收集窗口内各段的航向角 ---- */
    /* 段数 = win-1 = 3  (P[n-3]→P[n-2], P[n-2]→P[n-1], P[n-1]→P_raw) */
    uint16 n_segs = (uint16)(win - 1u);
    float  angles[3];    /* 最多 3 段（窗口 = 4） */
    uint8  seg_ok = 1u;  /* 是否所有段长度都合格 */

    uint16 i;
    for (i = 0u; i < n_segs; i++)
    {
        float ax, ay, bx, by;

        if (i < (uint16)(n_segs - 1u))
        {
            /* 已有航点之间的段 */
            uint16 idx_a = (uint16)(g_ins_path_count - (win - 1u) + i);
            uint16 idx_b = (uint16)(idx_a + 1u);
            ax = g_ins_path[idx_a].x;
            ay = g_ins_path[idx_a].y;
            bx = g_ins_path[idx_b].x;
            by = g_ins_path[idx_b].y;
        }
        else
        {
            /* 最后一段：P[n-1] → P_raw（新点） */
            uint16 idx_a = (uint16)(g_ins_path_count - 1u);
            ax = g_ins_path[idx_a].x;
            ay = g_ins_path[idx_a].y;
            bx = *px;
            by = *py;
        }

        float sdx = bx - ax;
        float sdy = by - ay;
        float slen = sqrtf(sdx * sdx + sdy * sdy);
        if (slen < (float)INS_FILTER_MIN_SEG_M)
        {
            seg_ok = 0u;
            break;
        }
        angles[i] = atan2f(sdy, sdx) * (180.0f / INS_PF_PI);
    }

    /* 如果有任何段太短 → 方向角不可靠，跳过滤波，保持原点 */
    if (!seg_ok)
    {
        s_straight_mode = 0u;
        return;
    }

    /* ---- 计算相邻段的航向角差 ---- */
    /* 角差数 = n_segs - 1 = 2 */
    float max_angle_diff = 0.0f;
    for (i = 0u; i + 1u < n_segs; i++)
    {
        float diff = fabsf(ins_wrap_deg(angles[i + 1u] - angles[i]));
        if (diff > max_angle_diff) max_angle_diff = diff;
    }

    /* ---- 迟滞判定：是否进入/退出直线模式 ---- */
    if (s_straight_mode == 0u)
    {
        /* 未在直线模式 → 用 ENTER 阈值（更严格）判定是否进入 */
        if (max_angle_diff < (float)INS_FILTER_ANGLE_ENTER_DEG)
        {
            s_straight_mode = 1u;
        }
    }
    else
    {
        /* 已在直线模式 → 用 EXIT 阈值（更宽松）判定是否退出 */
        if (max_angle_diff > (float)INS_FILTER_ANGLE_EXIT_DEG)
        {
            s_straight_mode = 0u;
        }
    }

    /* ---- 如果不在直线模式 → 弯道，不做任何修改 ---- */
    if (s_straight_mode == 0u) return;

    /* ---- 在直线模式 → 将新点投影到最后一段方向线上 ---- */
    {
        uint16 ref_idx = (uint16)(g_ins_path_count - 1u);
        float rx = g_ins_path[ref_idx].x;
        float ry = g_ins_path[ref_idx].y;

        /* 参考方向：用最后一段已有线段的方向（更稳定） */
        float dx, dy, seg_len;
        if (g_ins_path_count >= 2u)
        {
            uint16 ref_prev = (uint16)(g_ins_path_count - 2u);
            dx = g_ins_path[ref_idx].x - g_ins_path[ref_prev].x;
            dy = g_ins_path[ref_idx].y - g_ins_path[ref_prev].y;
            seg_len = sqrtf(dx * dx + dy * dy);
        }
        else
        {
            dx = *px - rx;
            dy = *py - ry;
            seg_len = sqrtf(dx * dx + dy * dy);
        }

        if (seg_len < 0.001f) return;  /* 安全检查 */

        /* 计算新点相对参考点的横向偏差 */
        float vx = (*px) - rx;
        float vy = (*py) - ry;
        float lateral = fabsf((vx * dy - vy * dx) / seg_len);

        /* 横向偏差太大 → 异常点，不投影 */
        if (lateral > (float)INS_FILTER_LATERAL_TH_M) return;

        /* 沿参考方向投影 */
        float t = (vx * dx + vy * dy) / (seg_len * seg_len);
        if (t < 0.0f) t = 0.0f;  /* 不允许往回投影 */

        *px = rx + t * dx;
        *py = ry + t * dy;
    }
}


/* ================================================================
 *  A. 路径录制
 * ================================================================ */

void ins_path_record_init(void)
{
    /* 清空航点表 */
    g_ins_path_count = 0u;
    s_record_active = 1u;
    s_record_last_odo_m = 0.0f;
    s_straight_mode = 0u;

    /* 重置航迹推算：位置归零 */
    ins_dr_reset();

    /* 记录起始点 (0, 0) */
    if (g_ins_path_count < (uint16)INS_PATH_MAX_POINTS)
    {
        g_ins_path[g_ins_path_count].x = 0.0f;
        g_ins_path[g_ins_path_count].y = 0.0f;
        g_ins_path_count++;
    }
}

uint16 ins_path_record_task_5ms(void)
{
    if (!s_record_active) return g_ins_path_count;

    /* 航点表录满 → 自动停止 */
    if (g_ins_path_count >= (uint16)INS_PATH_MAX_POINTS)
    {
        s_record_active = 0u;
        return g_ins_path_count;
    }

    /* 更新航迹推算（位置 + 里程） */
    ins_dr_update_5ms();

    /* 检查是否行驶了足够的距离 → 录入新航点 */
    float odo = ins_dr_get_odometer_m();
    if ((odo - s_record_last_odo_m) >= (float)INS_RECORD_INTERVAL_M)
    {
        float rec_x = ins_dr_get_pos_x();
        float rec_y = ins_dr_get_pos_y();

        ins_record_filter_point(&rec_x, &rec_y);

        g_ins_path[g_ins_path_count].x = rec_x;
        g_ins_path[g_ins_path_count].y = rec_y;
        g_ins_path_count++;
        s_record_last_odo_m = odo;
    }

    return g_ins_path_count;
}

void ins_path_record_stop(void)
{
    if (!s_record_active) return;
    s_record_active = 0u;

    /* 补录最后一个点（当前位置），避免尾段丢失 */
    if (g_ins_path_count > 0u && g_ins_path_count < (uint16)INS_PATH_MAX_POINTS)
    {
        float cx = ins_dr_get_pos_x();
        float cy = ins_dr_get_pos_y();

        /* 与上一个航点距离太近（< 1cm）则不重复录入 */
        float last_x = g_ins_path[g_ins_path_count - 1u].x;
        float last_y = g_ins_path[g_ins_path_count - 1u].y;
        if (ins_distf(cx, cy, last_x, last_y) > 0.01f)
        {
            g_ins_path[g_ins_path_count].x = cx;
            g_ins_path[g_ins_path_count].y = cy;
            g_ins_path_count++;
        }
    }
}

uint8 ins_path_record_is_active(void) { return s_record_active; }

/* ================================================================
 *  B. 路径生成（数学公式）
 * ================================================================ */

void ins_path_gen_circle(float radius_m, uint16 num_points)
{
    if (num_points > (uint16)INS_PATH_MAX_POINTS) num_points = (uint16)INS_PATH_MAX_POINTS;
    if (num_points < 4u) num_points = 4u;

    float R = fabsf(radius_m);
    if (R < 0.01f) R = 0.01f;

    /* 正值 = 左转（逆时针），负值 = 右转（顺时针） */
    float dir = (radius_m >= 0.0f) ? 1.0f : -1.0f;

    /*
     * 圆形路径，起点 (0, 0)，初始朝向 +X（与 yaw=0° 一致）：
     *
     *   左转（dir=+1）：圆心 = (0, +R)
     *     x(t) = R × sin(t)
     *     y(t) = R × (1 - cos(t))
     *     t=0: (0, 0)  t=π/2: (R, R)  t=π: (0, 2R)
     *
     *   右转（dir=-1）：圆心 = (0, -R)
     *     x(t) = R × sin(t)
     *     y(t) = -R × (1 - cos(t))
     *     t=0: (0, 0)  t=π/2: (R, -R)  t=π: (0, -2R)
     *
     *   t ∈ [0, 2π) → 走完一整圈回到 (0,0)
     */
    g_ins_path_count = 0u;
    for (uint16 i = 0u; i < num_points; i++)
    {
        float t = (float)i * (2.0f * INS_PF_PI / (float)num_points);
        g_ins_path[i].x = R * sinf(t);
        g_ins_path[i].y = dir * R * (1.0f - cosf(t));
    }
    g_ins_path_count = num_points;
}

void ins_path_gen_eight(float radius_m, uint16 num_points_per_circle)
{
    float R = fabsf(radius_m);
    if (R < 0.01f) R = 0.01f;

    uint16 total = num_points_per_circle * 2u;
    if (total > (uint16)INS_PATH_MAX_POINTS) total = (uint16)INS_PATH_MAX_POINTS;
    uint16 npc = total / 2u;

    g_ins_path_count = 0u;

    /*
     * 8 字 = 先左转一圈 + 再右转一圈
     *
     * 左圆：圆心 (0, +R)，逆时针
     *   x(t) = R × sin(t)
     *   y(t) = R × (1 - cos(t))
     *   t ∈ [0, 2π)
     *   起点 (0,0)，终点回到 (0,0)
     *
     * 右圆：圆心 (0, -R)，顺时针
     *   x(t) = R × sin(t)
     *   y(t) = -R × (1 - cos(t))
     *   t ∈ [0, 2π)
     *   起点 (0,0)，终点回到 (0,0)
     */

    /* ---- 左圆（逆时针）---- */
    for (uint16 i = 0u; i < npc; i++)
    {
        float t = (float)i * (2.0f * INS_PF_PI / (float)npc);
        g_ins_path[i].x = R * sinf(t);
        g_ins_path[i].y = R * (1.0f - cosf(t));
    }

    /* ---- 右圆（顺时针）---- */
    for (uint16 i = 0u; i < npc; i++)
    {
        float t = (float)i * (2.0f * INS_PF_PI / (float)npc);
        g_ins_path[npc + i].x = R * sinf(t);
        g_ins_path[npc + i].y = -R * (1.0f - cosf(t));
    }

    g_ins_path_count = total;
}

/* ================================================================
 *  C. 路径跟踪（Pure Pursuit）
 * ================================================================ */

void ins_path_follow_init(void)
{
    /* 航迹推算归零：跟踪起点 = 坐标原点 */
    ins_dr_reset();

    /* 跟踪状态归零 */
    s_follow_done   = 0u;
    s_nearest_idx   = 0u;
    s_lookahead_idx = 0u;
    s_cte           = 0.0f;
}

uint8 ins_path_follow_task_5ms(void)
{
    /* 航点不足，无法跟踪 */
    if (g_ins_path_count < 2u)
    {
        s_follow_done = 1u;
        return 1u;
    }

    /* 已到达终点，保持最后一次输出 */
    if (s_follow_done) return 1u;

    /* ---- 1) 更新航迹推算 → 获取当前位置 ---- */
    ins_dr_update_5ms();

    float cx = ins_dr_get_pos_x();
    float cy = ins_dr_get_pos_y();

    /* ---- 2) 搜索最近航点（从上次 nearest 附近搜索，避免全表遍历）----
     *
     *  搜索窗口：[nearest - 5, nearest + 30]
     *  向前偏置更多，因为车只往前走，nearest 只应前进不后退
     */
    {
        uint16 best_idx = s_nearest_idx;
        float  best_d2  = ins_dist2(cx, cy,
                            g_ins_path[best_idx].x,
                            g_ins_path[best_idx].y);

        uint16 search_start = (s_nearest_idx > 5u) ? (s_nearest_idx - 5u) : 0u;
        uint16 search_end   = s_nearest_idx + 30u;
        if (search_end >= g_ins_path_count) search_end = g_ins_path_count - 1u;

        for (uint16 i = search_start; i <= search_end; i++)
        {
            float d2 = ins_dist2(cx, cy, g_ins_path[i].x, g_ins_path[i].y);
            if (d2 < best_d2)
            {
                best_d2  = d2;
                best_idx = i;
            }
        }
        s_nearest_idx = best_idx;
    }

    /* ---- 3) 搜索预瞄点（look-ahead point）----
     *
     *  从 nearest 开始向前找，直到某航点与当前位置的距离 >= 预瞄距离 L_d。
     *  若到路径末尾仍未达到 L_d，则使用最后一个航点。
     */
    {
        float La = (float)INS_PURSUIT_LOOKAHEAD_M;
        uint16 la_idx = s_nearest_idx;

        for (uint16 i = s_nearest_idx; i < g_ins_path_count; i++)
        {
            float d = ins_distf(cx, cy, g_ins_path[i].x, g_ins_path[i].y);
            la_idx = i;
            if (d >= La)
            {
                break;  /* 找到了满足预瞄距离的航点 */
            }
        }
        s_lookahead_idx = la_idx;
    }

    /* ---- 4) 终点判定 ----
     *
     *  条件：预瞄点已经是最后一个航点 且 距离终点 < 判定半径
     */
    {
        float dist_to_end = ins_distf(cx, cy,
            g_ins_path[g_ins_path_count - 1u].x,
            g_ins_path[g_ins_path_count - 1u].y);

        if (s_lookahead_idx >= g_ins_path_count - 1u
            && dist_to_end < (float)INS_PATH_END_RADIUS_M)
        {
            s_follow_done = 1u;
            return 1u;
        }
    }

    /* ---- 5) 计算目标航向角 → 写入 g_yaw_target_deg ----
     *
     *  目标航向 = atan2(goal_y - pos_y, goal_x - pos_x)
     *  该角度在与 show_yaw_deg 相同的坐标系中（0° = +X，逆时针为正）
     */
    {
        float goal_x = g_ins_path[s_lookahead_idx].x;
        float goal_y = g_ins_path[s_lookahead_idx].y;

        float target_rad = atan2f(goal_y - cy, goal_x - cx);
        float target_deg = target_rad * (180.0f / INS_PF_PI);

        g_yaw_target_deg = target_deg;
    }

    /* ---- 6) 计算横向偏差 CTE（用于调试/显示）----
     *
     *  方法：最近航点 → 下一个航点构成路径线段 AB，
     *        车辆位置 C 到 AB 的带符号垂直距离。
     *
     *  公式：CTE = ((B-A) × (C-A)) / |B-A|
     *        叉积正 → C 在 AB 左侧（CTE > 0）
     *        叉积负 → C 在 AB 右侧（CTE < 0）
     */
    if (s_nearest_idx + 1u < g_ins_path_count)
    {
        float ax = g_ins_path[s_nearest_idx].x;
        float ay = g_ins_path[s_nearest_idx].y;
        float bx = g_ins_path[s_nearest_idx + 1u].x;
        float by = g_ins_path[s_nearest_idx + 1u].y;

        float abx = bx - ax;
        float aby = by - ay;
        float acx = cx - ax;
        float acy = cy - ay;

        float ab_len = sqrtf(abx * abx + aby * aby);
        if (ab_len > 0.001f)
        {
            /* 二维叉积：AB × AC = abx*acy - aby*acx */
            s_cte = (abx * acy - aby * acx) / ab_len;
        }
    }

    return 0u;
}

uint8 ins_path_follow_is_done(void) { return s_follow_done; }

/* ================================================================
 *  调试信息读取
 * ================================================================ */
float  ins_path_get_cte(void)           { return s_cte; }
uint16 ins_path_get_nearest_idx(void)   { return s_nearest_idx; }
uint16 ins_path_get_lookahead_idx(void) { return s_lookahead_idx; }
