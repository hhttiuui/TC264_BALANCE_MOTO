/*
 * ins_path_follow.c
 *
 *  Created on: 2026-03-24
 *      Author: hhttiuui
 *
 *  功能：路径录制 + 纯追踪(Pure Pursuit)路径跟踪算法
 *
 *  ===================== Pure Pursuit 算法简介 =====================
 *  Pure Pursuit（纯追踪）是经典的路径跟踪算法：
 *    1. 在目标路径上找到距离车辆最近的点（nearest）
 *    2. 从最近点沿路径向前查找一个"预瞄点"，该点与车辆距离 ≥ 预瞄距离 L_d
 *    3. 计算车辆当前位置到预瞄点的目标航向角
 *    4. 将目标航向角输出给PID控制器执行转向
 *
 *  预瞄距离 L_d 作用：
 *    L_d 越大 → 转向越提前、越平滑，但跟踪精度降低
 *    L_d 越小 → 跟踪越精准，但容易产生抖动（尤其弯道处）
 *    经验值：车速(m/s) × (0.5~1.5)，低速小车推荐 0.15~0.50m
 */

 #include "zf_common_headfile.h"
 #include "ins_path_follow.h"
 #include "ins_dead_reckoning.h"
 
 /* PI 常量 */
 #ifndef INS_PF_PI
 #define INS_PF_PI  (3.14159265358979f)
 #endif
 
 /* ================================================================
  *  转向角速度限制参数（Slew Rate Limiter）
  *
  *  作用：限制目标航向角每5ms最大变化量，
  *        防止纯追踪算法在弯道处目标航向突变，
  *        导致PID输出剧烈、小车失控翻车
  *
  *  INS_YAW_SLEW_RATE_DPS：最大允许的目标航向变化速率（deg/s）
  *    - 数值越大：转向越快（响应灵敏），但越容易翻车
  *    - 数值越小：转向越慢（平滑稳定），但可能跟踪不及时
  *    - 建议从 60~120 deg/s 开始调试
  *    - 参考：八字循迹默认 60 deg/s
  *
  *  每5ms最大步进值 = INS_YAW_SLEW_RATE_DPS × 0.005
  *    60 deg/s  → 每5ms最多变化 0.30°
  *    90 deg/s  → 每5ms最多变化 0.45°
  *    120 deg/s → 每5ms最多变化 0.60°
  * ================================================================ */
 #ifndef INS_YAW_SLEW_RATE_DPS
 #define INS_YAW_SLEW_RATE_DPS   (45.0f)
 #endif
 
 /* 每5ms调用一次时的最大转向步进（deg） */
 #define INS_YAW_SLEW_MAX_STEP   ((float)INS_YAW_SLEW_RATE_DPS * 0.005f)
 
 /* ================================================================
  *  全局路径点数组（外部调用使用）
  * ================================================================ */
 volatile ins_waypoint_t g_ins_path[INS_PATH_MAX_POINTS];
 volatile uint16         g_ins_path_count = 0u;
 
 /* ================================================================
  *  内部状态变量：路径录制
  * ================================================================ */
 static uint8  s_record_active      = 0u;     // 1=正在录制 0=未录制
 static float  s_record_last_odo_m  = 0.0f;   // 上一次录制点的里程计数值
 static uint8  s_straight_mode      = 0u;     // 直线模式标志：1=当前检测为直线
 
 /* ================================================================
  *  内部状态变量：路径跟踪
  * ================================================================ */
 static uint8  s_follow_done     = 0u;   // 1=已到达终点
 static uint16 s_nearest_idx     = 0u;   // 最近路径点索引
 static uint16 s_lookahead_idx   = 0u;   // 预瞄路径点索引
 static float  s_cte             = 0.0f; // 横向偏差（米）
 static float  s_yaw_slew_out    = 0.0f; // 角速度限制器当前输出（deg）
 static uint8  s_yaw_slew_inited = 0u;   // 角速度限制器初始化标志
 
 /* ================================================================
  *  辅助函数声明
  * ================================================================ */
 
 // 计算两点距离的平方（避免开方，用于比较大小更快）
 static inline float ins_dist2(float x1, float y1, float x2, float y2)
 {
     float dx = x2 - x1;
     float dy = y2 - y1;
     return dx * dx + dy * dy;
 }
 
 // 计算两点实际距离
 static inline float ins_distf(float x1, float y1, float x2, float y2)
 {
     return sqrtf(ins_dist2(x1, y1, x2, y2));
 }
 
 // 角度归一化到 [-180, +180)
 static inline float ins_wrap_deg(float deg)
 {
     while (deg >  180.0f) deg -= 360.0f;
     while (deg < -180.0f) deg += 360.0f;
     return deg;
 }
 
 /* ================================================================
  *  航向角角速度限制器（Slew Rate Limiter）
  *
  *  对纯追踪计算出的原始目标航向进行限速平滑：
  *  每5ms最大变化 INS_YAW_SLEW_MAX_STEP 度，
  *  正确处理-180/+180度跳变问题
  * ================================================================ */
 static float ins_yaw_slew_limit(float raw_target_deg)
 {
     // 首次调用：直接跟随，不做限制
     if (!s_yaw_slew_inited)
     {
         s_yaw_slew_inited = 1u;
         s_yaw_slew_out = raw_target_deg;
         return s_yaw_slew_out;
     }
 
     // 计算角度误差（处理-180/+180跳变）
     float err = ins_wrap_deg(raw_target_deg - s_yaw_slew_out);
 
     // 限制误差范围
     if (err > INS_YAW_SLEW_MAX_STEP)
         err = INS_YAW_SLEW_MAX_STEP;
     else if (err < -INS_YAW_SLEW_MAX_STEP)
         err = -INS_YAW_SLEW_MAX_STEP;
 
     s_yaw_slew_out = ins_wrap_deg(s_yaw_slew_out + err);
     return s_yaw_slew_out;
 }
 
 /* ================================================================
  *  录制结束后路径平滑处理
  *
  *  作用：
  *    1) 让录制轨迹整体更顺滑，消除手动推行抖动
  *    2) 让拐角由"尖角"变为圆滑曲线
  *
  *  方法：
  *    对内部点执行多轮三点平滑：
  *      avg = 0.25*前一点 + 0.5*当前点 + 0.25*后一点
  *      当前点 = 当前点 + 平滑系数*(平均值-当前点)
  *
  *  特点：
  *    - 首尾路径点保持不变
  *    - 平滑轮数越多，轨迹越圆滑
  *    - 平滑系数越大，平滑效果越强
  * ================================================================ */
 static void ins_record_smooth_path(void)
 {
     if (g_ins_path_count < 3u) return;
 
     static float sx[INS_PATH_MAX_POINTS];
     static float sy[INS_PATH_MAX_POINTS];
 
     uint16 count = g_ins_path_count;
     uint16 pass;
 
     // 执行多轮平滑
     for (pass = 0u; pass < (uint16)INS_RECORD_SMOOTH_PASSES; pass++)
     {
         uint16 i;
 
         // 首尾点保持不变
         sx[0] = g_ins_path[0].x;
         sy[0] = g_ins_path[0].y;
         sx[count - 1u] = g_ins_path[count - 1u].x;
         sy[count - 1u] = g_ins_path[count - 1u].y;
 
         // 中间点平滑计算
         for (i = 1u; i + 1u < count; i++)
         {
             float prev_x = g_ins_path[i - 1u].x;
             float prev_y = g_ins_path[i - 1u].y;
             float curr_x = g_ins_path[i].x;
             float curr_y = g_ins_path[i].y;
             float next_x = g_ins_path[i + 1u].x;
             float next_y = g_ins_path[i + 1u].y;
 
             float avg_x = 0.25f * prev_x + 0.50f * curr_x + 0.25f * next_x;
             float avg_y = 0.25f * prev_y + 0.50f * curr_y + 0.25f * next_y;
 
             sx[i] = curr_x + (avg_x - curr_x) * (float)INS_RECORD_SMOOTH_BLEND;
             sy[i] = curr_y + (avg_y - curr_y) * (float)INS_RECORD_SMOOTH_BLEND;
         }
 
         // 将平滑后的数据写回路径数组
         for (i = 1u; i + 1u < count; i++)
         {
             g_ins_path[i].x = sx[i];
             g_ins_path[i].y = sy[i];
         }
     }
 }
 
 /* ================================================================
  *  录制滤波：滑动窗口直线判断 + 直线投影滤波
  *
  *  算法流程：
  *    1) 取窗口内最近N个点的连续线段
  *    2) 计算每段航向角，再计算相邻航向角差值
  *    3) 延时判断当前是否为"直线模式"
  *       - 未进入直线模式：所有角度差 < 进入阈值 → 进入直线模式
  *       - 已在直线模式：任一角度差 > 退出阈值 → 退出直线模式
  *    4) 直线模式下：将新点投影到参考直线上，减少抖动
  *    5) 非直线模式：不处理，保留弯道原始点
  * ================================================================ */
 static void ins_record_filter_point(float *px, float *py)
 {
     if ((float *)0 == px || (float *)0 == py) return;
 
     // 窗口内需要足够的点才能滤波
     uint16 win = (uint16)INS_FILTER_WINDOW_SIZE;
     if (g_ins_path_count < (uint16)(win - 1u)) return;
 
     // 收集窗口内各线段的航向角
     uint16 n_segs = (uint16)(win - 1u);
     float  angles[3];    // 最大3段（窗口=4）
     uint8  seg_ok = 1u;  // 线段长度是否有效
 
     uint16 i;
     for (i = 0u; i < n_segs; i++)
     {
         float ax, ay, bx, by;
 
         if (i < (uint16)(n_segs - 1u))
         {
             // 已有路径点之间的线段
             uint16 idx_a = (uint16)(g_ins_path_count - (win - 1u) + i);
             uint16 idx_b = (uint16)(idx_a + 1u);
             ax = g_ins_path[idx_a].x;
             ay = g_ins_path[idx_a].y;
             bx = g_ins_path[idx_b].x;
             by = g_ins_path[idx_b].y;
         }
         else
         {
             // 最后一段：上一个点 → 新点
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
 
     // 线段过短，角度不可靠，跳过滤波
     if (!seg_ok)
     {
         s_straight_mode = 0u;
         return;
     }
 
     // 计算相邻线段最大角度差
     float max_angle_diff = 0.0f;
     for (i = 0u; i + 1u < n_segs; i++)
     {
         float diff = fabsf(ins_wrap_deg(angles[i + 1u] - angles[i]));
         if (diff > max_angle_diff) max_angle_diff = diff;
     }
 
     // 延时判断：进入/退出直线模式
     if (s_straight_mode == 0u)
     {
         // 未在直线模式，使用进入阈值判断
         if (max_angle_diff < (float)INS_FILTER_ANGLE_ENTER_DEG)
         {
             s_straight_mode = 1u;
         }
     }
     else
     {
         // 已在直线模式，使用退出阈值判断
         if (max_angle_diff > (float)INS_FILTER_ANGLE_EXIT_DEG)
         {
             s_straight_mode = 0u;
         }
     }
 
     // 非直线模式（弯道），不做处理
     if (s_straight_mode == 0u) return;
 
     // 直线模式：将新点投影到参考直线上
     {
         uint16 ref_idx = (uint16)(g_ins_path_count - 1u);
         float rx = g_ins_path[ref_idx].x;
         float ry = g_ins_path[ref_idx].y;
 
         // 参考方向：使用上一段有效直线方向
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
 
         if (seg_len < 0.001f) return;
 
         // 计算新点相对参考点的横向偏差
         float vx = (*px) - rx;
         float vy = (*py) - ry;
         float lateral = fabsf((vx * dy - vy * dx) / seg_len);
 
         // 横向偏差过大，判定为异常点，不投影
         if (lateral > (float)INS_FILTER_LATERAL_TH_M) return;
 
         // 沿参考方向投影
         float t = (vx * dx + vy * dy) / (seg_len * seg_len);
         if (t < 0.0f) t = 0.0f;  // 不允许向后投影
 
         *px = rx + t * dx;
         *py = ry + t * dy;
     }
 }
 
 /* ================================================================
  *  A. 路径录制功能
  * ================================================================ */
 
 /**
  * @brief  路径录制初始化
  * @note   清空路径点，重置航位推算，记录起点(0,0)
  */
 void ins_path_record_init(void)
 {
     // 清空路径点数组
     g_ins_path_count = 0u;
     s_record_active = 1u;
     s_record_last_odo_m = 0.0f;
     s_straight_mode = 0u;
 
     // 重置航位推算：位置归零
     ins_dr_reset();
 
     // 记录起始点 (0, 0)
     if (g_ins_path_count < (uint16)INS_PATH_MAX_POINTS)
     {
         g_ins_path[g_ins_path_count].x = 0.0f;
         g_ins_path[g_ins_path_count].y = 0.0f;
         g_ins_path_count++;
     }
 }
 
 /**
  * @brief  路径录制任务（5ms调用一次）
  * @return 当前录制的路径点数量
  */
 uint16 ins_path_record_task_5ms(void)
 {
     if (!s_record_active) return g_ins_path_count;
 
     // 路径点数组满，自动停止录制
     if (g_ins_path_count >= (uint16)INS_PATH_MAX_POINTS)
     {
         s_record_active = 0u;
         return g_ins_path_count;
     }
 
     // 更新航位推算（位置+里程计）
     ins_dr_update_5ms();
 
     // 检测行驶距离，达到间隔则录制新点
     float odo = ins_dr_get_odometer_m();
     if ((odo - s_record_last_odo_m) >= (float)INS_RECORD_INTERVAL_M)
     {
         float rec_x = ins_dr_get_pos_x();
         float rec_y = ins_dr_get_pos_y();
 
         // 路径点滤波
         ins_record_filter_point(&rec_x, &rec_y);
 
         g_ins_path[g_ins_path_count].x = rec_x;
         g_ins_path[g_ins_path_count].y = rec_y;
         g_ins_path_count++;
         s_record_last_odo_m = odo;
     }
 
     return g_ins_path_count;
 }
 
 /**
  * @brief  停止路径录制
  * @note   补充最后一个路径点，避免尾部丢失
  */
 void ins_path_record_stop(void)
 {
     if (!s_record_active) return;
     s_record_active = 0u;
 
     // 补充记录最后一个点，避免尾部丢失
     if (g_ins_path_count > 0u && g_ins_path_count < (uint16)INS_PATH_MAX_POINTS)
     {
         float cx = ins_dr_get_pos_x();
         float cy = ins_dr_get_pos_y();
 
         // 与上一个点距离过近（<1cm）则不重复记录
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
 
 /**
  * @brief  获取录制状态
  * @retval 1=正在录制 0=未录制
  */
 uint8 ins_path_record_is_active(void) { return s_record_active; }
 
 /* ================================================================
  *  B. 数学路径生成（圆形/8字）
  * ================================================================ */
 
 /**
  * @brief  生成圆形路径
  * @param  radius_m: 圆半径（正数左转，负数右转）
  * @param  num_points: 路径点数量
  */
 void ins_path_gen_circle(float radius_m, uint16 num_points)
 {
     if (num_points > (uint16)INS_PATH_MAX_POINTS) num_points = (uint16)INS_PATH_MAX_POINTS;
     if (num_points < 4u) num_points = 4u;
 
     float R = fabsf(radius_m);
     if (R < 0.01f) R = 0.01f;
 
     // 方向：正数=左转（逆时针），负数=右转（顺时针）
     float dir = (radius_m >= 0.0f) ? 1.0f : -1.0f;
 
     // 圆形路径，起点(0,0)，初始朝向+X轴
     g_ins_path_count = 0u;
     for (uint16 i = 0u; i < num_points; i++)
     {
         float t = (float)i * (2.0f * INS_PF_PI / (float)num_points);
         g_ins_path[i].x = R * sinf(t);
         g_ins_path[i].y = dir * R * (1.0f - cosf(t));
     }
     g_ins_path_count = num_points;
 }
 
 /**
  * @brief  生成8字路径
  * @param  radius_m: 圆半径
  * @param  num_points_per_circle: 单个圆的路径点数量
  */
 void ins_path_gen_eight(float radius_m, uint16 num_points_per_circle)
 {
     float R = fabsf(radius_m);
     if (R < 0.01f) R = 0.01f;
 
     uint16 total = num_points_per_circle * 2u;
     if (total > (uint16)INS_PATH_MAX_POINTS) total = (uint16)INS_PATH_MAX_POINTS;
     uint16 npc = total / 2u;
 
     g_ins_path_count = 0u;
 
     // 8字 = 左转一圈 + 右转一圈
     // 左圆（逆时针）
     for (uint16 i = 0u; i < npc; i++)
     {
         float t = (float)i * (2.0f * INS_PF_PI / (float)npc);
         g_ins_path[i].x = R * sinf(t);
         g_ins_path[i].y = R * (1.0f - cosf(t));
     }
 
     // 右圆（顺时针）
     for (uint16 i = 0u; i < npc; i++)
     {
         float t = (float)i * (2.0f * INS_PF_PI / (float)npc);
         g_ins_path[npc + i].x = R * sinf(t);
         g_ins_path[npc + i].y = -R * (1.0f - cosf(t));
     }
 
     g_ins_path_count = total;
 }
 
 /* ================================================================
  *  C. 纯追踪(Pure Pursuit)路径跟踪
  * ================================================================ */
 
 /**
  * @brief  路径跟踪初始化
  * @note   重置航位推算和跟踪状态
  */
 void ins_path_follow_init(void)
 {
     // 航位推算归零：跟踪起点=坐标原点
     ins_dr_reset();
 
     // 跟踪状态归零
     s_follow_done   = 0u;
     s_nearest_idx   = 0u;
     s_lookahead_idx = 0u;
     s_cte           = 0.0f;
 
     // 角速度限制器归零
     s_yaw_slew_inited = 0u;
     s_yaw_slew_out    = 0.0f;
 }
 
 /**
  * @brief  路径跟踪任务（5ms调用一次）
  * @retval 0=跟踪中 1=已完成
  */
 uint8 ins_path_follow_task_5ms(void)
 {
     // 路径点不足，无法跟踪
     if (g_ins_path_count < 2u)
     {
         s_follow_done = 1u;
         return 1u;
     }
 
     // 已到达终点，保持最后输出
     if (s_follow_done) return 1u;
 
     // 1. 更新航位推算，获取当前位置
     ins_dr_update_5ms();
     float cx = ins_dr_get_pos_x();
     float cy = ins_dr_get_pos_y();
 
     // 2. 查找最近路径点（从上一个最近点附近搜索）
     {
         uint16 best_idx = s_nearest_idx;
         float  best_d2  = ins_dist2(cx, cy,
                             g_ins_path[best_idx].x,
                             g_ins_path[best_idx].y);
 
         // 搜索窗口：向前多搜索，车辆只向前行驶
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
 
     // 3. 查找预瞄点：从最近点向前找，距离≥预瞄距离
     {
         float La = (float)INS_PURSUIT_LOOKAHEAD_M;
         uint16 la_idx = s_nearest_idx;
 
         for (uint16 i = s_nearest_idx; i < g_ins_path_count; i++)
         {
             float d = ins_distf(cx, cy, g_ins_path[i].x, g_ins_path[i].y);
             la_idx = i;
             if (d >= La)
             {
                 break;
             }
         }
         s_lookahead_idx = la_idx;
     }
 
     // 4. 终点判断：预瞄点为最后一点且距离小于终点半径
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
 
     // 5. 计算目标航向角 → 限速平滑 → 输出到g_yaw_target_deg
     {
         float goal_x = g_ins_path[s_lookahead_idx].x;
         float goal_y = g_ins_path[s_lookahead_idx].y;
 
         // 计算目标航向角
         float target_rad = atan2f(goal_y - cy, goal_x - cx);
         float target_deg = target_rad * (180.0f / INS_PF_PI);
 
         // 角速度限制：防止转向突变导致翻车
         g_yaw_target_deg = ins_yaw_slew_limit(target_deg);
     }
 
     // 6. 计算横向偏差CTE（用于调试/显示）
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
             // 二维叉乘计算带符号横向偏差
             s_cte = (abx * acy - aby * acx) / ab_len;
         }
     }
 
     return 0u;
 }
 
 /**
  * @brief  获取跟踪完成状态
  * @retval 1=已到达终点 0=跟踪中
  */
 uint8 ins_path_follow_is_done(void) { return s_follow_done; }
 
 /* ================================================================
  *  调试信息获取接口
  * ================================================================ */
 // 获取横向偏差CTE
 float  ins_path_get_cte(void)           { return s_cte; }
 // 获取最近路径点索引
 uint16 ins_path_get_nearest_idx(void)   { return s_nearest_idx; }
 // 获取预瞄点索引
 uint16 ins_path_get_lookahead_idx(void) { return s_lookahead_idx; }