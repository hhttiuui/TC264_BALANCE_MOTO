/*
 * ins_path_follow.h
 *
 *  Created on: 2026-03-24
 *      Author: hhttiuui
 *
 *  功能模块：路径录制与纯追踪(Pure Pursuit)路径跟踪
 *
 *  ===================== 模块概述 =====================
 *  本模块提供三大核心功能：
 *
 *  A. 路径录制（教学模式）
 *    手动/遥控小车行驶目标路线，模块自动按固定间隔（默认10cm）
 *    记录坐标点(x, y)。录制完成后可保存至Flash，下次上电直接加载使用。
 *
 *  B. 路径生成（数学公式）
 *    提供圆形、8字等常用路径的自动生成函数，
 *    用于快速测试航位推算 + 路径跟踪的完整链路。
 *
 *  C. 路径跟踪（Pure Pursuit 纯追踪算法）
 *    从已录制/已生成的路径点中，使用Pure Pursuit算法
 *    实时计算目标航向角，输出至g_yaw_target_deg，
 *    由现有PID控制（Yaw PID + Pitch PID + 舵机）实现自动转向。
 *
 *  ===================== 整体流程 =====================
 *
 *    编码器(速度) + IMU(航向角)
 *           ↓
 *    航位推算 ins_dr_update_5ms() → 输出实时位置(pos_x, pos_y)
 *           ↓
 *    Pure Pursuit 查找预瞄点 → 计算目标航向
 *           ↓
 *    g_yaw_target_deg → 现有Yaw PID → Pitch PID → 舵机控制
 *
 *  ===================== 使用方法 =====================
 *
 *  ■ 录制模式（function_kemu == 9）：
 *    1. 切换模式9 → 自动调用 ins_path_record_init()
 *    2. 每5ms调用 ins_path_record_task_5ms()
 *       此时Yaw PID设为被动模式，允许手动转向
 *    3. 录制完成：调用 ins_path_record_stop() → flash_storage_save_ins_path()
 *
 *  ■ 跟踪模式（function_kemu == 10）：
 *    1. 切换模式10 → 自动加载Flash路径 + 调用 ins_path_follow_init()
 *    2. 每5ms调用 ins_path_follow_task_5ms()
 *       自动输出g_yaw_target_deg，小车按录制路径自动行驶
 *    3. 到达终点时返回1，可执行停车/保持动作
 */

 #ifndef CODE_INS_PATH_FOLLOW_H_
 #define CODE_INS_PATH_FOLLOW_H_
 
 #include "zf_common_typedef.h"
 
 #ifdef __cplusplus
 extern "C" {
 #endif
 
 /* ================================================================
  *  路径点数据结构
  * ================================================================ */
 typedef struct {
     float x;    // X坐标（米）
     float y;    // Y坐标（米）
 } ins_waypoint_t;
 
 // 最大路径点数：500点 × 10cm间隔 = 最大50米行程
 #ifndef INS_PATH_MAX_POINTS
 #define INS_PATH_MAX_POINTS     (500u)
 #endif
 
 // 路径录制点间隔（米）
 #ifndef INS_RECORD_INTERVAL_M
 #define INS_RECORD_INTERVAL_M   (0.10f)
 #endif
 
 /* ================================================================
  *  录制滤波参数（滑动窗口直线判断 + 投影去抖）
  *
  *  原理：
  *    每次录制新点时，取最近N个点组成连续线段，计算各段航向角差。
  *    若所有角度差都很小 → 判定为直线 → 将新点投影到直线消除抖动。
  *    若角度差较大 → 判定为弯道 → 保留原始点不处理。
  *
  *  延时机制：
  *    进入直线模式阈值更严格（角度差小）
  *    退出直线模式阈值更宽松（角度差大）
  *    防止直线/弯道边界频繁切换
  * ================================================================ */
 
 // 滑动窗口大小：取最近4个点（3段线段，2个角度差）
 #ifndef INS_FILTER_WINDOW_SIZE
 #define INS_FILTER_WINDOW_SIZE        (4u)
 #endif
 
 // 进入直线模式航向角差阈值（度）：所有相邻段角度差 < 该值才进入
 #ifndef INS_FILTER_ANGLE_ENTER_DEG
 #define INS_FILTER_ANGLE_ENTER_DEG    (4.0f)
 #endif
 
 // 退出直线模式航向角差阈值（度）：任一角度差 > 该值则退出
 #ifndef INS_FILTER_ANGLE_EXIT_DEG
 #define INS_FILTER_ANGLE_EXIT_DEG     (7.0f)
 #endif
 
 // 横向偏差阈值（米）：超过此值不投影，保留原始点
 #ifndef INS_FILTER_LATERAL_TH_M
 #define INS_FILTER_LATERAL_TH_M       (0.05f)
 #endif
 
 // 最小线段长度（米）：过短则方向不可靠，跳过滤波
 #ifndef INS_FILTER_MIN_SEG_M
 #define INS_FILTER_MIN_SEG_M          (0.05f)
 #endif
 
 // 录制结束后离线平滑参数
 #ifndef INS_RECORD_SMOOTH_PASSES
 #define INS_RECORD_SMOOTH_PASSES      (3u)    // 平滑轮数
 #endif
 
 #ifndef INS_RECORD_SMOOTH_BLEND
 #define INS_RECORD_SMOOTH_BLEND       (0.50f) // 平滑混合系数
 #endif
 
 // 纯追踪预瞄距离（米）
 // 直线蛇形 → 调大
 // 弯道跟不上 → 调小
 #ifndef INS_PURSUIT_LOOKAHEAD_M
 #define INS_PURSUIT_LOOKAHEAD_M       (0.50f)
 #endif
 
 // 终点判定半径（米）
 #ifndef INS_PATH_END_RADIUS_M
 #define INS_PATH_END_RADIUS_M         (0.15f)
 #endif
 
 /* ================================================================
  *  全局路径点数组（外部可查看/修改）
  * ================================================================ */
 extern volatile ins_waypoint_t g_ins_path[INS_PATH_MAX_POINTS];
 extern volatile uint16         g_ins_path_count;  // 当前有效路径点数量
 
 /* ================================================================
  *  A. 路径录制功能接口
  * ================================================================ */
 
 /**
  * @brief  路径录制初始化
  * @note   清空路径点，重置航位推算，记录起点(0,0)
  */
 void ins_path_record_init(void);
 
 /**
  * @brief  路径录制5ms周期任务
  * @return 当前已录制的路径点数量
  * @note   定期更新位置，达到间隔自动记录新点
  */
 uint16 ins_path_record_task_5ms(void);
 
 /**
  * @brief  停止路径录制
  * @note   补充最后一个点，确保路径完整
  */
 void ins_path_record_stop(void);
 
 /**
  * @brief  获取录制状态
  * @retval 1=正在录制 0=未录制/已停止
  */
 uint8 ins_path_record_is_active(void);
 
 /* ================================================================
  *  B. 路径生成（数学公式，用于快速测试）
  * ================================================================ */
 
 /**
  * @brief  生成圆形路径
  * @param  radius_m: 圆半径，正数=左转，负数=右转
  * @param  num_points: 路径点总数
  */
 void ins_path_gen_circle(float radius_m, uint16 num_points);
 
 /**
  * @brief  生成8字路径
  * @param  radius_m: 单圆半径
  * @param  num_points_per_circle: 单个圆的路径点数
  */
 void ins_path_gen_eight(float radius_m, uint16 num_points_per_circle);
 
 /* ================================================================
  *  C. 纯追踪路径跟踪功能接口
  * ================================================================ */
 
 /**
  * @brief  路径跟踪初始化
  * @note   归零位置、重置跟踪状态
  */
 void ins_path_follow_init(void);
 
 /**
  * @brief  路径跟踪5ms周期任务
  * @retval 0=跟踪中 1=已到达终点
  * @note   自动计算目标航向并输出到g_yaw_target_deg
  */
 uint8 ins_path_follow_task_5ms(void);
 
 /**
  * @brief  获取跟踪完成状态
  * @retval 1=已到达终点 0=跟踪中
  */
 uint8 ins_path_follow_is_done(void);
 
 /* ================================================================
  *  调试信息接口（只读，用于屏幕显示/调试）
  * ================================================================ */
 
 /**
  * @brief  获取横向偏差CTE
  * @return 正数=小车在路径左侧，负数=在右侧
  */
 float ins_path_get_cte(void);
 
 /**
  * @brief  获取最近路径点索引
  */
 uint16 ins_path_get_nearest_idx(void);
 
 /**
  * @brief  获取当前预瞄点索引
  */
 uint16 ins_path_get_lookahead_idx(void);
 
 #ifdef __cplusplus
 }
 #endif
 
 #endif