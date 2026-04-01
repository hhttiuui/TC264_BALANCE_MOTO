// /*
//  * top_8_turn.c
//  *
//  *  Created on: 2026-01-26
//  *      Author: hhttiuui
//  *
//  *  赛事功能：8字绕行（分段 左-中-右-中，限速平滑）
//  *  - 输出：写 g_yaw_target_deg 作为舵机 PID 目标
//  *  - 触发：达到指定里程后开始
//  *  - 到位判定：按“规划输出到位”（out 接近 target）+ 连续保持 ticks
//  */

// #include "zf_common_headfile.h"

// extern float show_yaw_deg;

// // 前置声明：避免依赖 Ttop_funtion.c 的 static 函数
// static inline uint8 ttop_yaw_delay_done_local(void);

// #ifndef TTOP_EIGHT_ENABLE
// #define TTOP_EIGHT_ENABLE              (1)
// #endif

// // 8字启动触发距离（米）：达到该距离后开始 8字
// #ifndef TTOP_EIGHT_START_M
// #define TTOP_EIGHT_START_M             (0.5f)
// #endif

// // 8字幅度（deg）：左右分别为 +A/-A
// #ifndef TTOP_EIGHT_ANGLE_DEG
// #define TTOP_EIGHT_ANGLE_DEG           (40.0f)
// #endif

// // 8字限速平滑（deg/s）
// #ifndef TTOP_EIGHT_YAW_RATE_LIMIT_DPS
// #define TTOP_EIGHT_YAW_RATE_LIMIT_DPS  (40.0f)
// #endif

// // 8字“基准切换”（0组->180组）专用限速平滑（deg/s）
// #ifndef TTOP_EIGHT_BASE_SWITCH_YAW_RATE_LIMIT_DPS
// #define TTOP_EIGHT_BASE_SWITCH_YAW_RATE_LIMIT_DPS  (80.0f)
// #endif

// // 一个完整“8字”= 左(+A)->中(0)->右(-A)->中(0)
// #ifndef TTOP_EIGHT_REPEAT_TIMES
// #define TTOP_EIGHT_REPEAT_TIMES        (1u)
// #endif

// // 每完成一个“8字”后，基准角额外偏移多少度（deg）
// #ifndef TTOP_EIGHT_BASE_STEP_DEG
// #define TTOP_EIGHT_BASE_STEP_DEG       (180.0f)
// #endif

// // 到位阈值（deg）：按“规划输出到位”判定（比较 out 与 target）
// #ifndef TTOP_EIGHT_INPOS_EPS_DEG
// #define TTOP_EIGHT_INPOS_EPS_DEG       (0.5f)
// #endif

// // 连续到位确认次数：防止抖动误判（5ms一次，10次=50ms）
// #ifndef TTOP_EIGHT_HOLD_TICKS
// #define TTOP_EIGHT_HOLD_TICKS          (8u)
// #endif

// typedef enum
// {
//     TTOP_EIGHT_STATE_IDLE = 0,
//     TTOP_EIGHT_STATE_TO_LEFT,
//     TTOP_EIGHT_STATE_TO_CENTER_1,
//     TTOP_EIGHT_STATE_TO_RIGHT,
//     TTOP_EIGHT_STATE_TO_CENTER_2,
//     TTOP_EIGHT_STATE_BASE_SWITCH,   // base0 -> base0+180 专用大转角切换段（只此段用专用限速）
//     TTOP_EIGHT_STATE_DONE,
// } ttop_eight_state_t;

// typedef struct
// {
//     ttop_eight_state_t state;
//     uint16 repeat_done;
//     uint16 inpos_ticks;
//     float start_trigger_m;
//     float base_yaw_deg;
//     uint8 base_phase;            // 0: base0(0组)  1: base0+180(180组)
//     float base0_yaw_deg;
//     float yaw_cmd_target_deg;
//     float yaw_cmd_out_deg;
// } ttop_eight_ctrl_t;

// static ttop_eight_ctrl_t s_eight;

// static inline float ttop_distance_m_get(void)
// {
//     int64 cnt = encoder_get_demo5_count_total();
//     if (cnt < 0) cnt = -cnt;
//     return (float)((float)cnt / (float)ENCODER_COUNTS_PER_M);
// }

// static inline uint8 ttop_inpos(float target_deg, float out_deg)
// {
//     float err = target_deg - out_deg;
//     if (err < 0) err = -err;
//     return (err <= (float)TTOP_EIGHT_INPOS_EPS_DEG) ? 1u : 0u;
// }

// static inline float ttop_limit_slew(float current, float target, float max_delta)
// {
//     float err = target - current;
//     if (err > max_delta) return current + max_delta;
//     if (err < -max_delta) return current - max_delta;
//     return target;
// }

// void ttop_eight_init(void)
// {
//     s_eight.state = TTOP_EIGHT_STATE_IDLE;
//     s_eight.repeat_done = 0;
//     s_eight.inpos_ticks = 0;
//     s_eight.start_trigger_m = (float)TTOP_EIGHT_START_M;
//     s_eight.base_yaw_deg = 0.0f;
//     s_eight.base_phase = 0u;
//     s_eight.base0_yaw_deg = 0.0f;
//     s_eight.yaw_cmd_target_deg = 0.0f;
//     s_eight.yaw_cmd_out_deg = 0.0f;
// }


// //=======8字绕行任务，5ms调用一次=======//
// void ttop_eight_task_5ms(void)
// {
// #if (TTOP_EIGHT_ENABLE)
//     // 停机后不再规划，且保持 yaw 指令为 0
//     if (ttop_stop_is_active())
//     {
//         s_eight.yaw_cmd_target_deg = 0.0f;
//         s_eight.yaw_cmd_out_deg = 0.0f;
//         g_yaw_target_deg = 0.0f;
//         return;
//     }

//     // yaw 延时置零未完成时，不做 8字规划
//     if (!ttop_yaw_delay_done_local())
//     {
//         s_eight.yaw_cmd_target_deg = 0.0f;
//         s_eight.yaw_cmd_out_deg = 0.0f;
//         g_yaw_target_deg = 0.0f;
//         return;
//     }

//     float dist_m = ttop_distance_m_get();

//     float rate_limit_dps = (float)TTOP_EIGHT_YAW_RATE_LIMIT_DPS;
//     if (s_eight.state == TTOP_EIGHT_STATE_BASE_SWITCH)
//     {
//         // 仅在“base0->base0+180”这段大转角切换用专用限速
//         rate_limit_dps = (float)TTOP_EIGHT_BASE_SWITCH_YAW_RATE_LIMIT_DPS;
//     }

//     float max_delta = rate_limit_dps * ((float)TTOP_TURN_DT_MS * 0.001f);
//     const float A = (float)TTOP_EIGHT_ANGLE_DEG;

//     // ========== 状态机：生成离散目标 yaw_cmd_target_deg ==========
//     switch (s_eight.state)
//     {
//         default:
//         case TTOP_EIGHT_STATE_IDLE:
//             s_eight.repeat_done = 0;
//             s_eight.inpos_ticks = 0;
//             s_eight.yaw_cmd_target_deg = 0.0f;
//             s_eight.yaw_cmd_out_deg = 0.0f;
//             s_eight.base_phase = 0u;

//             // 触发：达到启动距离后开始 8字
//             if (dist_m >= (float)TTOP_EIGHT_START_M)
//             {
//                 s_eight.state = TTOP_EIGHT_STATE_TO_LEFT;
//                 // base0：以当前朝向作为第一组（0组）的中心线
//                 s_eight.base0_yaw_deg = show_yaw_deg;
//                 s_eight.base_yaw_deg = s_eight.base0_yaw_deg;
//                 // 从当前角开始输出，避免突变
//                 s_eight.yaw_cmd_out_deg = s_eight.base_yaw_deg;
//                 // 第一段目标：base + A
//                 s_eight.yaw_cmd_target_deg = s_eight.base_yaw_deg + A;
//             }
//             break;

//         case TTOP_EIGHT_STATE_TO_LEFT:
//             s_eight.yaw_cmd_target_deg = s_eight.base_yaw_deg + A;
//             if (ttop_inpos(s_eight.yaw_cmd_target_deg, s_eight.yaw_cmd_out_deg))
//             {
//                 if (++s_eight.inpos_ticks >= (uint16)TTOP_EIGHT_HOLD_TICKS)
//                 {
//                     s_eight.inpos_ticks = 0;
//                     s_eight.state = TTOP_EIGHT_STATE_TO_CENTER_1;
//                 }
//             }
//             else s_eight.inpos_ticks = 0;
//             break;

//         case TTOP_EIGHT_STATE_TO_CENTER_1:
//             s_eight.yaw_cmd_target_deg = s_eight.base_yaw_deg;
//             if (ttop_inpos(s_eight.yaw_cmd_target_deg, s_eight.yaw_cmd_out_deg))
//             {
//                 if (++s_eight.inpos_ticks >= (uint16)TTOP_EIGHT_HOLD_TICKS)
//                 {
//                     s_eight.inpos_ticks = 0;
//                     s_eight.state = TTOP_EIGHT_STATE_TO_RIGHT;
//                 }
//             }
//             else s_eight.inpos_ticks = 0;
//             break;

//         case TTOP_EIGHT_STATE_TO_RIGHT:
//             s_eight.yaw_cmd_target_deg = s_eight.base_yaw_deg - A;
//             if (ttop_inpos(s_eight.yaw_cmd_target_deg, s_eight.yaw_cmd_out_deg))
//             {
//                 if (++s_eight.inpos_ticks >= (uint16)TTOP_EIGHT_HOLD_TICKS)
//                 {
//                     s_eight.inpos_ticks = 0;
//                     s_eight.state = TTOP_EIGHT_STATE_TO_CENTER_2;
//                 }
//             }
//             else s_eight.inpos_ticks = 0;
//             break;

//         case TTOP_EIGHT_STATE_TO_CENTER_2:
//             s_eight.yaw_cmd_target_deg = s_eight.base_yaw_deg;
//             if (ttop_inpos(s_eight.yaw_cmd_target_deg, s_eight.yaw_cmd_out_deg))
//             {
//                 if (++s_eight.inpos_ticks >= (uint16)TTOP_EIGHT_HOLD_TICKS)
//                 {
//                     s_eight.inpos_ticks = 0;
//                     // 一个“小8字”（base固定的一轮：+A->0->-A->0）完成
//                     if (s_eight.base_phase == 0u)
//                     {
//                         // 先做 base0 -> base0+180 的大转角切换（只这个状态用专用限速）
//                         s_eight.base_phase = 1u;
//                         s_eight.base_yaw_deg = s_eight.base0_yaw_deg + (float)TTOP_EIGHT_BASE_STEP_DEG;
//                         s_eight.state = TTOP_EIGHT_STATE_BASE_SWITCH;
//                     }
//                     else
//                     {
//                         // 第二组也完成：算完成一个“完整8字”
//                         s_eight.repeat_done++;
//                         if (s_eight.repeat_done >= (uint16)TTOP_EIGHT_REPEAT_TIMES)
//                         {
//                             s_eight.state = TTOP_EIGHT_STATE_DONE;
//                         }
//                         else
//                         {
//                             // 下一次完整8字：重新从 base0 抓取（或延用当前策略，这里延用“重新抓取”更稳）
//                             s_eight.base_phase = 0u;
//                             s_eight.base0_yaw_deg = show_yaw_deg;
//                             s_eight.base_yaw_deg = s_eight.base0_yaw_deg;
//                             s_eight.state = TTOP_EIGHT_STATE_TO_LEFT;
//                         }
//                     }
//                 }
//             }
//             else s_eight.inpos_ticks = 0;
//             break;

//         case TTOP_EIGHT_STATE_BASE_SWITCH:
//             // base0 -> base0+180：此段只追“中心线(base)”，不带 +/-A。
//             // 目的：让“0->180”用专用限速；而第二组 180->210->180->150->180 全程用常规限速。
//             s_eight.yaw_cmd_target_deg = s_eight.base_yaw_deg;
//             if (ttop_inpos(s_eight.yaw_cmd_target_deg, s_eight.yaw_cmd_out_deg))
//             {
//                 if (++s_eight.inpos_ticks >= (uint16)TTOP_EIGHT_HOLD_TICKS)
//                 {
//                     s_eight.inpos_ticks = 0;
//                     // 切换完成：进入第二组的正常 8 字流程（先去 +A）
//                     s_eight.state = TTOP_EIGHT_STATE_TO_LEFT;
//                 }
//             }
//             else s_eight.inpos_ticks = 0;
//             break;

//         case TTOP_EIGHT_STATE_DONE:
//             // 完整 8 字后保持在 180 组的中心线直行（base0 + 180）
//             s_eight.yaw_cmd_target_deg = s_eight.base0_yaw_deg + (float)TTOP_EIGHT_BASE_STEP_DEG;
//             break;
//     }

//     // ========== 限速平滑：将目标转换为平滑输出，并写入 yaw PID 目标 ==========
//     s_eight.yaw_cmd_out_deg = ttop_limit_slew(s_eight.yaw_cmd_out_deg, s_eight.yaw_cmd_target_deg, max_delta);
//     g_yaw_target_deg = s_eight.yaw_cmd_out_deg;
// #else
//     // 未启用：不做任何处理
// #endif
// }

// static inline uint8 ttop_yaw_delay_done_local(void)
// {
//     const led_bzw_show_status_t *st = led_bzw_show_status_get();
//     return (st && st->yaw_delay_done) ? 1u : 0u;
// }
