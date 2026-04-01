/*
 * top_go_and_turn.c
 *
 * 赛事功能：距离触发转弯（限速平滑）
 */

#include "zf_common_headfile.h"

// 转弯功能开关
#ifndef TTOP_TURN_ENABLE
#define TTOP_TURN_ENABLE              (1)
#endif

// 第一次触发距离（米）
#ifndef TTOP_TURN_START_M
#define TTOP_TURN_START_M             (3.0f)
#endif

// 每次转弯总角度（deg）
#ifndef TTOP_TURN_ANGLE_DEG
#define TTOP_TURN_ANGLE_DEG           (180.0f)
#endif

// 限速平滑：每秒最多变化多少 deg（越小越平滑，但转弯更慢）
#ifndef TTOP_TURN_YAW_RATE_LIMIT_DPS
#define TTOP_TURN_YAW_RATE_LIMIT_DPS  (50.0f)
#endif

typedef enum
{
    TTOP_TURN_STATE_IDLE = 0,
    TTOP_TURN_STATE_TURNING,
    TTOP_TURN_STATE_DONE,
} ttop_turn_state_t;

typedef struct
{
    ttop_turn_state_t state;
    float yaw_cmd_target_deg;
    float yaw_cmd_out_deg;
} ttop_turn_ctrl_t;

static ttop_turn_ctrl_t s_turn;

static inline uint8 ttop_yaw_delay_done_local(void)
{
    const led_bzw_show_status_t *st = led_bzw_show_status_get();
    return (st && st->yaw_delay_done) ? 1u : 0u;
}

static inline float ttop_limit_slew(float current, float target, float max_delta)
{
    float err = target - current;
    if (err > max_delta) return current + max_delta;
    if (err < -max_delta) return current - max_delta;
    return target;
}

void ttop_turn_init(void)
{
    s_turn.state = TTOP_TURN_STATE_IDLE;
    s_turn.yaw_cmd_target_deg = 0.0f;
    s_turn.yaw_cmd_out_deg = 0.0f;
}

//=======转弯任务，5ms调用一次=======//
void ttop_turn_task_5ms(void)
{
#if (TTOP_TURN_ENABLE)
    // 停机后不再转弯，且保持 yaw 指令为 0
    if (ttop_stop_is_active())
    {
        s_turn.yaw_cmd_target_deg = 0.0f;
        s_turn.yaw_cmd_out_deg = 0.0f;
        g_yaw_target_deg = 0.0f;
        return;
    }

    // yaw 延时置零未完成时，不做转弯规划（避免未稳定就转向）
    if (!ttop_yaw_delay_done_local())
    {
        s_turn.yaw_cmd_target_deg = 0.0f;
        s_turn.yaw_cmd_out_deg = 0.0f;
        g_yaw_target_deg = 0.0f;
        return;
    }

    int64 dist_cnt = encoder_get_demo5_count_total();

    // ========== 状态机：生成离散目标 yaw_cmd_target_deg（只转一次） ==========
    switch (s_turn.state)
    {
        default:
        case TTOP_TURN_STATE_IDLE:
            // IDLE 阶段保持 0，避免未触发前乱改 yaw 目标
            s_turn.yaw_cmd_target_deg = 0.0f;
            s_turn.yaw_cmd_out_deg = 0.0f;

            // 触发：达到启动距离后开始转弯（只触发一次）
            if (dist_cnt >= (int64)((float)ENCODER_COUNTS_PER_M * (float)TTOP_TURN_START_M))
            {
                s_turn.state = TTOP_TURN_STATE_TURNING;
                // 转弯开始时从 0 起步，避免残留
                s_turn.yaw_cmd_target_deg = 0.0f;
                s_turn.yaw_cmd_out_deg = 0.0f;
            }
            break;

        case TTOP_TURN_STATE_TURNING:
            // TURNING 直接追最终目标，由限速器产生平滑过程
            s_turn.yaw_cmd_target_deg = (float)TTOP_TURN_ANGLE_DEG;
            break;

        case TTOP_TURN_STATE_DONE:
            // 转弯完成后保持目标不再变化（也可以选择回 0，根据你赛道需求）
            s_turn.yaw_cmd_target_deg = (float)TTOP_TURN_ANGLE_DEG;
            break;
    }

    // ========== 限速平滑：TURNING/DONE 都保持输出到 yaw 目标 ==========
    {
        float max_delta = (float)TTOP_TURN_YAW_RATE_LIMIT_DPS * ((float)TTOP_TURN_DT_MS * 0.001f);
        s_turn.yaw_cmd_out_deg = ttop_limit_slew(s_turn.yaw_cmd_out_deg, s_turn.yaw_cmd_target_deg, max_delta);
        g_yaw_target_deg = s_turn.yaw_cmd_out_deg;

        // “到位”判定：输出与最终目标足够接近 -> DONE
        if (s_turn.state == TTOP_TURN_STATE_TURNING)
        {
            float err = s_turn.yaw_cmd_target_deg - s_turn.yaw_cmd_out_deg;
            if (err < 0) err = -err;
            if (err <= 0.5f)
            {
                s_turn.state = TTOP_TURN_STATE_DONE;
            }
        }
    }
#else
    // 未启用转弯：保持 0
    g_yaw_target_deg = 0.0f;
#endif
}
