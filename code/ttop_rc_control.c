/*
 * ttop_rc_control.c
 *
 *  Created on: 2026-04-05
 *      Author: hhttiuui
 *
 *  赛事功能：遥控器通道控制。
 *  - CH0 -> g_yaw_target_deg
 *  - CH1 -> motor_set_target_counts_per_5ms
 */

#include "zf_common_headfile.h"
#include "ttop_rc_control.h"

static ttop_rc_control_cfg_t s_rc_cfg;

static inline float ttop_rc_clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline float ttop_rc_apply_sensitivity(float x, float center, float sensitivity)
{
    if (sensitivity <= 0.0f) sensitivity = 1.0f;
    return center + (x - center) * sensitivity;
}

static float ttop_rc_map_axis(float in_raw, const ttop_rc_axis_cfg_t *cfg)
{
    float x = ttop_rc_apply_sensitivity(in_raw, cfg->in_center, cfg->sensitivity);
    x = ttop_rc_clampf(x, cfg->in_min, cfg->in_max);

    if (x <= cfg->in_center)
    {
        float den = (cfg->in_center - cfg->in_min);
        float t = (den > 0.0f) ? ((x - cfg->in_min) / den) : 0.0f;
        return cfg->out_min + t * (cfg->out_center - cfg->out_min);
    }
    else
    {
        float den = (cfg->in_max - cfg->in_center);
        float t = (den > 0.0f) ? ((x - cfg->in_center) / den) : 0.0f;
        return cfg->out_center + t * (cfg->out_max - cfg->out_center);
    }
}

static float ttop_rc_map_ch0_to_unit_cmd(float ch0)
{
    float x = ttop_rc_apply_sensitivity(ch0, s_rc_cfg.yaw.in_center, s_rc_cfg.yaw.sensitivity);
    x = ttop_rc_clampf(x, s_rc_cfg.yaw.in_min, s_rc_cfg.yaw.in_max);

    if (x <= s_rc_cfg.yaw.in_center)
    {
        float den = (s_rc_cfg.yaw.in_center - s_rc_cfg.yaw.in_min);
        return (den > 0.0f) ? ((x - s_rc_cfg.yaw.in_center) / den) : 0.0f; // [-1, 0]
    }
    else
    {
        float den = (s_rc_cfg.yaw.in_max - s_rc_cfg.yaw.in_center);
        return (den > 0.0f) ? ((x - s_rc_cfg.yaw.in_center) / den) : 0.0f; // [0, 1]
    }
}

static int16 ttop_rc_map_ch1_to_speed_counts(float ch1)
{
    float speed = ttop_rc_map_axis(ch1, &s_rc_cfg.speed);
    speed = ttop_rc_clampf(speed, s_rc_cfg.speed.out_min, s_rc_cfg.speed.out_max);

    return (int16)roundf(speed);
}

void ttop_rc_control_init(void)
{
    // 初始化默认参数：后续可通过 ttop_rc_control_set_config 覆盖。
    s_rc_cfg.yaw.in_min = 200.0f;
    s_rc_cfg.yaw.in_center = 800.0f;
    s_rc_cfg.yaw.in_max = 1400.0f;
    s_rc_cfg.yaw.out_min = -1.0f;
    s_rc_cfg.yaw.out_center = 0.0f;
    s_rc_cfg.yaw.out_max = 1.0f;
    s_rc_cfg.yaw.sensitivity = 0.2f;
    s_rc_cfg.yaw_rate_limit_deg_per_5ms = 8.0f;

    s_rc_cfg.speed.in_min = 800.0f;
    s_rc_cfg.speed.in_center = 800.0f;
    s_rc_cfg.speed.in_max = 1400.0f;
    s_rc_cfg.speed.out_min = 0.0f;
    s_rc_cfg.speed.out_center = 0.0f;
    s_rc_cfg.speed.out_max = 300.0f;
    s_rc_cfg.speed.sensitivity = 1.0f;
}

void ttop_rc_control_set_config(const ttop_rc_control_cfg_t *cfg)
{
    if (cfg == NULL) return;
    s_rc_cfg = *cfg;
}

const ttop_rc_control_cfg_t *ttop_rc_control_get_config(void)
{
    return &s_rc_cfg;
}

void ttop_rc_control_task_5ms(void)
{
    // 遥控器未就绪或失控时不更新控制量。
    if (uart_receiver.finsh_flag == 0u) return;
    if (uart_receiver.state == 0u)
    {
        uart_receiver.finsh_flag = 0u;
        return;
    }

    float yaw_unit_cmd = ttop_rc_map_ch0_to_unit_cmd((float)uart_receiver.channel[0]);
    float yaw_rate_lim = s_rc_cfg.yaw_rate_limit_deg_per_5ms;
    if (yaw_rate_lim < 0.0f) yaw_rate_lim = -yaw_rate_lim;

    float yaw_delta_deg = yaw_unit_cmd * yaw_rate_lim;
    g_yaw_target_deg -= yaw_delta_deg;

    motor_set_target_counts_per_5ms(ttop_rc_map_ch1_to_speed_counts(uart_receiver.channel[1]));

    uart_receiver.finsh_flag = 0u;
}
