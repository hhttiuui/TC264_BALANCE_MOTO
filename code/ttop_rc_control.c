/*
 * ttop_rc_control.c
 *
 *  Created on: 2026-04-05
 *      Author: hhttiuui
 *
 *  璧涗簨鍔熻兘锛氶仴鎺у櫒閫氶亾鎺у埗銆�
 *  - CH0 -> g_yaw_target_deg
 *  - CH1 -> motor_set_target_counts_per_5ms
 */

#include "zf_common_headfile.h"
#include "ttop_rc_control.h"
#include "Menu.h"

static ttop_rc_control_cfg_t s_rc_cfg;
static float s_rc_speed_target_out = 0.0f;
static unsigned char s_rc_ch4_reset_latched = 0u;
static unsigned char s_rc_ch4_start_latched = 0u;
static unsigned char s_rc_emergency_active = 0u;

#ifndef TTOP_RC_SPEED_SLEW_COUNTS_PER_5MS
#define TTOP_RC_SPEED_SLEW_COUNTS_PER_5MS   (3.0f)  //1f锛氭煍鍜岋紝鍝嶅簲閫熷害鎱�  2~3f锛氶�備腑锛屽搷搴旈�熷害閫備腑  4~5f锛氬揩閫燂紝鍝嶅簲閫熷害蹇紝绋冲畾鎬у樊
#endif

#ifndef TTOP_RC_LOW_SPEED_RUN_COUNTS_PER_5MS
#define TTOP_RC_LOW_SPEED_RUN_COUNTS_PER_5MS (120.0f)
#endif

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
    // 【修改点 1】增加摇杆中位死区
    // 如果摇杆偏离中位在 ±30 以内，强制回中，防止遥控器没回正导致“前轮偷偷打歪”
    float deadzone = 30.0f;
    if (ch0 > (s_rc_cfg.yaw.in_center - deadzone) && ch0 < (s_rc_cfg.yaw.in_center + deadzone))
    {
        ch0 = s_rc_cfg.yaw.in_center;
    }

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

    // 【修改点 2】起步保底速度
    // 平衡车最难骑的就是低速(比如20~60)区间，很容易蛇形。
    // 我们强制：只要油门超了一点点(比如10)，就立刻给到一个稳定的保底速度(比如80)
    // 跨过危险起步区。
    float startup_threshold = 10.0f; // 起步推杆阈值
    float base_speed = (float)TTOP_RC_LOW_SPEED_RUN_COUNTS_PER_5MS; // 稳定的最低起步速度

    if (speed > startup_threshold && speed < base_speed)
    {
        speed = base_speed;
    }
    // 反向如果也有倒车逻辑，可以同样处理，这里假设是只往前开发车

    return (int16)roundf(speed);
}

void ttop_rc_control_init(void)
{
    // 鍒濆鍖栭粯璁ゅ弬鏁帮細鍚庣画鍙�氳繃 ttop_rc_control_set_config 瑕嗙洊銆�
    s_rc_cfg.yaw.in_min = 200.0f;
    s_rc_cfg.yaw.in_center = 800.0f;
    s_rc_cfg.yaw.in_max = 1400.0f;
    s_rc_cfg.yaw.out_min = -1.0f;
    s_rc_cfg.yaw.out_center = 0.0f;
    s_rc_cfg.yaw.out_max = 1.0f;
    s_rc_cfg.yaw.sensitivity = 0.6f;
    s_rc_cfg.yaw_rate_limit_deg_per_5ms = 3.5f;

    s_rc_cfg.speed.in_min = 800.0f;
    s_rc_cfg.speed.in_center = 800.0f;
    s_rc_cfg.speed.in_max = 1400.0f;
    s_rc_cfg.speed.out_min = 0.0f;
    s_rc_cfg.speed.out_center = 0.0f;
    s_rc_cfg.speed.out_max = 220.0f;
    s_rc_cfg.speed.sensitivity = 1.0f;

    s_rc_speed_target_out = 0.0f;
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

void ttop_rc_control_emergency_stop_now(void)
{
    int16 ch4 = uart_receiver.channel[3];

    // CH4 一键复位：
    // - 150~250：触发一次 reset，并进入急停态
    // - 1750~1850：触发一次 start，并退出急停态
    // - 950~1050：不操作，退出急停态并清除锁存，允许下一次触发
    if ((ch4 >= 150) && (ch4 <= 250))
    {
        if (!s_rc_ch4_reset_latched)
        {
            Menu_ResetAll();
            s_rc_ch4_reset_latched = 1u;
        }

        s_rc_emergency_active = 1u;
        s_rc_speed_target_out = 0.0f;
        //motor_set_target_counts_per_5ms(0);
    }
    else if ((ch4 >= 1750) && (ch4 <= 1850))
    {
        if (!s_rc_ch4_start_latched)
        {
            Menu_StartAll();
            s_rc_ch4_start_latched = 1u;
        }

        s_rc_emergency_active = 0u;
    }
    else if ((ch4 >= 950) && (ch4 <= 1050))
    {
        s_rc_ch4_reset_latched = 0u;
        s_rc_ch4_start_latched = 0u;
        s_rc_emergency_active = 0u;
    }
}

void ttop_rc_control_task_5ms(void)
{
    // 閬ユ帶鍣ㄦ湭灏辩华鎴栧け鎺ф椂涓嶆洿鏂版帶鍒堕噺銆�
    if (uart_receiver.finsh_flag == 0u) return;
    if (uart_receiver.state == 0u)
    {
        uart_receiver.finsh_flag = 0u;
        return;
    }

    float yaw_unit_cmd = ttop_rc_map_ch0_to_unit_cmd((float)uart_receiver.channel[0]);
    float yaw_rate_lim = s_rc_cfg.yaw_rate_limit_deg_per_5ms;
    float speed_target_raw;
    float delta;

    if (yaw_rate_lim < 0.0f) yaw_rate_lim = -yaw_rate_lim;

    // CH4 急停态下，持续屏蔽本周期控制输出（CH4 判定已移到 emergency_stop 函数）
    if (s_rc_emergency_active)
    {
        s_rc_speed_target_out = 0.0f;
        motor_set_target_counts_per_5ms(0);
        uart_receiver.finsh_flag = 0u;
        return;
    }

    // 【修改点 3】低速转向衰减
    // 当车速极低时（甚至0速），强烈打方向容易直接破坏平衡倒车（因为没有足够前向稳定和陀螺效应）
    // 所以在低速时，让它打方向的反应变“迟钝”一点。
    float current_speed_abs = s_rc_speed_target_out;
    if(current_speed_abs < 0) current_speed_abs = -current_speed_abs;

    float yaw_attenuation = 1.0f; // 满舵
    if (current_speed_abs < ((float)TTOP_RC_LOW_SPEED_RUN_COUNTS_PER_5MS * 0.4f)) {
        yaw_attenuation = 0.2f; // 极低速，只有 20% 舵量
    } else if (current_speed_abs < (float)TTOP_RC_LOW_SPEED_RUN_COUNTS_PER_5MS) {
        yaw_attenuation = 0.5f; // 起步初段，一半舵量
    }

    g_yaw_target_deg -= yaw_unit_cmd * yaw_rate_lim * yaw_attenuation;

    speed_target_raw = (float)ttop_rc_map_ch1_to_speed_counts((float)uart_receiver.channel[1]);
    delta = speed_target_raw - s_rc_speed_target_out;

    if (delta > TTOP_RC_SPEED_SLEW_COUNTS_PER_5MS)
        delta = TTOP_RC_SPEED_SLEW_COUNTS_PER_5MS;
    else if (delta < -TTOP_RC_SPEED_SLEW_COUNTS_PER_5MS)
        delta = -TTOP_RC_SPEED_SLEW_COUNTS_PER_5MS;

    s_rc_speed_target_out += delta;
    motor_set_target_counts_per_5ms((int16)roundf(s_rc_speed_target_out));

    uart_receiver.finsh_flag = 0u;
}
