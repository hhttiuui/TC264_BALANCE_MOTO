/*
 * ttop_speed_change.c
 *
 *  Created on: 2026-04-01
 *      Author: hhttiuui
 *
 *  赛事功能：不同速度对应不同 PID 的实现文件（预留）。
 */

#include "zf_common_headfile.h"
#include "ttop_speed_change.h"

// 速度低通滤波系数（0~1，越大越平滑）
#ifndef TTOP_SPEED_CHANGE_LPF_ALPHA
#define TTOP_SPEED_CHANGE_LPF_ALPHA     (0.97f)
#endif

// 动态插值目标更新分频（2 表示每 10ms 更新一次目标）
#ifndef TTOP_SPEED_CHANGE_UPDATE_DIV
#define TTOP_SPEED_CHANGE_UPDATE_DIV    (4u)
#endif

// 动态 Kp 步进按“斜率 * 速度变化量”计算：
// step ~= |dK/dv| * dv_per_5ms。
// 下面参数用于限制 dv 与最终步进的上下界，保证稳定性。
#ifndef TTOP_SPEED_CHANGE_STEP_DV_MIN
#define TTOP_SPEED_CHANGE_STEP_DV_MIN        (1.0f)
#endif

#ifndef TTOP_SPEED_CHANGE_STEP_DV_MAX
#define TTOP_SPEED_CHANGE_STEP_DV_MAX        (30.0f)
#endif

#ifndef TTOP_SPEED_CHANGE_ANGLE_STEP_MIN
#define TTOP_SPEED_CHANGE_ANGLE_STEP_MIN     (0.005f)
#endif

#ifndef TTOP_SPEED_CHANGE_RATE_STEP_MIN
#define TTOP_SPEED_CHANGE_RATE_STEP_MIN      (0.005f)
#endif

#ifndef TTOP_SPEED_CHANGE_ANGLE_STEP_MAX
#define TTOP_SPEED_CHANGE_ANGLE_STEP_MAX     (0.20f)
#endif

#ifndef TTOP_SPEED_CHANGE_RATE_STEP_MAX
#define TTOP_SPEED_CHANGE_RATE_STEP_MAX      (0.20f)
#endif

// 1/v^2 映射的最小速度保护，避免 v 过小导致过大增益。
#ifndef TTOP_SPEED_CHANGE_INV_SQ_VMIN
#define TTOP_SPEED_CHANGE_INV_SQ_VMIN    (30.0f)
#endif

typedef struct
{
	uint8_t inited;
	pid_mode_t mode;
	pid_mode_t mode_high;
	pid_mode_t mode_low;
	float speed_abs_filt;
	float speed_abs_filt_prev;
	uint8_t kp_update_div_cnt;
	float angle_kp_now;
	float angle_kp_tgt;
	float rate_kp_now;
	float rate_kp_tgt;
} ttop_speed_change_ctrl_t;

static ttop_speed_change_ctrl_t s_spd = {0u, PID_MODE_CASCADE, PID_MODE_CASCADE_EX2, PID_MODE_CASCADE_EX1, 0.0f, 0.0f, 0u, 0.0f, 0.0f, 0.0f, 0.0f};

static inline float ttop_absf_local(float x)
{
	return (x < 0.0f) ? -x : x;
}

static inline float ttop_clampf_local(float x, float lo, float hi)
{
	if (x < lo) return lo;
	if (x > hi) return hi;
	return x;
}

static inline float ttop_lerpf_local(float a, float b, float t)
{
	return a + (b - a) * t;
}

static inline float ttop_slew_toward_local(float cur, float tgt, float step)
{
	float d = tgt - cur;
	if (d > step) return cur + step;
	if (d < -step) return cur - step;
	return tgt;
}

static inline float ttop_inv_sq_local(float v)
{
	float vv = (v < (float)TTOP_SPEED_CHANGE_INV_SQ_VMIN) ? (float)TTOP_SPEED_CHANGE_INV_SQ_VMIN : v;
	return 1.0f / (vv * vv);
}

static inline const pid_mode_cfg_t *ttop_cfg_by_mode(pid_mode_t mode)
{
	int idx = (int)mode;
	if (idx < 0 || idx >= (int)PID_MODE_CFG_COUNT)
	{
		idx = (int)PID_MODE_CASCADE;
	}
	return &s_pid_mode_cfg[idx];
}

static inline uint8_t ttop_mode_is_cascade(pid_mode_t mode)
{
	const pid_mode_cfg_t *cfg = ttop_cfg_by_mode(mode);
	return (cfg->strategy == PID_STRAT_CASCADE) ? 1u : 0u;
}

void ttop_speed_change_init(pid_mode_t high_mode, pid_mode_t low_mode)
{
	s_spd.inited = 1u;
	s_spd.mode = PID_MODE_CASCADE;

	// 仅接受 cascade 策略作为高/低速参数来源，非法输入回退到 EX2/EX1。
	s_spd.mode_high = ttop_mode_is_cascade(high_mode) ? high_mode : PID_MODE_CASCADE_EX2;
	s_spd.mode_low = ttop_mode_is_cascade(low_mode) ? low_mode : PID_MODE_CASCADE_EX1;
	s_spd.speed_abs_filt = 0.0f;
	s_spd.speed_abs_filt_prev = 0.0f;
	s_spd.kp_update_div_cnt = 0u;

	const pid_mode_cfg_t *cfg_low = ttop_cfg_by_mode(s_spd.mode_low);
	s_spd.angle_kp_now = cfg_low->angle.kp;
	s_spd.angle_kp_tgt = cfg_low->angle.kp;
	s_spd.rate_kp_now = cfg_low->rate.kp;
	s_spd.rate_kp_tgt = cfg_low->rate.kp;

	// 使用低速模式作为动态链路的基础参数（限幅/ki/kd/yaw 等）。
	PID_HOT_SWITCH_BY_MODE(s_spd.mode_low);
}

void ttop_speed_change_task_5ms(void)
{
	if (!s_spd.inited)
	{
		ttop_speed_change_init(PID_MODE_CASCADE_EX2, PID_MODE_CASCADE_EX1);
	}

	// 使用编码器 5ms 增量的绝对值作为速度量。
	const float speed_abs_raw = ttop_absf_local((float)encoder_get_demo5_delta_5ms());
	s_spd.speed_abs_filt = (float)TTOP_SPEED_CHANGE_LPF_ALPHA * s_spd.speed_abs_filt +
						  (1.0f - (float)TTOP_SPEED_CHANGE_LPF_ALPHA) * speed_abs_raw;

	const pid_mode_cfg_t *cfg_low = ttop_cfg_by_mode(s_spd.mode_low);
	const pid_mode_cfg_t *cfg_high = ttop_cfg_by_mode(s_spd.mode_high);

	float v_low = (float)cfg_low->motor_target_counts_5ms;
	float v_high = (float)cfg_high->motor_target_counts_5ms;
	if (v_high <= v_low + 1e-6f)
	{
		v_low = 140.0f;
		v_high = 200.0f;
	}

	const float dv_anchor = (v_high > v_low + 1e-6f) ? (v_high - v_low) : 1.0f;
	const float angle_slope_abs = ttop_absf_local(cfg_high->angle.kp - cfg_low->angle.kp) / dv_anchor;
	const float rate_slope_abs = ttop_absf_local(cfg_high->rate.kp - cfg_low->rate.kp) / dv_anchor;

	// 以 EX1/EX2 的速度点作为严格锚点：
	// speed=v_low 时严格等于 low 的 Kp，speed=v_high 时严格等于 high 的 Kp。
	// 在 EX1-20 到 EX2+20 内按 1/v^2 比例映射，窗口外不再继续外推。
	const float v_extrap_low = v_low - 20.0f;
	const float v_extrap_high = v_high + 20.0f;

	if (++s_spd.kp_update_div_cnt >= (uint8_t)TTOP_SPEED_CHANGE_UPDATE_DIV)
	{
		s_spd.kp_update_div_cnt = 0u;
		float v_eval = s_spd.speed_abs_filt;
		if (v_eval < v_extrap_low) v_eval = v_extrap_low;
		if (v_eval > v_extrap_high) v_eval = v_extrap_high;

		if (v_eval <= v_low)
		{
			// 低速侧固定使用低速锚点参数，避免出现“低速却更像高速”的反向效果。
			s_spd.angle_kp_tgt = cfg_low->angle.kp;
			s_spd.rate_kp_tgt = cfg_low->rate.kp;
		}
		else if (v_eval >= v_high)
		{
			// 高速侧固定使用高速锚点参数。
			s_spd.angle_kp_tgt = cfg_high->angle.kp;
			s_spd.rate_kp_tgt = cfg_high->rate.kp;
		}
		else
		{
			float inv_low = ttop_inv_sq_local(v_low);
			float inv_high = ttop_inv_sq_local(v_high);
			float inv_eval = ttop_inv_sq_local(v_eval);
			float den = inv_low - inv_high;
			float alpha = 0.0f;

			if (ttop_absf_local(den) > 1e-12f)
			{
				// alpha: v_low -> 0, v_high -> 1
				alpha = (inv_low - inv_eval) / den;
			}

			alpha = ttop_clampf_local(alpha, 0.0f, 1.0f);
			s_spd.angle_kp_tgt = ttop_lerpf_local(cfg_low->angle.kp, cfg_high->angle.kp, alpha);
			s_spd.rate_kp_tgt = ttop_lerpf_local(cfg_low->rate.kp, cfg_high->rate.kp, alpha);
		}

		if (s_spd.angle_kp_tgt < 0.0f) s_spd.angle_kp_tgt = 0.0f;
		if (s_spd.rate_kp_tgt < 0.0f) s_spd.rate_kp_tgt = 0.0f;
	}

	{
		float dv_now = ttop_absf_local(s_spd.speed_abs_filt - s_spd.speed_abs_filt_prev);
		float dv_eff = ttop_clampf_local(dv_now, (float)TTOP_SPEED_CHANGE_STEP_DV_MIN, (float)TTOP_SPEED_CHANGE_STEP_DV_MAX);
		float angle_step = ttop_clampf_local(angle_slope_abs * dv_eff,
			(float)TTOP_SPEED_CHANGE_ANGLE_STEP_MIN,
			(float)TTOP_SPEED_CHANGE_ANGLE_STEP_MAX);
		float rate_step = ttop_clampf_local(rate_slope_abs * dv_eff,
			(float)TTOP_SPEED_CHANGE_RATE_STEP_MIN,
			(float)TTOP_SPEED_CHANGE_RATE_STEP_MAX);

		s_spd.angle_kp_now = ttop_slew_toward_local(s_spd.angle_kp_now, s_spd.angle_kp_tgt, angle_step);
		s_spd.rate_kp_now = ttop_slew_toward_local(s_spd.rate_kp_now, s_spd.rate_kp_tgt, rate_step);
		s_spd.speed_abs_filt_prev = s_spd.speed_abs_filt;
	}

	// 插值仅在 speed_change 模块内完成，common_flie 仅负责接收参数并执行调度。
	pid_dynamic_cascade_set_kp(s_spd.angle_kp_now, s_spd.rate_kp_now);

	// 动态 PID 专用处理（固定 cascade 链路）。
	pid_timer_handler_dynamic_cascade(s_spd.mode_low);
}

//返回当前分档选择的 PID 模式（用于调试显示）
pid_mode_t ttop_speed_change_get_mode(void){return s_spd.mode;}

//返回当前滤波后的速度绝对值（用于调试显示）
float ttop_speed_change_get_speed_abs_filt(void){return s_spd.speed_abs_filt;}

//返回当前动态插值后的 angle_kp（用于调试显示）
float ttop_speed_change_get_angle_kp(void){return s_spd.angle_kp_now;}

//返回当前动态插值后的 rate_kp（用于调试显示）
float ttop_speed_change_get_rate_kp(void){return s_spd.rate_kp_now;}