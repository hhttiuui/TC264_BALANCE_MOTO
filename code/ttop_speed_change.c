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
#define TTOP_SPEED_CHANGE_LPF_ALPHA     (0.95f)
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
	float speed_pred;
	uint8_t kp_update_div_cnt;
	float angle_kp_now;
	float angle_kp_tgt;
	float rate_kp_now;
	float rate_kp_tgt;
} ttop_speed_change_ctrl_t;

static ttop_speed_change_ctrl_t s_spd = {0u, PID_MODE_CASCADE, PID_MODE_CASCADE_EX2, PID_MODE_CASCADE_EX1, 0.0f, 0.0f, 0.0f, 0u, 0.0f, 0.0f, 0.0f, 0.0f};

// 拟合方式选择（可被菜单修改）：0=1/v^2 映射，1=线性 y=kx+b
volatile uint8 g_ttop_speed_change_fit_mode = 2u;

// 速度低通滤波系数（0~1，可被菜单修改）
volatile float g_ttop_speed_change_lpf_alpha = (float)TTOP_SPEED_CHANGE_LPF_ALPHA;

// 速度预测增益（0~2）：speed_pred = speed_filt + gain * dv_filt
volatile float g_ttop_speed_change_pred_gain = 5.0f;

#define TTOP_SPEED_CHANGE_POINT_COUNT (7u)
static const pid_mode_t s_spd_point_modes[TTOP_SPEED_CHANGE_POINT_COUNT] =
{
	PID_MODE_CASCADE_EX1,
	PID_MODE_CASCADE_EX2,
	PID_MODE_CASCADE_EX3,
	PID_MODE_CASCADE_EX4,
	PID_MODE_CASCADE_EX5,
	PID_MODE_CASCADE_EX6,
	PID_MODE_CASCADE_EX7,
};

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

	// 兼容旧接口：高/低速参数仅做默认兜底，分段线性拟合使用 EX1..EX7 表。
	s_spd.mode_high = ttop_mode_is_cascade(high_mode) ? high_mode : PID_MODE_CASCADE_EX7;
	s_spd.mode_low = ttop_mode_is_cascade(low_mode) ? low_mode : PID_MODE_CASCADE_EX1;
	s_spd.speed_abs_filt = 0.0f;
	s_spd.speed_abs_filt_prev = 0.0f;
	s_spd.speed_pred = 0.0f;
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
		ttop_speed_change_init(PID_MODE_CASCADE_EX7, PID_MODE_CASCADE_EX1);
	}

	// 使用编码器 5ms 增量的绝对值作为速度量。
	const float speed_abs_raw = ttop_absf_local((float)encoder_get_demo5_delta_5ms());
	s_spd.speed_abs_filt = g_ttop_speed_change_lpf_alpha * s_spd.speed_abs_filt +
					  (1.0f - g_ttop_speed_change_lpf_alpha) * speed_abs_raw;

	// 速度预测：用滤波速度的差分作为“加速度”代理
	float dv_now = s_spd.speed_abs_filt - s_spd.speed_abs_filt_prev;
	float dv_pred = ttop_clampf_local(dv_now, -(float)TTOP_SPEED_CHANGE_STEP_DV_MAX, (float)TTOP_SPEED_CHANGE_STEP_DV_MAX);
	s_spd.speed_pred = s_spd.speed_abs_filt + g_ttop_speed_change_pred_gain * dv_pred;
	if (s_spd.speed_pred < 0.0f) s_spd.speed_pred = 0.0f;

	const pid_mode_cfg_t *cfg_low = ttop_cfg_by_mode(s_spd.mode_low);
	const pid_mode_cfg_t *cfg_high = ttop_cfg_by_mode(s_spd.mode_high);

	float v_low = (float)cfg_low->motor_target_counts_5ms;
	float v_high = (float)cfg_high->motor_target_counts_5ms;
	if (v_high <= v_low + 1e-6f)
	{
		v_low = 140.0f;
		v_high = 200.0f;
	}

	float angle_slope_abs = 0.0f;
	float rate_slope_abs = 0.0f;
	const float dv_anchor = (v_high > v_low + 1e-6f) ? (v_high - v_low) : 1.0f;
	if (dv_anchor > 1e-6f)
	{
		angle_slope_abs = ttop_absf_local(cfg_high->angle.kp - cfg_low->angle.kp) / dv_anchor;
		rate_slope_abs = ttop_absf_local(cfg_high->rate.kp - cfg_low->rate.kp) / dv_anchor;
	}

	// 以 EX1/EX2 的速度点作为严格锚点：
	// speed=v_low 时严格等于 low 的 Kp，speed=v_high 时严格等于 high 的 Kp。
	// angle_kp 在高速侧继续按 1/v^2 映射（不做上限钳制）；
	// rate_kp 也按 EX1/EX2 两锚点做同类映射，并做上部封顶。

	if (++s_spd.kp_update_div_cnt >= (uint8_t)TTOP_SPEED_CHANGE_UPDATE_DIV)
	{
		s_spd.kp_update_div_cnt = 0u;
		float inv_low = ttop_inv_sq_local(v_low);
		float inv_high = ttop_inv_sq_local(v_high);
		float den = inv_low - inv_high;

		if (g_ttop_speed_change_fit_mode == 0u)
		{
			// angle_kp：低速侧保持低速锚点；高速侧继续按 1/v^2 外推。
			if (s_spd.speed_pred <= v_low)
			{
				s_spd.angle_kp_tgt = cfg_low->angle.kp;
			}
			else
			{
				float inv_eval_angle = ttop_inv_sq_local(s_spd.speed_pred);
				float alpha_angle = 0.0f;

				if (ttop_absf_local(den) > 1e-12f)
				{
					alpha_angle = (inv_low - inv_eval_angle) / den;
				}

				if (alpha_angle < 0.0f) alpha_angle = 0.0f;
				s_spd.angle_kp_tgt = ttop_lerpf_local(cfg_low->angle.kp, cfg_high->angle.kp, alpha_angle);
			}

			// rate_kp：与 angle_kp 一样按 EX1/EX2 两锚点做 1/v^2 映射外推（上不封顶）。
			if (s_spd.speed_pred <= v_low)
			{
				s_spd.rate_kp_tgt = cfg_low->rate.kp;
			}
			else
			{
				float inv_eval_rate = ttop_inv_sq_local(s_spd.speed_pred);
				float alpha_rate = 0.0f;

				if (ttop_absf_local(den) > 1e-12f)
				{
					alpha_rate = (inv_low - inv_eval_rate) / den;
				}

				if (alpha_rate < 0.0f) alpha_rate = 0.0f;
				s_spd.rate_kp_tgt = ttop_lerpf_local(cfg_low->rate.kp, cfg_high->rate.kp, alpha_rate);
			}
		}
		else if (g_ttop_speed_change_fit_mode == 1u)
		{
			float angle_k = 0.0f;
			float angle_b = cfg_low->angle.kp;
			float rate_k = 0.0f;
			float rate_b = cfg_low->rate.kp;

			if (dv_anchor > 1e-6f)
			{
				angle_k = (cfg_high->angle.kp - cfg_low->angle.kp) / dv_anchor;
				angle_b = cfg_low->angle.kp - angle_k * v_low;
				rate_k = (cfg_high->rate.kp - cfg_low->rate.kp) / dv_anchor;
				rate_b = cfg_low->rate.kp - rate_k * v_low;
			}

			if (s_spd.speed_pred <= v_low)
			{
				s_spd.angle_kp_tgt = cfg_low->angle.kp;
				s_spd.rate_kp_tgt = cfg_low->rate.kp;
			}
			else
			{
				s_spd.angle_kp_tgt = angle_k * s_spd.speed_pred + angle_b;
				s_spd.rate_kp_tgt = rate_k * s_spd.speed_pred + rate_b;
			}
		}
		else
		{
			float speeds[TTOP_SPEED_CHANGE_POINT_COUNT];
			const pid_mode_cfg_t *cfgs[TTOP_SPEED_CHANGE_POINT_COUNT];
			uint8_t count = 0u;

			for (uint8_t i = 0u; i < (uint8_t)TTOP_SPEED_CHANGE_POINT_COUNT; i++)
			{
				const pid_mode_cfg_t *cfg = ttop_cfg_by_mode(s_spd_point_modes[i]);
				if (!ttop_mode_is_cascade(s_spd_point_modes[i]))
				{
					continue;
				}
				speeds[count] = (float)cfg->motor_target_counts_5ms;
				cfgs[count] = cfg;
				count++;
			}

			if (count < 2u)
			{
				s_spd.angle_kp_tgt = cfg_low->angle.kp;
				s_spd.rate_kp_tgt = cfg_low->rate.kp;
			}
			else
			{
				for (uint8_t i = 0u; i + 1u < count; i++)
				{
					for (uint8_t j = 0u; j + 1u < count - i; j++)
					{
						if (speeds[j] > speeds[j + 1u])
						{
							float vs = speeds[j];
							const pid_mode_cfg_t *cs = cfgs[j];
							speeds[j] = speeds[j + 1u];
							cfgs[j] = cfgs[j + 1u];
							speeds[j + 1u] = vs;
							cfgs[j + 1u] = cs;
						}
					}
				}

				float v = s_spd.speed_pred;
				uint8_t seg = 0u;
				if (v <= speeds[0])
				{
					seg = 0u;
					v = speeds[0];
				}
				else if (v >= speeds[count - 1u])
				{
					seg = (uint8_t)(count - 2u);
					v = speeds[count - 1u];
				}
				else
				{
					for (uint8_t i = 0u; i + 1u < count; i++)
					{
						if (v <= speeds[i + 1u])
						{
							seg = i;
							break;
						}
					}
				}

				float v0 = speeds[seg];
				float v1 = speeds[seg + 1u];
				float dv = v1 - v0;
				float t = (dv > 1e-6f) ? ((v - v0) / dv) : 0.0f;

				s_spd.angle_kp_tgt = ttop_lerpf_local(cfgs[seg]->angle.kp, cfgs[seg + 1u]->angle.kp, t);
				s_spd.rate_kp_tgt = ttop_lerpf_local(cfgs[seg]->rate.kp, cfgs[seg + 1u]->rate.kp, t);

				if (dv > 1e-6f)
				{
					angle_slope_abs = ttop_absf_local(cfgs[seg + 1u]->angle.kp - cfgs[seg]->angle.kp) / dv;
					rate_slope_abs = ttop_absf_local(cfgs[seg + 1u]->rate.kp - cfgs[seg]->rate.kp) / dv;
				}
			}
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

//返回当前预测速度（用于调试显示）
float ttop_speed_change_get_speed_pred(void){return s_spd.speed_pred;}

//返回当前动态插值后的 angle_kp（用于调试显示）
float ttop_speed_change_get_angle_kp(void){return s_spd.angle_kp_now;}

//返回当前动态插值后的 rate_kp（用于调试显示）
float ttop_speed_change_get_rate_kp(void){return s_spd.rate_kp_now;}