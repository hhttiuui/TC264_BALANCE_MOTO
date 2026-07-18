/*
 * ttop_rc_control.h
 *
 *  Created on: 2026-04-05
 *      Author: hhttiuui
 *
 *  赛事功能：遥控器通道到控制量映射（CH0->g_yaw_target_deg, CH1->motor target）。
 */

#ifndef CODE_TTOP_RC_CONTROL_H_
#define CODE_TTOP_RC_CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
	float in_min;
	float in_center;
	float in_max;
	float out_min;
	float out_center;
	float out_max;
	float sensitivity;
} ttop_rc_axis_cfg_t;

typedef struct
{
	ttop_rc_axis_cfg_t yaw;
	ttop_rc_axis_cfg_t speed;
	float yaw_rate_limit_deg_per_5ms;
} ttop_rc_control_cfg_t;

void ttop_rc_control_init(void);
void ttop_rc_control_set_config(const ttop_rc_control_cfg_t *cfg);
const ttop_rc_control_cfg_t *ttop_rc_control_get_config(void);
// 以当前接收的 CH0 原始值重置 yaw 输入中心（用于开车前零点校准）
void ttop_rc_control_zero_reset_yaw_center_by_ch0(void);
void ttop_rc_control_emergency_stop_now(void);
void ttop_rc_control_task_5ms(void);
// 仅根据 RC 调整速度（不改变 yaw 目标）
void ttop_rc_control_speed_only_task_5ms(void);
// 仅根据 RC 调整方向（不改变速度目标）
void ttop_rc_control_yaw_only_task_5ms(void);
// 裸 yaw 控制：不做 MODE3 守卫，不做速度输出。供 KM3 USE_REC=2 遥控录制使用。
void ttop_rc_control_yaw_direct_task_5ms(void);

// ============================ RC Yaw 轨迹录制/回放（50ms采样） ============================
// 录制：在 5ms 调用函数内部累计到 50ms 采样一次 g_yaw_target_deg 到数组
void ttop_rc_yaw_record_start(void);
void ttop_rc_yaw_record_stop(void);
void ttop_rc_yaw_record_task_5ms(void);

// 回放：在 5ms 调用函数内部累计到 50ms 读取一个记录点并写入 g_yaw_target_deg
void ttop_rc_yaw_replay_start(void);
void ttop_rc_yaw_replay_stop(void);
void ttop_rc_yaw_replay_task_5ms(void);

// 存储：slot 1 使用 RCY0/RCY1，slot 2 使用 RCY3/RCY4
uint8 ttop_rc_yaw_record_save_to_flash(uint8 slot);
uint8 ttop_rc_yaw_record_load_from_flash(uint8 slot);

// 状态查询
uint8 ttop_rc_yaw_record_is_active(void);
uint8 ttop_rc_yaw_replay_is_active(void);
uint16 ttop_rc_yaw_record_get_count(void);
uint16 ttop_rc_yaw_replay_get_index(void);

#ifdef __cplusplus
}
#endif

#endif /* CODE_TTOP_RC_CONTROL_H_ */
