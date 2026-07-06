/*
 * Menu.h
 *
 *  Created on: 2026-01-17
 *      Author: hhttiuui
 *
 *  菜单/界面显示模块接口声明。
 */

#ifndef CODE_MENU_H_
#define CODE_MENU_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "zf_common_headfile.h"
void User_Interface(void);
void Menu_Select(void);
void Menu_StartAll(void);
void Menu_ResetAll(void);

// MODE3（RC Yaw）运行态开关：由菜单配置并在 START 时锁存
uint8 menu_mode3_run_yaw_only_enabled(void);
uint8 menu_mode3_run_record_enabled(void);
uint8 menu_mode3_run_replay_enabled(void);

extern float ips_show_pid_out;

extern float show_roll_deg;
extern float show_pitch_deg;
extern float show_yaw_deg;
extern float show_pitch_rate_dps_f;

extern int16 show_duty;

#ifdef __cplusplus
}
#endif

#endif /* CODE_MENU_H_ */
