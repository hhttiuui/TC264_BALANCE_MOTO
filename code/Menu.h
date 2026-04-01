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
