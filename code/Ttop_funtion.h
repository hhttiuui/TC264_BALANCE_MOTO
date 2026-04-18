/*
 * Ttop_funtion.h
 *
 *  Created on: 2026-01-22
 *      Author: hhttiuui
 *
 *  璧涗簨鍔熻兘妯″潡锛堝崰浣嶏級
 */

#ifndef CODE_TTOP_FUNTION_H_
#define CODE_TTOP_FUNTION_H_

#ifdef __cplusplus
extern "C" {
#endif

// ============================ 缁熶竴鏍囧畾锛氱紪鐮佸櫒姣忕背璁℃暟 ============================
// 鏍囧畾锛氳溅琛岄┒ 1m 鏃讹紝encoder_get_demo5_count_total() 绾︿负 18813銆�
// 绾﹀畾锛氬悗缁墍鏈夎禌浜嬪姛鑳斤紙turn / 8瀛� / stop 鍙婁綘鎷嗗嚭鐨勬柊妯″潡锛夐兘搴斾娇鐢ㄨ繖涓畾涔夛紝閬垮厤鍒板鏁ｈ惤 18813銆�
#ifndef ENCODER_COUNTS_PER_M
#define ENCODER_COUNTS_PER_M          (18813LL)
#endif

// ============================ 缁熶竴鍛ㄦ湡锛氳禌浜嬩换鍔� dt锛坢s锛� ============================
// 褰撳墠璧涗簨鐩稿叧浠诲姟绾﹀畾鎸� 5ms 璋冪敤涓�娆★紙杞集/鍏瓧/鍏跺畠瑙勫垝锛夈��
// 绾﹀畾锛氬悗缁媶鍒嗙殑鏂版ā鍧楀闇� dt锛岃鐩存帴浣跨敤璇ュ畯锛岄伩鍏嶅悇鏂囦欢閲嶅瀹氫箟銆�
#ifndef TTOP_TURN_DT_MS
#define TTOP_TURN_DT_MS               (5u)
#endif

// ============================ 璧涗簨鍔熻兘锛氶噷绋嬪埌鐐瑰仠鏈� ============================
// 鍒濆鍖栵紙鍙�夛級锛氭竻闆跺仠鏈烘爣蹇�
void ttop_stop_init(void);

// 鏄惁宸茬粡瑙﹀彂鈥滃仠姝竴鍒囪皟鎺р��
uint8 ttop_stop_is_active(void);

// ============================ 灏佽涓诲惊鐜换鍔★紙鎶� if 鍧椾綋涓�璧锋惉杩囨潵锛� ============================
// 5ms 缂栫爜鍣�/鐢垫満/鎸夐敭绛変换鍔★細鍐呴儴浼氬垽鏂� yaw_delay_done / stop / encoder_flag
// 婊¤冻鏉′欢鏃惰嚜鍔ㄦ竻 encoder_flag 骞舵墽琛屽師鏉ョ殑 if{} 鍐呭
void ttop_encoder_task_5ms(void);

// 5ms/10ms PID 浠诲姟锛氬唴閮ㄤ細鍒ゆ柇 yaw_delay_done / stop / pid_flag
// 婊¤冻鏉′欢鏃惰嚜鍔ㄦ竻 pid_flag 骞舵墽琛屽師鏉ョ殑 if{} 鍐呭
void ttop_pid_task_5ms(void);

void ttop_funtion_new(void);

static void load_saved_route_points(void);

// ============================ 妯″紡鍒囨崲锛氱數鏈虹洰鏍囧�间竴娆℃�ц缃� ============================
// 寤鸿鍦� while(TRUE) 涓诲惊鐜噷鍛ㄦ湡璋冪敤锛堜緥濡傛瘡鍦堣皟鐢ㄤ竴娆″嵆鍙級銆�
// - funtion_kemu==1锛氬彧鍦ㄢ�滈娆″垏鎹㈠埌1鈥濇椂鎵ц motor_set_target_counts_per_5ms(200)
// - funtion_kemu==2锛氬彧鍦ㄢ�滈娆″垏鎹㈠埌2鈥濇椂鎵ц motor_set_target_counts_per_5ms(100)
// - 鍏跺畠鍊硷細涓嶅鐞�
void ttop_funtion_mode_once_apply(void);

void ttop_funtion_init(void);

extern volatile uint8 funtion_kemu;          // 鍔熻兘绉戠洰閫夋嫨锛�0鏃�/鏈�夛紱1~12 瀵瑰簲鑿滃崟 MODE 1~12
extern volatile uint8 funtion_ready_flag;    // 杩愯浣胯兘/ready 鏍囧織锛堣彍鍗曠‘璁ゅ悗缃� 1锛�

// ============================ GPS 宸ュ叿浜嬩欢锛堜緵鑿滃崟璋冪敤锛� ============================
// KEY1: 淇濆瓨褰撳墠 GPS 鐐�
void ttop_gps_save_point(void);
// KEY2: 鍔犺浇宸蹭繚瀛樿矾绾垮苟缁樺埗
void ttop_gps_display_route(void);
// KEY3: 娓呭睆骞舵樉绀哄府鍔╂彁绀�
void ttop_gps_clear_and_show_help(void);
// KEY4: 璁＄畻骞舵樉绀哄娈佃埅鍚戣
void ttop_gps_calc_and_show_headings(void);
// 停止 GPS 导航任务输出（不清空保存点/导航状态）
void ttop_gps_nav_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* CODE_TTOP_FUNTION_H_ */
