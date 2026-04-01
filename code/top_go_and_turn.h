#ifndef TOP_GO_AND_TURN_H_
#define TOP_GO_AND_TURN_H_

#include "zf_common_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================ 赛事功能：距离触发转弯（限速平滑） ============================
// 说明：
// - 目标：达到指定里程后执行一次转弯（只转一次，不做多次间隔/次数统计）。
// - 距离换算：统一使用 `ENCODER_COUNTS_PER_M`（定义在 `Ttop_funtion.h`）。
// - 函数命名：保持 `ttop_turn_*`，以适配 `Ttop_funtion.c` 现有调用。

void ttop_turn_init(void);

// 5ms 调用一次：内部判断 stop/yaw_delay_done，并输出到 common_flie 的 g_yaw_target_deg。
void ttop_turn_task_5ms(void);

#ifdef __cplusplus
}
#endif

#endif // TOP_GO_AND_TURN_H_
