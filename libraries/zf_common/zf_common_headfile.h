/*********************************************************************************************************************
* TC264 Opensourec Library 鍗筹紙TC264 寮�婧愬簱锛夋槸涓�涓熀浜庡畼鏂� SDK 鎺ュ彛鐨勭涓夋柟寮�婧愬簱
* Copyright (c) 2022 SEEKFREE 閫愰绉戞妧
*
* 鏈枃浠舵槸 TC264 寮�婧愬簱鐨勪竴閮ㄥ垎
*
* TC264 寮�婧愬簱 鏄厤璐硅蒋浠�
* 鎮ㄥ彲浠ユ牴鎹嚜鐢辫蒋浠跺熀閲戜細鍙戝竷鐨� GPL锛圙NU General Public License锛屽嵆 GNU閫氱敤鍏叡璁稿彲璇侊級鐨勬潯娆�
* 鍗� GPL 鐨勭3鐗堬紙鍗� GPL3.0锛夋垨锛堟偍閫夋嫨鐨勶級浠讳綍鍚庢潵鐨勭増鏈紝閲嶆柊鍙戝竷鍜�/鎴栦慨鏀瑰畠
*
* 鏈紑婧愬簱鐨勫彂甯冩槸甯屾湜瀹冭兘鍙戞尌浣滅敤锛屼絾骞舵湭瀵瑰叾浣滀换浣曠殑淇濊瘉
* 鐢氳嚦娌℃湁闅愬惈鐨勯�傞攢鎬ф垨閫傚悎鐗瑰畾鐢ㄩ�旂殑淇濊瘉
* 鏇村缁嗚妭璇峰弬瑙� GPL
*
* 鎮ㄥ簲璇ュ湪鏀跺埌鏈紑婧愬簱鐨勫悓鏃舵敹鍒颁竴浠� GPL 鐨勫壇鏈�
* 濡傛灉娌℃湁锛岃鍙傞槄<https://www.gnu.org/licenses/>
*
* 棰濆娉ㄦ槑锛�
* 鏈紑婧愬簱浣跨敤 GPL3.0 寮�婧愯鍙瘉鍗忚 浠ヤ笂璁稿彲鐢虫槑涓鸿瘧鏂囩増鏈�
* 璁稿彲鐢虫槑鑻辨枃鐗堝湪 libraries/doc 鏂囦欢澶逛笅鐨� GPL3_permission_statement.txt 鏂囦欢涓�
* 璁稿彲璇佸壇鏈湪 libraries 鏂囦欢澶逛笅 鍗宠鏂囦欢澶逛笅鐨� LICENSE 鏂囦欢
* 娆㈣繋鍚勪綅浣跨敤骞朵紶鎾湰绋嬪簭 浣嗕慨鏀瑰唴瀹规椂蹇呴』淇濈暀閫愰绉戞妧鐨勭増鏉冨０鏄庯紙鍗虫湰澹版槑锛�
*
* 鏂囦欢鍚嶇О          zf_common_headfile
* 鍏徃鍚嶇О          鎴愰兘閫愰绉戞妧鏈夐檺鍏徃
* 鐗堟湰淇℃伅          鏌ョ湅 libraries/doc 鏂囦欢澶瑰唴 version 鏂囦欢 鐗堟湰璇存槑
* 寮�鍙戠幆澧�          ADS v1.10.2
* 閫傜敤骞冲彴          TC264D
* 搴楅摵閾炬帴          https://seekfree.taobao.com/
*
* 淇敼璁板綍
* 鏃ユ湡              浣滆��                澶囨敞
* 2022-09-15       pudding            first version
********************************************************************************************************************/

#ifndef _zf_common_headfile_h_
#define _zf_common_headfile_h_

//===================================================C璇█ 鍑芥暟搴�===================================================
#include "math.h"
#include "stdio.h"
#include "stdint.h"
#include "stdbool.h"
#include "string.h"
//===================================================C璇█ 鍑芥暟搴�===================================================

//===================================================鑺墖 SDK 搴曞眰===================================================
#include "ifxAsclin_reg.h"
#include "SysSe/Bsp/Bsp.h"
#include "IfxCcu6_Timer.h"
#include "IfxScuEru.h"
//===================================================鑺墖 SDK 搴曞眰===================================================

//====================================================寮�婧愬簱鍏叡灞�====================================================
#include "zf_common_typedef.h"
#include "zf_common_clock.h"
#include "zf_common_debug.h"
#include "zf_common_fifo.h"
#include "zf_common_font.h"
#include "zf_common_function.h"
#include "zf_common_interrupt.h"
#include "isr_config.h"
//====================================================寮�婧愬簱鍏叡灞�====================================================

//===================================================鑺墖澶栬椹卞姩灞�===================================================
#include "zf_driver_adc.h"
#include "zf_driver_delay.h"
#include "zf_driver_dma.h"
#include "zf_driver_encoder.h"
#include "zf_driver_exti.h"
#include "zf_driver_flash.h"
#include "zf_driver_gpio.h"
#include "zf_driver_pit.h"
#include "zf_driver_pwm.h"
#include "zf_driver_soft_iic.h"
#include "zf_driver_spi.h"
#include "zf_driver_soft_spi.h"
#include "zf_driver_uart.h"
#include "zf_driver_timer.h"
//===================================================鑺墖澶栬椹卞姩灞�===================================================

//===================================================澶栨帴璁惧椹卞姩灞�===================================================
#include "zf_device_absolute_encoder.h"
#include "zf_device_ble6a20.h"
#include "zf_device_bluetooth_ch9141.h"
#include "zf_device_gnss.h"
#include "zf_device_camera.h"
#include "zf_device_dl1a.h"
#include "zf_device_dl1b.h"
#include "zf_device_icm20602.h"
#include "zf_device_imu660ra.h"
#include "zf_device_imu660rb.h"
#include "zf_device_imu660rx.h"
#include "zf_device_imu963ra.h"
#include "zf_device_ips114.h"
#include "zf_device_ips200.h"
#include "zf_device_ips200pro.h"
#include "zf_device_key.h"
#include "zf_device_menc15a.h"
#include "zf_device_mpu6050.h"
#include "zf_device_mt9v03x_double.h"
#include "zf_device_oled.h"
#include "zf_device_ov7725.h"
#include "zf_device_scc8660.h"
#include "zf_device_tft180.h"
#include "zf_device_tsl1401.h"
#include "zf_device_type.h"
#include "zf_device_uart_receiver.h"
#include "zf_device_virtual_oscilloscope.h"
#include "zf_device_wifi_uart.h"
#include "zf_device_wifi_spi.h"
#include "zf_device_wireless_uart.h"
//===================================================澶栨帴璁惧椹卞姩灞�===================================================

//====================================================搴旂敤缁勪欢灞�====================================================
#include "seekfree_assistant.h"
#include "seekfree_assistant_interface.h"
//====================================================搴旂敤缁勪欢灞�====================================================

//=====================================================鐢ㄦ埛灞�======================================================

#include "common_flie.h"
#include "encoder_count.h"
#include "flash_storage.h"
#include "gps_Azimuth.h"
#include "gps_display.h"
#include "gps_key_led.h"
#include "GPS.h"
#include "imu_filter.h"
#include "ins_dead_reckoning.h"
#include "ins_path_follow.h"
#include "Key.h"
#include "Menu.h"
#include "motor_controller.h"
#include "quaternion_ahrs.h"
#include "small_driver_uart_control.h"
#include "top_8_turn_2.h"
#include "top_stop.h"
#include "Ttop_funtion.h"
#include "wifi_spi_tcp_demo.h"



//=====================================================鐢ㄦ埛灞�======================================================

#endif

