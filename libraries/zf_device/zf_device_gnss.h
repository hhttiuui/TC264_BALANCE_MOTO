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
* 鏂囦欢鍚嶇О          zf_device_gnss
* 鍏徃鍚嶇О          鎴愰兘閫愰绉戞妧鏈夐檺鍏徃
* 鐗堟湰淇℃伅          鏌ョ湅 libraries/doc 鏂囦欢澶瑰唴 version 鏂囦欢 鐗堟湰璇存槑
* 寮�鍙戠幆澧�          ADS v1.10.2
* 閫傜敤骞冲彴          TC264D
* 搴楅摵閾炬帴          https://seekfree.taobao.com/
*
* 淇敼璁板綍
* 鏃ユ湡              浣滆��               澶囨敞
* 2023-12-28       pudding            first version
********************************************************************************************************************/
/*********************************************************************************************************************
* 鎺ョ嚎瀹氫箟锛�
*                   ------------------------------------
*                   妯″潡绠¤剼            鍗曠墖鏈虹鑴�
*                   RX                  鏌ョ湅 zf_device_gnss.h 涓� GNSS_RX 瀹忓畾涔�
*                   TX                  鏌ョ湅 zf_device_gnss.h 涓� GNSS_TX 瀹忓畾涔�
*                   VCC                 3.3V鐢垫簮
*                   GND                 鐢垫簮鍦�
*                   ------------------------------------
********************************************************************************************************************/

#ifndef _zf_device_gnss_h_
#define _zf_device_gnss_h_

#include "zf_common_typedef.h"

//--------------------------------------------------------------------------------------------------
//寮曡剼閰嶇疆
//--------------------------------------------------------------------------------------------------
#define GNSS_UART           (UART_3)
#define GNSS_RX             (UART3_TX_P15_6 )                            // GPS RX寮曡剼杩炴帴鍒板崟鐗囨満姝�
#define GNSS_TX             (UART3_RX_P15_7 )                                // GPS TX涓插彛寮曡剼

#define ANGLE_TO_RAD(x)     ( (x) * PI / 180.0 )                                // 瑙掑害杞崲涓哄姬搴�
#define RAD_TO_ANGLE(x)     ( (x) * 180.0 / PI )                                // 寮у害杞崲涓鸿搴�
#define PI                  ( 3.1415926535898 )

typedef enum
{
    TAU1201 = 1,                                                                // 閫愰绉戞妧鍙岄GPS妯″潡
    GN42A   = 1,                                                                // 閫愰绉戞妧鍙岄GPS妯″潡 涓嶵AU1201鏄竴鏍风殑
    GN43RFA = 2,                                                                // 閫愰绉戞妧涓夐RTK妯″潡
}gps_device_enum;

typedef struct
{
    uint16      year;  
    uint8       month; 
    uint8       day;
    uint8       hour;
    uint8       minute;
    uint8       second;
}gps_time_struct;

typedef struct
{
    gps_time_struct    time;                                                    // 鏃堕棿
    
    uint8       state;                                                          // 鏈夋晥鐘舵��  1锛氬畾浣嶆湁鏁�  0锛氬畾浣嶆棤鏁�
    
    uint16      latitude_degree;                                                // 搴�
    uint16      latitude_cent;                                                  // 鍒�
    uint16      latitude_second;                                                // 绉�    杩欓噷鐨勭鏄鏀惧ぇ浜�100鍊嶇殑锛屼富瑕佹槸閬垮厤浣跨敤娴偣鏁�
    uint16      longitude_degree;                                               // 搴�
    uint16      longitude_cent;                                                 // 鍒�
    uint16      longitude_second;                                               // 绉�    杩欓噷鐨勭鏄鏀惧ぇ浜�100鍊嶇殑锛屼富瑕佹槸閬垮厤浣跨敤娴偣鏁�
    
    double      latitude;                                                       // 绾害
    double      longitude;                                                      // 缁忓害
    
    int8        ns;                                                             // 绾害鍗婄悆 N锛堝寳鍗婄悆锛夋垨 S锛堝崡鍗婄悆锛�
    int8        ew;                                                             // 缁忓害鍗婄悆 E锛堜笢缁忥級鎴� W锛堣タ缁忥級
    
    float       speed;                                                          // 閫熷害锛堝叕閲�/姣忓皬鏃讹級
    float       direction;                                                      // 鍦伴潰鑸悜锛�000.0~359.9 搴︼紝浠ョ湡鍖楁柟涓哄弬鑰冨熀鍑嗭級+
    uint8       antenna_direction_state;                                        // 鍙屽ぉ绾挎祴鍚戞湁鏁堢姸鎬� 1锛氭祴鍚戞湁鏁�  0锛氭祴鍚戞棤鏁堬紝鏃犳晥鏃禷ntenna_direction鏁版嵁鏄棤鏁堢殑
    float       antenna_direction;                                              // 涓诲ぉ绾挎寚鍚戜粠澶╃嚎涓庣湡鍖楁瀯鎴愮殑澶硅锛�000.0~359.9 搴︼級
    
    // 涓嬮潰涓や釜涓俊鎭粠GNGGA璇彞涓幏鍙�
    uint8       satellite_used;                                                 // 鐢ㄤ簬瀹氫綅鐨勫崼鏄熸暟閲�
    float       height;                                                         // 楂樺害
}gnss_info_struct;

typedef enum
{
    GPS_STATE_RECEIVING,                                                        // 姝ｅ湪鎺ユ敹鏁版嵁
    GPS_STATE_RECEIVED,                                                         // 鏁版嵁鎺ユ敹瀹屾垚
    GPS_STATE_PARSING,                                                          // 姝ｅ湪瑙ｆ瀽
}gps_state_enum;

extern gnss_info_struct gnss;
extern uint8            gnss_flag;


double      get_two_points_distance     (double lat1, double lng1, double lat2, double lng2);
double      get_two_points_azimuth      (double lat1, double lon1, double lat2, double lon2);
uint8       gnss_data_parse             (void);
void        gnss_uart_callback          (void);
void        gnss_init                   (gps_device_enum gps_device);

#endif
