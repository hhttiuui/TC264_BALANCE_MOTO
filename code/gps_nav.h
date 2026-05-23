/*
 * gps_nav.h
 *
 *  Created on: 2026-03-28
 *      Author: hhttiuui
 *
 *  GPS 鐎佃壈鍩呭Ο鈥虫健 閳ユ柡锟斤拷 閼割亜鎮滅憴鎺曟祮閹癸拷 + 鐎佃壈鍩呴幒褍鍩�
 *
 *  閺嶇绺鹃崝鐔诲厴閿涳拷
 *  1. 鐏忥拷 GPS 閼割亜鎮滅憴鎺炵礄濮濓絽瀵抽梿璺哄閵嗕線銆庨弮鍫曟嫛濮濓絻锟斤拷0~360鎺抽敍锟�
 *     鏉烆剚宕叉稉锟� IMU 閼割亜鎮滅憴鎺炵礄娑撳﹦鏁搁崜宥嗘煙闂嗚泛瀹抽妴渚�锟藉棙妞傞柦鍫燁劀閵嗕籍锟�720鎺� 鏉╃偟鐢荤憴鎺炵礆
 *  2. 閺嶈宓� GPS 鐎涙ê鍋嶉悙纭咁吀缁犳娲伴弽鍥焻閸氭埊绱濇稉锟� IMU 鐎圭偤妾懜顏勬倻閸嬫艾妯婇敍锟�
 *     鏉堟挸鍤潪顒�鎮滄晶鐐哄櫤閸掞拷 g_yaw_target_deg閿涘矂鈹嶉崝锟� PID 闁炬崘鐭鹃幍褑顢戞潪顒�鎮�
 *
 *  閼割亜鎮滅憴鎺戠暰娑斿顕В鏃撶窗
 *  閳瑰备鏀㈤埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞顑芥敘閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳诡兘鏀㈤埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟�
 *  閳癸拷          閳癸拷 GPS                閳癸拷 IMU (AHRS yaw)       閳癸拷
 *  閳规壕鏀㈤埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞灏栨敘閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳瑰皷鏀㈤埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟�
 *  閳癸拷 闂嗚泛瀹抽弬鐟版倻 閳癸拷 濮濓絽瀵�               閳癸拷 娑撳﹦鏁搁崜宥嗘煙 (+X)        閳癸拷
 *  閳癸拷 濮濓絾鏌熼崥锟�   閳癸拷 妞ょ儤妞傞柦锟�             閳癸拷 闁棙妞傞柦锟�               閳癸拷
 *  閳癸拷 閼煎啫娲�     閳癸拷 0鎺� ~ 360鎺�          閳癸拷 鍗�720鎺� 鏉╃偟鐢荤憴锟�         閳癸拷
 *  閳规柡鏀㈤埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞绮规敘閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳癸拷閳圭补鏀㈤埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟介埞锟�
 *
 *  鏉烆剚宕查崗顒�绱￠敍姝﹎u_yaw = -( gps_heading - gps_heading_0 )  閳拷  wrap [-180, 180)
 */

#ifndef CODE_GPS_NAV_H_
#define CODE_GPS_NAV_H_

#include <stdint.h>
#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== 閸欘垵鐨熼崣鍌涙殶閿涘牆鐣敍锟� ===================== */

/* 閺嶁�冲櫙闂冭埖顔岄張锟芥径褏鐡戝鍛闂傝揪绱�100ms 鐠嬪啰鏁ゅ▎鈩冩殶閿涘绱�
 * 100濞嗭拷 鑴� 100ms = 10缁夛拷 鐡掑懏妞傞崗婊冪俺
 * 閺嶁�冲櫙闁槒绶敍娆窹S闁插洭娉﹂崚鐗堟箒閺佸牅缍呯粔浼欑礄閳拷 GPS_NAV_MIN_MOVE_FOR_HEADING_M閿涘宓嗙粩瀣煝闁拷閸戠尨绱�
 * 娑撳秴鍟�閸ュ搫鐣剧粵澶嬪姬閹碉拷閺堬拷 tick閿涙稖绉撮弮璺烘倵閹靛秴浠涢崗婊冪俺 heading_0 = 0閵嗭拷              */
#ifndef GPS_NAV_CALIB_TICKS
#define GPS_NAV_CALIB_TICKS                     (100u)
#endif

/* 閸掓媽鎻惄顔界垼閻愬湱娈戦崚銈呯暰閸楀﹤绶為敍鍫㈣儗閿涘绱濊ぐ锟� GPS 鐎圭偞妞傛担宥囩枂鐠烘繄娲伴弽鍥╁仯 < 濮濄倛绐涚粋缁樻鐠併倓璐熷鎻掑煂鏉堬拷 */
#ifndef GPS_NAV_ARRIVE_RADIUS_M
#define GPS_NAV_ARRIVE_RADIUS_M                 (0.6f)
#endif

/* yaw 濮ｏ拷 100ms 閺堬拷婢堆冨綁閸栨牠鍣洪敍鍫ユ闁喎閽╁鎴礆閿涘苯宕熸担锟� deg/100ms */
#ifndef GPS_NAV_YAW_SLEW_RATE_DEG_100MS
#define GPS_NAV_YAW_SLEW_RATE_DEG_100MS         (2.0f)
#endif

/* 閼割亜鎮滃顔筋劥閸栫尨绱檇eg閿涘绱濆顔硷拷鑲╃卜鐎电懓锟界厧鐨禍搴㈩劃閸婂吋妞傛稉宥勬叏濮濓綇绱濋梼鍙夘剾鐏忓繐绠欐惔锔藉閸旓拷 */
#ifndef GPS_NAV_HEADING_DEADZONE_DEG
#define GPS_NAV_HEADING_DEADZONE_DEG            (3.0f)
#endif

/* 閻€劏绻涚紒顓濊⒈娑擄拷 GPS 閻愮顓哥粻妤�缍嬮崜宥堢箥閸斻劏鍩呴崥鎴炴閿涘矁顩﹀Ч鍌滄畱閺堬拷鐏忓繋缍呯粔浼欑礄缁绱� */
#ifndef GPS_NAV_MIN_MOVE_FOR_HEADING_M
#define GPS_NAV_MIN_MOVE_FOR_HEADING_M          (2.0f)
#endif

/* GPS 鐎碉拷 IMU yaw 濠曞倻些閻ㄥ嫭鍙冮柅鐔哥墡濮濓絽寮弫锟� */
#ifndef GPS_NAV_DRIFT_CORR_GAIN_100MS
#define GPS_NAV_DRIFT_CORR_GAIN_100MS           (0.12f)
#endif

#ifndef GPS_NAV_DRIFT_CORR_MAX_STEP_DEG_100MS
#define GPS_NAV_DRIFT_CORR_MAX_STEP_DEG_100MS   (1.0f)
#endif

#ifndef GPS_NAV_DRIFT_CORR_DEADZONE_DEG
#define GPS_NAV_DRIFT_CORR_DEADZONE_DEG         (1.0f)
#endif

/* 鐠哄磭鍋ｆ潻鍥ㄦ姢閿涳拷100ms 閸愬懎鍘戠拋鍝ユ畱閺堬拷婢堆傜秴缁夛拷(缁拷)閿涘矁绉存潻鍥у灟鐟欏棔璐� GPS 鐠哄磭鍋� */
#ifndef GPS_NAV_OUTLIER_MAX_JUMP_M
#define GPS_NAV_OUTLIER_MAX_JUMP_M              (5.0f)
#endif

/* ===================== 鐠嬪啳鐦�/閻樿埖锟戒胶绮ㄩ弸鍕秼 ===================== */
typedef struct
{
    float gps_heading_0_deg;      /* GPS->IMU 鏉烆剚宕查梿鍓佸仯閿涘奔绱扮悮顐ｅ弮闁喐绱撶粔缁樼墡濮濓絽濮╅幀浣锋叏濮濓拷 */
    float gps_target_heading_deg; /* 瑜版挸澧犻惄顔界垼閻愯鏌熼崥鎴礄GPS閸ф劖鐖ｇ化浼欑礉0~360閿涳拷 */
    float gps_motion_heading_deg; /* 瑜版挸澧犳潪锔跨秼鏉╂劕濮╅弬鐟版倻閿涘牏鏁辨潻鐐电敾 GPS 閻愮顓哥粻妤嬬礆 */
    float imu_heading_est_deg;    /* IMU 瑜版挸澧犻懜顏勬倻閹广垻鐣婚崚锟� GPS 閸ф劖鐖ｇ化璇叉倵閻ㄥ嫪鍙婄拋鈥筹拷锟� */
    float imu_target_deg;         /* 鏉烆剚宕查崥搴ｆ畱 IMU 閸ф劖鐖ｇ化鑽ゆ窗閺嶅洩鍩呴崥锟� */
    float heading_delta_deg;      /* 閼割亜鎮滃顕嗙礄濮濓拷=闂囷拷瀹革箒娴嗛敍宀冪=闂囷拷閸欏疇娴嗛敍锟� */
    float drift_error_deg;        /* GPS 閼割亜鎮� 娑擄拷 IMU 閼割亜鎮滄稊瀣？閻ㄥ嫬浜稿锟� */
    float drift_correction_deg;   /* 閺堫剙鎳嗛張鐔风杽闂勫懏鏌﹂崝鐘垫畱閹便垽锟界喐鐗庡锝夊櫤 */
    uint8_t target_point_idx;      /* 瑜版挸澧犲锝呮躬鏉╁�熼嚋閻ㄥ嫮娲伴弽鍥╁仯缁便垹绱� */
    uint8_t nav_done;               /* 1=閹碉拷閺堝鍋ｅ鑼惰泲鐎癸拷 */
    uint8_t heading_0_valid;        /* 閸掓繂顫愰懜顏勬倻閺勵垰鎯佸鏌ユ敚鐎癸拷 */
    uint8_t gps_heading_valid;      /* 瑜版挸澧� GPS 閼割亜鎮滈弰顖氭儊閺堝鏅� */
} gps_nav_status_t;

/* ===================== 鐎电懓顦婚幒銉ュ經 ===================== */

/*
 * 閺嶇绺炬潪顒佸床閸戣姤鏆熼敍娆窹S 閼割亜鎮滅憴锟� 閳拷 IMU 閼割亜鎮滅憴锟�
 *
 * 閸欏倹鏆熼敍锟�
 *   gps_heading_deg  閳ユ柡锟斤拷 GPS 閸樼喎顫愰懜顏勬倻閿涘牊顒滈崠妤呮祩鎼达讣绱濇い鐑樻闁藉牊顒滈敍锟�0~360鎺抽敍锟�
 *   gps_heading_0_deg 閳ユ柡锟斤拷 娑撳﹦鏁搁弮璺哄煝鐠佹澘缍嶉惃锟� GPS 閸掓繂顫愰懜顏勬倻閿涘牊顒滈崠妤�鐔�閸戝棴绱�
 *
 * 鏉╂柨娲栭崐纭风窗
 *   IMU 閸ф劖鐖ｇ化璁崇瑓閻ㄥ嫮鐡戦弫鍫ｅ焻閸氭埊绱欐稉濠勬暩閸撳秵鏌熼梿璺哄閿涘矂锟藉棙妞傞柦鍫燁劀閿涘-180, 180)閿涳拷
 */
float gps_nav_heading_to_imu(float gps_heading_deg, float gps_heading_0_deg);

/*
 * 閸欏秴鎮滄潪顒佸床閸戣姤鏆熼敍娆糓U 閼割亜鎮滅憴锟� 閳拷 GPS 閼割亜鎮滅憴鎺炵礄鐠嬪啳鐦悽顭掔礆
 */
float gps_nav_imu_to_heading(float imu_yaw_deg, float gps_heading_0_deg);

/*
 * 閸掓繂顫愰崠锟� GPS 鐎佃壈鍩呭Ο鈥虫健
 * 鐠囧瓨妲戦敍锟�
 *   - 濞撳懐鈹栫�佃壈鍩呴悩鑸碉拷锟�
 *   - 娑撳﹦鏁搁崥搴ㄧ帛鐠併倓绗夐崥顖氬З鐎佃壈鍩呴敍宀勬付鐠嬪啰鏁� gps_nav_start()
 */
void gps_nav_init(void);

/*
 * 閸氼垰濮� GPS 鐎佃壈鍩�
 * 鐠囧瓨妲戦敍锟�
 *   - 婢跺秳缍呴惄顔界垼閻愬湱鍌ㄥ锟�/鐎瑰本鍨氶弽鍥х箶/閼割亜鎮滈柨浣哥暰閻樿埖锟斤拷
 *   - 閸氬海鐢婚悽锟� gps_nav_task_100ms() 濮ｏ拷100ms閼奉亜濮╅幍褑顢�
 */
void gps_nav_start(void);

/*
 * 閸嬫粍顒� GPS 鐎佃壈鍩�
 * 鐠囧瓨妲戦敍锟�
 *   - 閸嬫粍顒� gps_nav_task_100ms() 閻ㄥ嫬顕遍懜顏囩翻閸戠儤娲块弬锟�
 *   - 娣囨繄鏆�瑜版挸澧犵�佃壈鍩呴悩鑸碉拷渚婄礉娓氬じ绨崥搴ｇ敾缂佈呯敾 start
 */
void gps_nav_stop(void);

/*
 * 闁夸礁鐣炬稉濠勬暩閸掓繂顫愰懜顏勬倻
 * 閸欏倹鏆熼敍姝s_heading_deg 閳ユ柡锟斤拷 瑜版挸澧� GPS 閼割亜鎮滈敍鍫燁劀閸栨鐔�閸戝棴绱�0~360鎺抽敍锟�
 * 鐠囧瓨妲戦敍姘安閸︼拷 GPS 娣団�冲娇缁嬪啿鐣鹃崥搴ょ殶閻€劋绔村▎锟�
 */
void gps_nav_lock_heading_0(float gps_heading_deg);

/*
 * 100ms 閸涖劍婀＄�佃壈鍩呮禒璇插閿涳拷10Hz閿涘奔绗孏PS濡�虫健鏉堟挸鍤０鎴犲芳閸栧綊鍘ら敍锟�
 *
 * 瀹搞儰缍斿ù浣衡柤閿涳拷
 *   1. 閻拷 GPS 鐎圭偞妞傜紒蹇曞惈鎼达拷 + 閻╊喗鐖ｉ悙鍦病缁绢剙瀹抽敍宀�鐣婚崙锟� GPS 閸ф劖鐖ｇ化鑽ゆ窗閺嶅洩鍩呴崥锟�
 *   2. 鐏忓棛娲伴弽鍥焻閸氭垼娴嗛幑銏犲煂 IMU 閸ф劖鐖ｇ化锟�
 *   3. 娑擄拷 show_yaw_deg閿涘湜MU 鐎圭偞绁撮敍澶婁粵瀹革拷 閳拷 閼割亜鎮滃锟� delta
 *   4. 闂勬劙锟界喎閽╁鎴濇倵閸愭瑥鍙� g_yaw_target_deg
 *
 * 鏉╂柨娲栭敍锟�0=鐎佃壈鍩呮潻娑滎攽娑擃叏绱�1=閹碉拷閺堝娲伴弽鍥╁仯瀹告彃鍩屾潏锟�
 */
uint8_t gps_nav_task_100ms(void);

/*
 * 閼惧嘲褰囩�佃壈鍩呴悩鑸碉拷渚婄礄閸欘亣顕伴敍宀�鏁ゆ禍搴＄潌楠炴洘妯夌粈锟�/鐠嬪啳鐦敍锟�
 */
const gps_nav_status_t* gps_nav_status_get(void);

/*
 * 閺屻儴顕楃�佃壈鍩呴弰顖氭儊鐎瑰本鍨�
 */
uint8_t gps_nav_is_done(void);

/*
 * 閺屻儴顕楅弽鈥冲櫙闂冭埖顔岄弰顖氭儊瀹告彃鐣幋鎰剁礄閸擄拷5缁夋帒锟芥帟顓搁弮鑸垫Ц閸氾妇绮ㄩ弶鐕傜礆
 * 鏉╂柨娲栭敍锟�0=娴犲秴婀弽鈥冲櫙闂冭埖顔岄敍锟�1=閺嶁�冲櫙鐎瑰本鍨氶崣顖欎簰鐎佃壈鍩�
 */
uint8_t gps_nav_calib_done(void);

/*
 * 閺屻儴顕楅弽鈥冲櫙闂冭埖顔岄崜鈺�缍戠拋鈩冩殶閿涳拷100ms閸楁洑缍呴敍锟�0鐞涖劎銇氶弽鈥冲櫙鐎瑰本鍨氶敍锟�
 */
uint8_t gps_nav_calib_remain(void);

#ifdef __cplusplus
}
#endif

#endif /* CODE_GPS_NAV_H_ */

