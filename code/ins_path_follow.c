/*
 * ins_path_follow.c
 *
 *  Created on: 2026-03-24
 *      Author: hhttiuui
 *
 *  鎯�у鑸� - 璺緞璺熻釜涓庣ず鏁欏綍鍒舵ā鍧�
 *
 *  ===================== Pure Pursuit 绠楁硶绠�浠� =====================
 *  Pure Pursuit锛堢函杩借釜锛夋槸涓�绉嶇粡鍏哥殑璺緞璺熻釜绠楁硶锛�
 *    1. 鍦ㄧ洰鏍囪矾寰勪笂鎵惧埌璺濊溅杈嗘渶杩戠殑鐐癸紙nearest锛�
 *    2. 浠� nearest 娌胯矾寰勫線鍓嶆壘涓�涓�"棰勭瀯鐐�"锛坙ook-ahead point锛夛紝
 *       璇ョ偣涓庤溅杈嗙殑璺濈 鈮� 棰勭瀯璺濈 L_d
 *    3. 璁＄畻杞﹁締褰撳墠浣嶇疆鍒伴鐬勭偣鐨勮埅鍚戣
 *    4. 灏嗚鑸悜瑙掍綔涓� g_yaw_target_deg锛岀敱 PID 閾捐矾鎵ц杞悜
 *
 *  棰勭瀯璺濈 L_d 鐨勪綔鐢細
 *    L_d 瓒婂ぇ 鈫� 杞集瓒婃彁鍓嶃�佽秺骞虫粦锛屼絾璺熻釜绮惧害鐣ヤ綆
 *    L_d 瓒婂皬 鈫� 璺熻釜瓒婄揣鍑戯紝浣嗗鏄撲骇鐢熸尟鑽★紙灏ゅ叾寮亾澶勶級
 *    缁忛獙鍊硷細杞﹂��(m/s) 脳 (0.5~1.5)锛屼綆閫熷皬杞﹀缓璁� 0.15~0.50m
 */

#include "zf_common_headfile.h"
#include "ins_path_follow.h"
#include "ins_dead_reckoning.h"

/* PI 甯搁噺 */
#ifndef INS_PF_PI
#define INS_PF_PI  (3.14159265358979f)
#endif

/* ================================================================
 *  杞集鍙樺寲鐜囬檺骞呭弬鏁帮紙Slew Rate Limiter锛�
 *
 *  浣滅敤锛氶檺鍒� g_yaw_target_deg 姣� 5ms 鏈�澶у彉鍖栭噺锛�
 *        閬垮厤 Pure Pursuit 鍦ㄦ嫄寮鐨勭洰鏍囪埅鍚戠獊鍙�
 *        瀵艰嚧 Yaw PID 鐚涙墦 鈫� 灏忚溅鍊掑湴銆�
 *
 *  INS_YAW_SLEW_RATE_DPS锛氭渶澶у厑璁哥殑鐩爣鑸悜鍙樺寲閫熺巼锛坉eg/s锛�
 *    - 鍊艰秺澶э細杞集瓒婂揩锛堢伒鏁忥級锛屼絾瓒婂鏄撳�掕溅
 *    - 鍊艰秺灏忥細杞集瓒婃參锛堝钩婊戯級锛屼絾鍙兘璺熶笉涓婃�ュ集
 *    - 寤鸿浠� 60~120 deg/s 寮�濮嬭皟璇�
 *    - 鍙傝�冿細鍏瓧缁曡榛樿涔熸槸 60 deg/s
 *
 *  姣� 5ms 姝ヨ繘閲� = INS_YAW_SLEW_RATE_DPS 脳 0.005
 *    60 deg/s  鈫� 姣� 5ms 鏈�澶氬彉鍖� 0.30掳
 *    90 deg/s  鈫� 姣� 5ms 鏈�澶氬彉鍖� 0.45掳
 *    120 deg/s 鈫� 姣� 5ms 鏈�澶氬彉鍖� 0.60掳
 * ================================================================ */
#ifndef INS_YAW_SLEW_RATE_DPS
#define INS_YAW_SLEW_RATE_DPS   (70.0f)
#endif

/* 姣� 5ms 璋冪敤涓�娆℃椂鐨勬渶澶ф杩涳紙deg锛� */
#define INS_YAW_SLEW_MAX_STEP   ((float)INS_YAW_SLEW_RATE_DPS * 0.005f)

/* ================================================================
 *  鑸偣琛紙鍏ㄥ眬锛屽澶栨毚闇诧級
 * ================================================================ */
volatile ins_waypoint_t g_ins_path[INS_PATH_MAX_POINTS];
volatile uint16         g_ins_path_count = 0u;

/* ================================================================
 *  鍐呴儴鐘舵�侊細褰曞埗
 * ================================================================ */
static uint8  s_record_active      = 0u;     /* 1=褰曞埗涓� 0=鏈綍鍒� */
static float  s_record_last_odo_m  = 0.0f;   /* 涓婃褰曞埗鐐圭殑閲岀▼ */
static uint8  s_straight_mode      = 0u;     /* 杩熸粸鐘舵�侊細1=褰撳墠鍒ゅ畾涓虹洿绾� */

/* ================================================================
 *  鍐呴儴鐘舵�侊細璺熻釜
 * ================================================================ */
static uint8  s_follow_done     = 0u;   /* 1=宸插埌缁堢偣 */
static uint16 s_nearest_idx     = 0u;   /* 鏈�杩戣埅鐐圭储寮� */
static uint16 s_lookahead_idx   = 0u;   /* 棰勭瀯鑸偣绱㈠紩 */
static float  s_cte             = 0.0f; /* 妯悜鍋忓樊锛堢背锛� */
static float  s_yaw_slew_out    = 0.0f; /* 鏂滅巼闄愬箙鍣ㄥ綋鍓嶈緭鍑猴紙deg锛� */
static uint8  s_yaw_slew_inited = 0u;   /* 鏂滅巼闄愬箙鍣ㄦ槸鍚﹀凡鍒濆鍖� */

/* ================================================================
 *  杈呭姪鍑芥暟
 * ================================================================ */

/* 涓ょ偣璺濈鐨勫钩鏂癸紙閬垮厤 sqrt锛岀敤浜庢瘮杈冨ぇ灏忔椂鏇村揩锛� */
static inline float ins_dist2(float x1, float y1, float x2, float y2)
{
    float dx = x2 - x1;
    float dy = y2 - y1;
    return dx * dx + dy * dy;
}

/* 涓ょ偣璺濈 */
static inline float ins_distf(float x1, float y1, float x2, float y2)
{
    return sqrtf(ins_dist2(x1, y1, x2, y2));
}

/* 瑙掑害褰掍竴鍖栧埌 [-180, +180) */
static inline float ins_wrap_deg(float deg)
{
    while (deg >  180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

/* ================================================================
 *  Yaw 鐩爣鏂滅巼闄愬箙鍣紙Slew Rate Limiter锛�
 *
 *  瀵� Pure Pursuit 璁＄畻鍑虹殑鍘熷鐩爣鑸悜鍋氶檺閫熷钩婊戯細
 *  姣� 5ms 鏈�澶氬彉鍖� INS_YAW_SLEW_MAX_STEP 搴︼紝
 *  闇�瑕佹纭鐞嗚搴︾幆缁曪紙-180/+180 璺冲彉锛夈��
 * ================================================================ */
static float ins_yaw_slew_limit(float raw_target_deg)
{
    /* 棣栨璋冪敤锛氱洿鎺ヨ窡闅忥紝涓嶅仛闄愬箙 */
    if (!s_yaw_slew_inited)
    {
        s_yaw_slew_inited = 1u;
        s_yaw_slew_out = raw_target_deg;
        return s_yaw_slew_out;
    }

    /* 璁＄畻瑙掑害宸紙澶勭悊 -180/+180 璺冲彉锛� */
    float err = ins_wrap_deg(raw_target_deg - s_yaw_slew_out);

    /* 闄愬箙 */
    if (err > INS_YAW_SLEW_MAX_STEP)
        err = INS_YAW_SLEW_MAX_STEP;
    else if (err < -INS_YAW_SLEW_MAX_STEP)
        err = -INS_YAW_SLEW_MAX_STEP;

    s_yaw_slew_out = ins_wrap_deg(s_yaw_slew_out + err);
    return s_yaw_slew_out;
}

/* ================================================================
 *  录制结束后路径平滑
 *
 *  作用：
 *    1) 让录制轨迹整体更顺滑，减小手推抖动
 *    2) 让拐角由“尖折线”变得更圆滑
 *
 *  方法：
 *    对内部点做多轮三点平滑：
 *      avg = 0.25*prev + 0.5*curr + 0.25*next
 *      curr = curr + blend * (avg - curr)
 *
 *  特点：
 *    - 首尾点保持不动
 *    - 轮数越多，轨迹越圆滑
 *    - blend 越大，平滑越强
 * ================================================================ */
static void ins_record_smooth_path(void)
{
    if (g_ins_path_count < 3u) return;

    static float sx[INS_PATH_MAX_POINTS];
    static float sy[INS_PATH_MAX_POINTS];

    uint16 count = g_ins_path_count;
    uint16 pass;

    for (pass = 0u; pass < (uint16)INS_RECORD_SMOOTH_PASSES; pass++)
    {
        uint16 i;

        sx[0] = g_ins_path[0].x;
        sy[0] = g_ins_path[0].y;
        sx[count - 1u] = g_ins_path[count - 1u].x;
        sy[count - 1u] = g_ins_path[count - 1u].y;

        for (i = 1u; i + 1u < count; i++)
        {
            float prev_x = g_ins_path[i - 1u].x;
            float prev_y = g_ins_path[i - 1u].y;
            float curr_x = g_ins_path[i].x;
            float curr_y = g_ins_path[i].y;
            float next_x = g_ins_path[i + 1u].x;
            float next_y = g_ins_path[i + 1u].y;

            float avg_x = 0.25f * prev_x + 0.50f * curr_x + 0.25f * next_x;
            float avg_y = 0.25f * prev_y + 0.50f * curr_y + 0.25f * next_y;

            sx[i] = curr_x + (avg_x - curr_x) * (float)INS_RECORD_SMOOTH_BLEND;
            sy[i] = curr_y + (avg_y - curr_y) * (float)INS_RECORD_SMOOTH_BLEND;
        }

        for (i = 1u; i + 1u < count; i++)
        {
            g_ins_path[i].x = sx[i];
            g_ins_path[i].y = sy[i];
        }
    }
}


/* ================================================================
 *  褰曞埗婊ゆ尝锛氭粦鍔ㄧ獥鍙ｇ洿绾垮垽瀹� + 杩熸粸 + 寮亾涓嶈繃婊�
 *
 *  绠楁硶娴佺▼锛�
 *    1) 鍙栫獥鍙ｅ唴鏈�杩� N 涓偣锛堝惈鏂扮偣锛夌殑杩炵画绾挎
 *    2) 璁＄畻姣忔鑸悜瑙掞紝鍐嶈绠楃浉閭昏埅鍚戣涔嬪樊
 *    3) 鐢ㄨ繜婊炲垽瀹氬綋鍓嶆槸鍚�"鐩寸嚎妯″紡"锛�
 *       - 鏈繘鍏ョ洿绾挎ā寮忔椂锛氭墍鏈夎宸� < ENTER 闃堝�� 鈫� 杩涘叆
 *       - 宸插湪鐩寸嚎妯″紡鏃讹細浠讳竴瑙掑樊 > EXIT  闃堝�� 鈫� 閫�鍑�
 *    4) 濡傛灉鏄洿绾挎ā寮忥細
 *       - 鍙栨渶鍚庝竴娈� (P[n-1] 鈫� P_raw) 鐨勬柟鍚戜负鍙傝�冪嚎
 *       - 璁＄畻鏂扮偣鍒板弬鑰冪嚎鐨勬í鍚戝亸宸�
 *       - 鑻ュ亸宸皬 鈫� 鎶曞奖鍒板弬鑰冪嚎
 *       - 鑻ュ亸宸ぇ 鈫� 淇濇寔鍘熺偣锛堝紓甯歌烦鐐逛繚鎶わ級
 *    5) 濡傛灉涓嶆槸鐩寸嚎妯″紡锛氫笉鍋氫换浣曚慨鏀癸紙淇濈暀寮亾鍘熷鐐癸級
 * ================================================================ */
static void ins_record_filter_point(float *px, float *py)
{
    if ((float *)0 == px || (float *)0 == py) return;

    /* 绐楀彛鍐呴渶瑕� (WINDOW_SIZE - 1) 涓凡鏈夌偣 + 1 涓柊鐐� */
    uint16 win = (uint16)INS_FILTER_WINDOW_SIZE;
    if (g_ins_path_count < (uint16)(win - 1u)) return;   /* 鐐瑰お灏戯紝璺宠繃 */

    /* ---- 鏀堕泦绐楀彛鍐呭悇娈电殑鑸悜瑙� ---- */
    /* 娈垫暟 = win-1 = 3  (P[n-3]鈫扨[n-2], P[n-2]鈫扨[n-1], P[n-1]鈫扨_raw) */
    uint16 n_segs = (uint16)(win - 1u);
    float  angles[3];    /* 鏈�澶� 3 娈碉紙绐楀彛 = 4锛� */
    uint8  seg_ok = 1u;  /* 鏄惁鎵�鏈夋闀垮害閮藉悎鏍� */

    uint16 i;
    for (i = 0u; i < n_segs; i++)
    {
        float ax, ay, bx, by;

        if (i < (uint16)(n_segs - 1u))
        {
            /* 宸叉湁鑸偣涔嬮棿鐨勬 */
            uint16 idx_a = (uint16)(g_ins_path_count - (win - 1u) + i);
            uint16 idx_b = (uint16)(idx_a + 1u);
            ax = g_ins_path[idx_a].x;
            ay = g_ins_path[idx_a].y;
            bx = g_ins_path[idx_b].x;
            by = g_ins_path[idx_b].y;
        }
        else
        {
            /* 鏈�鍚庝竴娈碉細P[n-1] 鈫� P_raw锛堟柊鐐癸級 */
            uint16 idx_a = (uint16)(g_ins_path_count - 1u);
            ax = g_ins_path[idx_a].x;
            ay = g_ins_path[idx_a].y;
            bx = *px;
            by = *py;
        }

        float sdx = bx - ax;
        float sdy = by - ay;
        float slen = sqrtf(sdx * sdx + sdy * sdy);
        if (slen < (float)INS_FILTER_MIN_SEG_M)
        {
            seg_ok = 0u;
            break;
        }
        angles[i] = atan2f(sdy, sdx) * (180.0f / INS_PF_PI);
    }

    /* 濡傛灉鏈変换浣曟澶煭 鈫� 鏂瑰悜瑙掍笉鍙潬锛岃烦杩囨护娉紝淇濇寔鍘熺偣 */
    if (!seg_ok)
    {
        s_straight_mode = 0u;
        return;
    }

    /* ---- 璁＄畻鐩搁偦娈电殑鑸悜瑙掑樊 ---- */
    /* 瑙掑樊鏁� = n_segs - 1 = 2 */
    float max_angle_diff = 0.0f;
    for (i = 0u; i + 1u < n_segs; i++)
    {
        float diff = fabsf(ins_wrap_deg(angles[i + 1u] - angles[i]));
        if (diff > max_angle_diff) max_angle_diff = diff;
    }

    /* ---- 杩熸粸鍒ゅ畾锛氭槸鍚﹁繘鍏�/閫�鍑虹洿绾挎ā寮� ---- */
    if (s_straight_mode == 0u)
    {
        /* 鏈湪鐩寸嚎妯″紡 鈫� 鐢� ENTER 闃堝�硷紙鏇翠弗鏍硷級鍒ゅ畾鏄惁杩涘叆 */
        if (max_angle_diff < (float)INS_FILTER_ANGLE_ENTER_DEG)
        {
            s_straight_mode = 1u;
        }
    }
    else
    {
        /* 宸插湪鐩寸嚎妯″紡 鈫� 鐢� EXIT 闃堝�硷紙鏇村鏉撅級鍒ゅ畾鏄惁閫�鍑� */
        if (max_angle_diff > (float)INS_FILTER_ANGLE_EXIT_DEG)
        {
            s_straight_mode = 0u;
        }
    }

    /* ---- 濡傛灉涓嶅湪鐩寸嚎妯″紡 鈫� 寮亾锛屼笉鍋氫换浣曚慨鏀� ---- */
    if (s_straight_mode == 0u) return;

    /* ---- 鍦ㄧ洿绾挎ā寮� 鈫� 灏嗘柊鐐规姇褰卞埌鏈�鍚庝竴娈垫柟鍚戠嚎涓� ---- */
    {
        uint16 ref_idx = (uint16)(g_ins_path_count - 1u);
        float rx = g_ins_path[ref_idx].x;
        float ry = g_ins_path[ref_idx].y;

        /* 鍙傝�冩柟鍚戯細鐢ㄦ渶鍚庝竴娈靛凡鏈夌嚎娈电殑鏂瑰悜锛堟洿绋冲畾锛� */
        float dx, dy, seg_len;
        if (g_ins_path_count >= 2u)
        {
            uint16 ref_prev = (uint16)(g_ins_path_count - 2u);
            dx = g_ins_path[ref_idx].x - g_ins_path[ref_prev].x;
            dy = g_ins_path[ref_idx].y - g_ins_path[ref_prev].y;
            seg_len = sqrtf(dx * dx + dy * dy);
        }
        else
        {
            dx = *px - rx;
            dy = *py - ry;
            seg_len = sqrtf(dx * dx + dy * dy);
        }

        if (seg_len < 0.001f) return;  /* 瀹夊叏妫�鏌� */

        /* 璁＄畻鏂扮偣鐩稿鍙傝�冪偣鐨勬í鍚戝亸宸� */
        float vx = (*px) - rx;
        float vy = (*py) - ry;
        float lateral = fabsf((vx * dy - vy * dx) / seg_len);

        /* 妯悜鍋忓樊澶ぇ 鈫� 寮傚父鐐癸紝涓嶆姇褰� */
        if (lateral > (float)INS_FILTER_LATERAL_TH_M) return;

        /* 娌垮弬鑰冩柟鍚戞姇褰� */
        float t = (vx * dx + vy * dy) / (seg_len * seg_len);
        if (t < 0.0f) t = 0.0f;  /* 涓嶅厑璁稿線鍥炴姇褰� */

        *px = rx + t * dx;
        *py = ry + t * dy;
    }
}


/* ================================================================
 *  A. 璺緞褰曞埗
 * ================================================================ */

void ins_path_record_init(void)
{
    /* 娓呯┖鑸偣琛� */
    g_ins_path_count = 0u;
    s_record_active = 1u;
    s_record_last_odo_m = 0.0f;
    s_straight_mode = 0u;

    /* 閲嶇疆鑸抗鎺ㄧ畻锛氫綅缃綊闆� */
    ins_dr_reset();

    /* 璁板綍璧峰鐐� (0, 0) */
    if (g_ins_path_count < (uint16)INS_PATH_MAX_POINTS)
    {
        g_ins_path[g_ins_path_count].x = 0.0f;
        g_ins_path[g_ins_path_count].y = 0.0f;
        g_ins_path_count++;
    }
}

uint16 ins_path_record_task_5ms(void)
{
    if (!s_record_active) return g_ins_path_count;

    /* 鑸偣琛ㄥ綍婊� 鈫� 鑷姩鍋滄 */
    if (g_ins_path_count >= (uint16)INS_PATH_MAX_POINTS)
    {
        s_record_active = 0u;
        return g_ins_path_count;
    }

    /* 鏇存柊鑸抗鎺ㄧ畻锛堜綅缃� + 閲岀▼锛� */
    ins_dr_update_5ms();

    /* 妫�鏌ユ槸鍚﹁椹朵簡瓒冲鐨勮窛绂� 鈫� 褰曞叆鏂拌埅鐐� */
    float odo = ins_dr_get_odometer_m();
    if ((odo - s_record_last_odo_m) >= (float)INS_RECORD_INTERVAL_M)
    {
        float rec_x = ins_dr_get_pos_x();
        float rec_y = ins_dr_get_pos_y();

        ins_record_filter_point(&rec_x, &rec_y);

        g_ins_path[g_ins_path_count].x = rec_x;
        g_ins_path[g_ins_path_count].y = rec_y;
        g_ins_path_count++;
        s_record_last_odo_m = odo;
    }

    return g_ins_path_count;
}

void ins_path_record_stop(void)
{
    if (!s_record_active) return;
    s_record_active = 0u;

    /* 琛ュ綍鏈�鍚庝竴涓偣锛堝綋鍓嶄綅缃級锛岄伩鍏嶅熬娈典涪澶� */
    if (g_ins_path_count > 0u && g_ins_path_count < (uint16)INS_PATH_MAX_POINTS)
    {
        float cx = ins_dr_get_pos_x();
        float cy = ins_dr_get_pos_y();

        /* 涓庝笂涓�涓埅鐐硅窛绂诲お杩戯紙< 1cm锛夊垯涓嶉噸澶嶅綍鍏� */
        float last_x = g_ins_path[g_ins_path_count - 1u].x;
        float last_y = g_ins_path[g_ins_path_count - 1u].y;
        if (ins_distf(cx, cy, last_x, last_y) > 0.01f)
        {
            g_ins_path[g_ins_path_count].x = cx;
            g_ins_path[g_ins_path_count].y = cy;
            g_ins_path_count++;
        }
    }
}

uint8 ins_path_record_is_active(void) { return s_record_active; }

/* ================================================================
 *  B. 璺緞鐢熸垚锛堟暟瀛﹀叕寮忥級
 * ================================================================ */

void ins_path_gen_circle(float radius_m, uint16 num_points)
{
    if (num_points > (uint16)INS_PATH_MAX_POINTS) num_points = (uint16)INS_PATH_MAX_POINTS;
    if (num_points < 4u) num_points = 4u;

    float R = fabsf(radius_m);
    if (R < 0.01f) R = 0.01f;

    /* 姝ｅ�� = 宸﹁浆锛堥�嗘椂閽堬級锛岃礋鍊� = 鍙宠浆锛堥『鏃堕拡锛� */
    float dir = (radius_m >= 0.0f) ? 1.0f : -1.0f;

    /*
     * 鍦嗗舰璺緞锛岃捣鐐� (0, 0)锛屽垵濮嬫湞鍚� +X锛堜笌 yaw=0掳 涓�鑷达級锛�
     *
     *   宸﹁浆锛坉ir=+1锛夛細鍦嗗績 = (0, +R)
     *     x(t) = R 脳 sin(t)
     *     y(t) = R 脳 (1 - cos(t))
     *     t=0: (0, 0)  t=蟺/2: (R, R)  t=蟺: (0, 2R)
     *
     *   鍙宠浆锛坉ir=-1锛夛細鍦嗗績 = (0, -R)
     *     x(t) = R 脳 sin(t)
     *     y(t) = -R 脳 (1 - cos(t))
     *     t=0: (0, 0)  t=蟺/2: (R, -R)  t=蟺: (0, -2R)
     *
     *   t 鈭� [0, 2蟺) 鈫� 璧板畬涓�鏁村湀鍥炲埌 (0,0)
     */
    g_ins_path_count = 0u;
    for (uint16 i = 0u; i < num_points; i++)
    {
        float t = (float)i * (2.0f * INS_PF_PI / (float)num_points);
        g_ins_path[i].x = R * sinf(t);
        g_ins_path[i].y = dir * R * (1.0f - cosf(t));
    }
    g_ins_path_count = num_points;
}

void ins_path_gen_eight(float radius_m, uint16 num_points_per_circle)
{
    float R = fabsf(radius_m);
    if (R < 0.01f) R = 0.01f;

    uint16 total = num_points_per_circle * 2u;
    if (total > (uint16)INS_PATH_MAX_POINTS) total = (uint16)INS_PATH_MAX_POINTS;
    uint16 npc = total / 2u;

    g_ins_path_count = 0u;

    /*
     * 8 瀛� = 鍏堝乏杞竴鍦� + 鍐嶅彸杞竴鍦�
     *
     * 宸﹀渾锛氬渾蹇� (0, +R)锛岄�嗘椂閽�
     *   x(t) = R 脳 sin(t)
     *   y(t) = R 脳 (1 - cos(t))
     *   t 鈭� [0, 2蟺)
     *   璧风偣 (0,0)锛岀粓鐐瑰洖鍒� (0,0)
     *
     * 鍙冲渾锛氬渾蹇� (0, -R)锛岄『鏃堕拡
     *   x(t) = R 脳 sin(t)
     *   y(t) = -R 脳 (1 - cos(t))
     *   t 鈭� [0, 2蟺)
     *   璧风偣 (0,0)锛岀粓鐐瑰洖鍒� (0,0)
     */

    /* ---- 宸﹀渾锛堥�嗘椂閽堬級---- */
    for (uint16 i = 0u; i < npc; i++)
    {
        float t = (float)i * (2.0f * INS_PF_PI / (float)npc);
        g_ins_path[i].x = R * sinf(t);
        g_ins_path[i].y = R * (1.0f - cosf(t));
    }

    /* ---- 鍙冲渾锛堥『鏃堕拡锛�---- */
    for (uint16 i = 0u; i < npc; i++)
    {
        float t = (float)i * (2.0f * INS_PF_PI / (float)npc);
        g_ins_path[npc + i].x = R * sinf(t);
        g_ins_path[npc + i].y = -R * (1.0f - cosf(t));
    }

    g_ins_path_count = total;
}

/* ================================================================
 *  C. 璺緞璺熻釜锛圥ure Pursuit锛�
 * ================================================================ */

void ins_path_follow_init(void)
{
    /* 鑸抗鎺ㄧ畻褰掗浂锛氳窡韪捣鐐� = 鍧愭爣鍘熺偣 */
    ins_dr_reset();

    /* 璺熻釜鐘舵�佸綊闆� */
    s_follow_done   = 0u;
    s_nearest_idx   = 0u;
    s_lookahead_idx = 0u;
    s_cte           = 0.0f;

    /* 鏂滅巼闄愬箙鍣ㄥ綊闆讹細涓嬫杩涘叆鏃堕噸鏂板垵濮嬪寲 */
    s_yaw_slew_inited = 0u;
    s_yaw_slew_out    = 0.0f;
}

uint8 ins_path_follow_task_5ms(void)
{
    /* 鑸偣涓嶈冻锛屾棤娉曡窡韪� */
    if (g_ins_path_count < 2u)
    {
        s_follow_done = 1u;
        return 1u;
    }

    /* 宸插埌杈剧粓鐐癸紝淇濇寔鏈�鍚庝竴娆¤緭鍑� */
    if (s_follow_done) return 1u;

    /* ---- 1) 鏇存柊鑸抗鎺ㄧ畻 鈫� 鑾峰彇褰撳墠浣嶇疆 ---- */
    ins_dr_update_5ms();

    float cx = ins_dr_get_pos_x();
    float cy = ins_dr_get_pos_y();

    /* ---- 2) 鎼滅储鏈�杩戣埅鐐癸紙浠庝笂娆� nearest 闄勮繎鎼滅储锛岄伩鍏嶅叏琛ㄩ亶鍘嗭級----
     *
     *  鎼滅储绐楀彛锛歔nearest - 5, nearest + 30]
     *  鍚戝墠鍋忕疆鏇村锛屽洜涓鸿溅鍙線鍓嶈蛋锛宯earest 鍙簲鍓嶈繘涓嶅悗閫�
     */
    {
        uint16 best_idx = s_nearest_idx;
        float  best_d2  = ins_dist2(cx, cy,
                            g_ins_path[best_idx].x,
                            g_ins_path[best_idx].y);

        uint16 search_start = (s_nearest_idx > 5u) ? (s_nearest_idx - 5u) : 0u;
        uint16 search_end   = s_nearest_idx + 30u;
        if (search_end >= g_ins_path_count) search_end = g_ins_path_count - 1u;

        for (uint16 i = search_start; i <= search_end; i++)
        {
            float d2 = ins_dist2(cx, cy, g_ins_path[i].x, g_ins_path[i].y);
            if (d2 < best_d2)
            {
                best_d2  = d2;
                best_idx = i;
            }
        }
        s_nearest_idx = best_idx;
    }

    /* ---- 3) 鎼滅储棰勭瀯鐐癸紙look-ahead point锛�----
     *
     *  浠� nearest 寮�濮嬪悜鍓嶆壘锛岀洿鍒版煇鑸偣涓庡綋鍓嶄綅缃殑璺濈 >= 棰勭瀯璺濈 L_d銆�
     *  鑻ュ埌璺緞鏈熬浠嶆湭杈惧埌 L_d锛屽垯浣跨敤鏈�鍚庝竴涓埅鐐广��
     */
    {
        float La = (float)INS_PURSUIT_LOOKAHEAD_M;
        uint16 la_idx = s_nearest_idx;

        for (uint16 i = s_nearest_idx; i < g_ins_path_count; i++)
        {
            float d = ins_distf(cx, cy, g_ins_path[i].x, g_ins_path[i].y);
            la_idx = i;
            if (d >= La)
            {
                break;  /* 鎵惧埌浜嗘弧瓒抽鐬勮窛绂荤殑鑸偣 */
            }
        }
        s_lookahead_idx = la_idx;
    }

    /* ---- 4) 缁堢偣鍒ゅ畾 ----
     *
     *  鏉′欢锛氶鐬勭偣宸茬粡鏄渶鍚庝竴涓埅鐐� 涓� 璺濈缁堢偣 < 鍒ゅ畾鍗婂緞
     */
    {
        float dist_to_end = ins_distf(cx, cy,
            g_ins_path[g_ins_path_count - 1u].x,
            g_ins_path[g_ins_path_count - 1u].y);

        if (s_lookahead_idx >= g_ins_path_count - 1u
            && dist_to_end < (float)INS_PATH_END_RADIUS_M)
        {
            s_follow_done = 1u;
            return 1u;
        }
    }

    /* ---- 5) 璁＄畻鐩爣鑸悜瑙� 鈫� 闄愬箙骞虫粦 鈫� 鍐欏叆 g_yaw_target_deg ----
     *
     *  鐩爣鑸悜 = atan2(goal_y - pos_y, goal_x - pos_x)
     *  璇ヨ搴﹀湪涓� show_yaw_deg 鐩稿悓鐨勫潗鏍囩郴涓紙0掳 = +X锛岄�嗘椂閽堜负姝ｏ級
     *
     *  銆愰檺骞呭钩婊戙�戯細Pure Pursuit 璁＄畻鍑虹殑 target_deg 鍙兘鍦ㄥ集閬撳
     *  浜х敓绐佸彉锛堜緥濡傜洿绾库啋寮亾鐬棿璺冲彉 30~90掳锛夛紝瀵艰嚧 Yaw PID
     *  鐚涙墦鑸垫満 鈫� 灏忚溅澶у箙鍋忚浆鍊掑湴銆�
     *  閫氳繃 ins_yaw_slew_limit() 闄愬埗姣� 5ms 鏈�澶у彉鍖栭噺锛�
     *  浣胯浆寮繃绋嬪钩婊戙�佸彲鎺с��
     */
    {
        float goal_x = g_ins_path[s_lookahead_idx].x;
        float goal_y = g_ins_path[s_lookahead_idx].y;

        float target_rad = atan2f(goal_y - cy, goal_x - cx);
        float target_deg = target_rad * (180.0f / INS_PF_PI);

        /* 鏂滅巼闄愬箙锛氶檺鍒剁洰鏍囪埅鍚戝彉鍖栭�熺巼锛岄槻姝㈡�ヨ浆鍊掕溅 */
        g_yaw_target_deg = ins_yaw_slew_limit(target_deg);
    }

    /* ---- 6) 璁＄畻妯悜鍋忓樊 CTE锛堢敤浜庤皟璇�/鏄剧ず锛�----
     *
     *  鏂规硶锛氭渶杩戣埅鐐� 鈫� 涓嬩竴涓埅鐐规瀯鎴愯矾寰勭嚎娈� AB锛�
     *        杞﹁締浣嶇疆 C 鍒� AB 鐨勫甫绗﹀彿鍨傜洿璺濈銆�
     *
     *  鍏紡锛欳TE = ((B-A) 脳 (C-A)) / |B-A|
     *        鍙夌Н姝� 鈫� C 鍦� AB 宸︿晶锛圕TE > 0锛�
     *        鍙夌Н璐� 鈫� C 鍦� AB 鍙充晶锛圕TE < 0锛�
     */
    if (s_nearest_idx + 1u < g_ins_path_count)
    {
        float ax = g_ins_path[s_nearest_idx].x;
        float ay = g_ins_path[s_nearest_idx].y;
        float bx = g_ins_path[s_nearest_idx + 1u].x;
        float by = g_ins_path[s_nearest_idx + 1u].y;

        float abx = bx - ax;
        float aby = by - ay;
        float acx = cx - ax;
        float acy = cy - ay;

        float ab_len = sqrtf(abx * abx + aby * aby);
        if (ab_len > 0.001f)
        {
            /* 浜岀淮鍙夌Н锛欰B 脳 AC = abx*acy - aby*acx */
            s_cte = (abx * acy - aby * acx) / ab_len;
        }
    }

    return 0u;
}

uint8 ins_path_follow_is_done(void) { return s_follow_done; }

/* ================================================================
 *  璋冭瘯淇℃伅璇诲彇
 * ================================================================ */
float  ins_path_get_cte(void)           { return s_cte; }
uint16 ins_path_get_nearest_idx(void)   { return s_nearest_idx; }
uint16 ins_path_get_lookahead_idx(void) { return s_lookahead_idx; }
