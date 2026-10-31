#include "zf_common_headfile.h"
#include "quaternion_ahrs.h"
#include "Ttop_funtion.h"
#include "flash_storage.h"

#pragma section all "cpu0_dsram"

uint32 count_imu;

#include "zf_device_wifi_spi.h"
#define IPS200_TYPE     (IPS200_TYPE_SPI)

int core0_main(void)
{
    clock_init();                   // 获取并设置系统时钟
    debug_init();                   // 璋冭瘯妯″潡鍒濆鍖栵紙涓插彛/鏃ュ織绛夛級
    system_start();
    gnss_init(TAU1201);
    system_delay_ms(500);

    ttop_funtion_init();
    gpio_init(P21_2, GPO, GPIO_HIGH, GPO_PUSH_PULL);//测试
    gpio_init(P21_3, GPO, GPIO_HIGH, GPO_PUSH_PULL);//测试

    gpio_init(P22_0, GPO, GPIO_HIGH, GPO_PUSH_PULL);//测试
    gpio_init(P22_1, GPO, GPIO_HIGH, GPO_PUSH_PULL);//暂定pid调控指示引脚
    gpio_init(P22_2, GPO, GPIO_HIGH, GPO_PUSH_PULL);//暂定主周期指示引脚
    gpio_init(P22_3, GPO, GPIO_HIGH, GPO_PUSH_PULL);//测试

    cpu_wait_event_ready();

    while (TRUE)
    {
        //test
        //test
        Menu_Select();
        User_Interface();
        gnss_data_parse();
        //ips200_show_string(0, 0 * 15, "KOU_KOU_FLY_CAR");
        //led_bzw_show_task_20ms_auto();// LED 状态显示 + 更新标志位

        
    }

}

#pragma section all restore
// ********************************************************
//现在的任务触发：姿态解算->2ms中断触发
//               编码器+按键+pid+按键切换后的初始化 -> while里5ms等待触发

/* 
现在的调控方式：




用1ms定时器，以5ms为周期，分别触发：
0ms ~ 1ms -> 编码器更新
1ms ~ 2ms -> 按键
2ms ~ 3ms -> 姿态解算
3ms ~ 4ms -> pid计算
4ms ~ 5ms -> 留空
鑿滃崟鎿嶄綔鏀惧湪涓诲惊鐜腑



已使用引脚：
舵机 -> 33_9

UART_2_TX -> P10_5（UART2_TX_P10_5）
UART_2_RX -> P10_6（UART2_RX_P10_6）
UART_3_TX -> P15_6（注意：宏名写的是 SMALL_DRIVER_RX (UART3_TX_P15_6)）
UART_3_RX -> P15_7（注意：宏名写的是 SMALL_DRIVER_TX (UART3_RX_P15_7)）

KEY1 -> P11_3
KEY2 -> P11_2
KEY3 -> P20_7

LED1 -> P20_9
LED2 -> P20_8
LED3 -> P21_5
LED4 -> P21_4

编码器 A 相 / CH1 -> P33_7（TIM2_ENCODER_CH1_P33_7）
编码器 B 相 / CH2 -> P33_6（TIM2_ENCODER_CH2_P33_6）


函数耗时：
ttop_encoder_task_5ms();//编码器-> 135.26us
ttop_key_task_5ms();    //按键-> 2.06us
imu_task_handler_2ms(); //姿态解算-> 25.84us
ttop_pid_task_5ms();    //pid计算-> 19.62us


*/
