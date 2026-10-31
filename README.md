# 平衡摩托 · TC264 智能车控制程序

全国大学生智能汽车竞赛 **摩托组** 的整车控制代码，跑在 Infineon **TC264D**
（TriCore 双核）上，基于逐飞 TC264 开源库 + Infineon iLLD。

核心是一套**多速率 PID 姿态平衡 + 航向控制 + 惯导路径跟踪**的实现：车体靠
动量轮维持俯仰平衡，靠舵机控制转向，靠 GNSS / 惯导走完规定赛道。

```
IMU660RX ──SPI──▶ Mahony AHRS (1ms) ──▶ roll / pitch / yaw
                                            │
                          yaw 外环 (8ms) ──▶ 角度环 (4ms) ──▶ 角速度环 (2ms)
                                            │
                                        舵机 P33_9 (300Hz)
```

## 目录结构

```
Seekfree_TC264_Opensource_Library/
├── user/
│   ├── cpu0_main.c            入口：外设初始化 + 主循环（只跑菜单/UI/GNSS）
│   ├── cpu1_main.c            第二个核（空框架）
│   ├── isr.c                  ★ 真正的时间片调度在这里
│   └── isr_config.h           中断优先级配置
│
├── code/                      全部自研代码
│   ├── Ttop_funtion.c/h       顶层调度：模式分派 + 模式一次性初始化
│   ├── common_flie.c/h        多速率 PID、AHRS 任务、舵机输出
│   ├── quaternion_ahrs.c/h    Mahony 四元数姿态解算（6 轴）
│   ├── imu_filter.c/h         IMU 一阶低通
│   ├── motor_controller.c/h   电机速度环（5ms）
│   ├── encoder_count.c/h      TIM2 正交编码器计数与速度换算
│   ├── roll_slope_speed.c/h   坡度补偿：按当前速度修正电机目标
│   ├── small_driver_uart_control.c/h  小驱动板 UART 协议
│   ├── ttop_rc_control.c/h    遥控器通道 → yaw 目标 / 电机目标
│   ├── ins_dead_reckoning.c/h 惯导航迹推算（编码器 + yaw）
│   ├── ins_path_follow.c/h    路径示教录制 + Pure Pursuit 跟踪
│   ├── GPS.c/h  gps_*.c/h     GNSS 解析、航向角、导航、轨迹绘制
│   ├── flash_storage.c/h      DFLASH 参数持久化（PID 表 / 轨迹 / 坡度 / 杂项）
│   ├── Menu.c/h  Key.c/h      IPS200 菜单界面 + 4 键
│   ├── ttop_kemu1/3.c/h       科目路径（7 段 / 8 段）
│   ├── top_8_turn_2.c/h       8 字绕行
│   ├── ttop_speed_change.c/h  变速控制
│   └── top_stop.c/h           里程到点停机
│
├── libraries/                 第三方库（勿改，见 NOTICE.md）
│   ├── zf_common/ zf_components/ zf_device/ zf_driver/   GPLv3
│   └── infineon_libraries/                               Boost 1.0
│
├── .cproject / .project       ADS (Eclipse CDT) 工程文件
├── Lcf_Tasking_Tricore_Tc.lsl TASKING 链接脚本（勿删）
├── 航向角PID计算方式总结.md    航向环三层 PID 的推导
└── 尽量不要使用的引脚.txt      TC264 引脚避坑清单
```

`Debug/` 不在仓库里——ADS 每次编译重新生成，见 `.gitignore`。

## 运行调度

**注意：主循环 `core0_main()` 里只有三个函数**，所有实时任务都在中断里：

```c
while (TRUE) {
    Menu_Select();        // 菜单状态机
    User_Interface();     // IPS200 刷新
    gnss_data_parse();    // GNSS 解析（另一处在中断里）
}
```

真正的调度在 `user/isr.c` 的 `cc61_pit_ch0_isr`，由
`pit_init(CCU61_CH0, 1000)` 以 **1 ms** 触发，内部再做分频：

| 实际周期 | 任务 | 说明 |
|---|---|---|
| **1 ms** | `imu_task_handler_2ms()` | 读 IMU + Mahony 解算，`IMU_TASK_DT_S = 0.001` |
| **2 ms** | `pid_timer_handler_multirate_2ms()` | 内部再分频成 2 / 4 / 8 ms 三层 |
| **5 ms** | `Key_Sample()` | 按键采样 |
| **5 ms** | `ttop_funtion_new()` | 按 `funtion_kemu` 分派到 12 种模式 |
| **5 ms** | `ttop_encoder_task_5ms()` | 编码器 + 坡度补偿 + 电机速度环 |
| **20 ms** | `gps_nav_task_100ms()` | GNSS 导航 |
| 事件 | `gnss_data_parse()` | `gnss_flag` 置位时 |

**上电后约 2 秒才开始跑任务**——`enable_count > 2000` 置 `enable_flag`
之后才进调度，给 IMU 和 GNSS 留出稳定时间。

`pit_init(CCU60_CH1, 2000)` 那个 2 ms 定时器虽然初始化了，但中断函数
`cc60_pit_ch1_isr` 的**函数体是空的**（`imu_task_handler_2ms()` 那行被注释掉了）。

## 姿态与 PID

Mahony 六轴解算在 1 ms 任务里跑，输出 `roll / pitch / yaw` 到缓存。
yaw 是六轴不可观量，用 `ahrs_yaw_get_rel_deg()` 做相对零位。

PID 是**三层多速率**（`code/common_flie.c`）：

| 层 | 周期 | dt 常量 |
|---|---|---|
| 角速度环 rate | 2 ms | `PID_MR_RATE_DT_S = 0.002` |
| 角度环 angle | 4 ms | `PID_MR_ANGLE_DT_S = 0.004` |
| yaw 外环 | 8 ms | `PID_MR_YAW_DT_S = 0.008` |

yaw 外环有个**俯仰门控**：`|pitch| ≤ 30°` 才更新 yaw 目标，
超过就把目标清零（`YAW_GATE_ABS_PITCH_MAX_DEG`）。车快倒的时候不追航向。

舵机输出按 `cfg->strategy` 分三种，不是只有串级：

| 策略 | 结构 |
|---|---|
| `PID_STRAT_CASCADE` | 角度环 → 角速度环（标准双环） |
| `PID_STRAT_ANGLE_ONLY` | 角度环 + `1.5 × 角速度` 前馈 |
| 阻尼（else 分支） | 角度环 + `damp_k × 角速度` |

PID 参数按 **16 组表**（`PID_MODE_CFG_COUNT`）存在 Flash 里，菜单里切换，
上电由 `flash_storage_load_pid_mode_cfg()` 恢复。

## 12 种赛事模式

`funtion_kemu` 由菜单设定，`ttop_funtion_mode_once_apply()` 在模式切换时
执行一次性初始化，`ttop_funtion_new()` 每 5 ms 执行周期任务：

| # | 一次性初始化 | 5ms 周期任务 |
|---|---|---|
| 1 | `CASCADE` | 空（只靠默认电机目标） |
| 2 | `CASCADE_EX3` + 变速 | 变速控制 |
| 3 | `DAMPED_TWO` + 遥控 | 遥控 yaw + 录制 + 回放，回放完自动停 |
| 4 | `DAMPED_THREE` + 遥控 | 同 MODE3，只是 PID 参数不同 |
| 5 | `CASCADE` + kemu1(7 段) + 变速 | 7 段路径 + 变速，跑完自动停 |
| 6 | `DAMPED_TWO` + 8 字 | 8 字绕行（`eight_count > 300` 后启动，约 1.5 s） |
| 7 | kemu3(8 段) + 变速 | 三种子模式：录制 / 回放 / 纯速度 |
| 8 | 遥控 + 变速 | 只取遥控速度通道，yaw 不管 |
| 9 | 遥控 + 变速 + 航迹推算 | 路径示教录制（手动推车记点） |
| 10 | 遥控 + 变速 + 航迹推算 | Pure Pursuit 自动跟踪 |
| 11 | kemu3 + 遥控 + 变速 | 遥控专属 KM3 录制/回放 |
| 12 | `DAMPED_FOUR` | 空（预留） |

MODE9 / MODE10 是一对：9 手动推车把路径录进 Flash，10 按录好的路径自动跑。

## 引脚分配

以下全部来自代码里的宏定义，不是注释：

| 外设 | 引脚 | 出处 |
|---|---|---|
| 舵机 PWM | **P33_9**（ATOM0_CH1，300 Hz） | `common_flie.h:11` |
| 编码器 CH1 / CH2 | **P33_7 / P33_6**（TIM2 正交） | `encoder_count.c:27` |
| 小驱动板 UART | **UART2，460800** | `small_driver_uart_control.h:20` |
| 按键 KEY1~4 | **P13_0 / P13_3 / P13_1 / P13_2** | `Key.h:18-21` |
| LED1~4 | **P20_9 / P20_8 / P21_5 / P21_4** | `gps_key_led.h:15-18` |
| 蜂鸣器 | **P33_10** | `gps_display.h:13` |
| 测试/示波器脚 | P21_2 P21_3 P22_0 P22_1 P22_2 P22_3 | `cpu0_main.c:22-28` |

其中三个脚在任务入口/出口各翻转一次，HIGH 的宽度就是任务耗时，可以直接上
逻辑分析仪量：

| 脚 | 翻转者 | 备注 |
|---|---|---|
| **P21_2** | `ttop_encoder_task_5ms()` | 独占，波形干净 |
| **P22_1** | `imu_task_handler_2ms()` | 独占，波形干净 |
| **P22_2** | `pid_timer_handler_multirate_2ms()` **和** `ttop_funtion_new()` | ⚠ 两个任务共用，波形是叠加的，只能看个大概 |

`P21_3`、`P22_0`、`P22_3` 在 `cpu0_main.c` 里初始化了，但**没有任何地方翻转**，
是空的测试脚。

## 关键参数

| 参数 | 值 | 出处 |
|---|---|---|
| 编码器每米计数 | **18813** | `Ttop_funtion.h:21` `ENCODER_COUNTS_PER_M` |
| 舵机频率 / 脉宽 | 300 Hz / 1000~2000 µs（中位 500） | `common_flie.h:12-14` |
| 小驱动板波特率 | 460800 | `small_driver_uart_control.h:21` |
| IMU 解算周期 | 1 ms（`IMU_TASK_DT_S = 0.001`） | `common_flie.c:42` |
| PID 三层周期 | 2 / 4 / 8 ms | `common_flie.c:31-33` |
| PID 参数组数 | 16 组，存 Flash | `common_flie.h:104` |
| GNSS 最多存点 | 50 个 | `gps_display.h:16` |
| kemu1 / kemu3 段数 | 7 段 / 8 段 | `ttop_kemu1.h:22`、`ttop_kemu3.h:19` |

## 构建

需要 **AURIX Development Studio**（内含 TASKING 工具链）。GCC 编不了——
TC264 的链接脚本和启动代码都是 TASKING 专用格式。

1. ADS → `File → Import → Existing Projects into Workspace`，选本目录
2. 右键工程 → `Build Project`
3. 首次编译会自动生成 `Debug/`（makefile、`.o`、`.elf`），不用管

命令行方式（TASKING 已在 PATH 中时）：

```powershell
cd Debug
make
```

烧录用调试配置 `Seekfree_TC264_Opensource_Library Debug.launch`。

## 命名陷阱（重要）

这个工程迭代了几个月，有些函数名和注释已经跟不上代码了。**以行为为准，别信名字**：

| 名字 | 实际行为 |
|---|---|
| `imu_task_handler_2ms()` | 实际 **1 ms** 调用一次，内部 `dt = 0.001` |
| `gps_nav_task_100ms()` | 实际 **20 ms** 调用一次（`s_gps_nav_div20 >= 20`） |
| `ttop_encoder_task_5ms()` | 确实是 5 ms，但由名为 `period_10ms_count` 的变量驱动 |
| `SMALL_DRIVER_RX` | 定义成 `UART2_TX_P10_5`，**名字和引脚方向相反** |
| `SMALL_DRIVER_TX` | 定义成 `UART2_RX_P10_6`，同上 |
| `pit_init(CCU60_CH1, 2000)` | 2 ms 定时器，但中断函数体是空的 |
| `Debug/` | 不是"调试版本"，是 ADS 的构建输出目录（Release 也放这儿） |

上面这些都在源码里加了注释说明。另外 `ttop_pid_task_5ms()` 只剩一个
被注释掉的声明——全工程没有实现，PID 早就迁到
`pid_timer_handler_multirate_2ms()` 了。

## 踩过的坑

- **`Debug/` 不要提交。** 600 多个文件、80 多 MB，每次编译全变。
- **`.mdf` 和 `makefile` 是自动生成的。** 文件头写着 `Do not edit!`，
  改工程配置请改 `.cproject`，别手改 makefile。
- **`Lcf_Tasking_Tricore_Tc.lsl` 不能删。** 删了链接直接失败。
- **`wifi_spi_tcp_demo.c` 里的凭据是占位的。** 用之前换成你自己的热点。
- **`libraries/` 不要改。** 要封装的接口写在 `code/` 里。
- **`libraries/doc/version.txt` 是 GBK 编码**，GitHub 上看是乱码。
  那是逐飞原版文件，没动过。`code/` 和 `user/` 下全部是 UTF-8。
- **`eight_count` / `eight_flag` 是 MODE6 / MODE9 / MODE10 共用的全局变量**，
  而且从不清零。所以那个「启动前等 1.5 秒」的延时**只在每次上电后第一次生效**，
  之后再切到这三个模式都是立刻启动。想每次都延时，得在
  `ttop_funtion_mode_once_apply()` 里把这两个变量复位。

## 仓库里没有的东西

- `Debug/`：编译产物，见上。
- `日志/` 和参赛技术报告：个人调车记录，留在本地没公开。

## 许可

[GNU GPL v3](LICENSE)。因为 `libraries/` 下的逐飞 TC264 开源库是 GPLv3，
链接它的程序必须同样以 GPLv3 分发。第三方组件见 [NOTICE.md](NOTICE.md)。
