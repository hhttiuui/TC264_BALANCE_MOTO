# 平衡摩托 · TC264 智能车控制程序

全国大学生智能汽车竞赛 **摩托组** 的整车控制代码，跑在 Infineon **TC264D**
（TriCore 双核）上，基于逐飞 TC264 开源库 + Infineon iLLD。

核心是一套**串级 PID 姿态平衡 + 航向控制 + 惯导路径跟踪**的实现：车体靠
一个动量轮维持俯仰平衡，靠舵机控制转向，靠 GPS / 惯导走完规定赛道。

```
IMU660RX ──SPI──▶ Mahony AHRS ──▶ roll/pitch/yaw
                                     │
                              串级 PID（航向→俯仰→角速度）
                                     │
                    ┌────────────────┴────────────────┐
                 舵机 PWM                        电机 UART
                (P33_9, 300Hz)                (P10_5/6, 460800)
```

## 目录结构

```
Seekfree_TC264_Opensource_Library/
├── user/
│   ├── cpu0_main.c            入口：外设初始化 + 主循环
│   ├── cpu1_main.c            第二个核（当前为空框架）
│   └── isr.c / isr_config.h   中断服务与中断配置
│
├── code/                      全部自研代码（约 15000 行）
│   ├── Ttop_funtion.c/h       顶层调度器：12 种赛事模式切换
│   ├── common_flie.c/h        PID 初始化、舵机 PWM、PID 模式枚举
│   ├── quaternion_ahrs.c/h    Mahony 四元数姿态解算（2ms）
│   ├── imu_filter.c/h         IMU 一阶低通
│   ├── motor_controller.c/h   电机速度环（5ms）
│   ├── encoder_count.c/h      编码器计数与速度换算
│   ├── small_driver_uart_control.c/h  小驱动板 UART 协议
│   ├── ins_dead_reckoning.c/h 惯导航迹推算（编码器 + yaw）
│   ├── ins_path_follow.c/h    路径示教录制 + Pure Pursuit 跟踪
│   ├── GPS.c/h  gps_*.c/h     GPS 解析、航向角、导航、轨迹绘制
│   ├── flash_storage.c/h      DFLASH 参数持久化（带 CRC）
│   ├── Menu.c/h  Key.c/h      IPS200 菜单界面 + 按键
│   └── ttop_*.c/h  top_*.c/h  各赛事模式的独立实现
│
├── libraries/                 第三方库（勿改，见 NOTICE.md）
│   ├── zf_common/  zf_components/  zf_device/  zf_driver/   GPLv3
│   └── infineon_libraries/                                 Boost 1.0
│
├── .cproject / .project       ADS (Eclipse CDT) 工程文件
├── Lcf_Tasking_Tricore_Tc.lsl TASKING 链接脚本（勿删）
├── 航向角PID计算方式总结.md    航向环三层 PID 的详细推导
└── 尽量不要使用的引脚.txt      TC264 引脚避坑清单
```

`Debug/` 不在仓库里——那是 ADS 每次编译重新生成的产物，见 `.gitignore`。

## 硬件平台

| 项目 | 型号 / 说明 |
|---|---|
| MCU | Infineon TC264D（TriCore，双核，200 MHz） |
| 开发环境 | AURIX Development Studio（TASKING VX-toolset for TriCore） |
| 姿态 | 逐飞 IMU660RX（SPI） |
| 显示 | IPS200 屏幕（SPI） |
| 驱动 | 逐飞「小驱动板」无刷电机驱动（UART2，460800） |
| 定位 | TAU1201 GNSS 模块（UART0） |
| 转向 | 舵机，300 Hz PWM |
| 无线 | WiFi-SPI 模块（可选，`code/wifi_spi_tcp_demo.c`） |

## 引脚分配

| 外设 | 引脚 | 备注 |
|---|---|---|
| 舵机 PWM | P33_9（ATOM0_CH1） | 300 Hz，脉宽 1000~2000 µs |
| 编码器 A / B | P33_7 / P33_6（TIM2） | |
| 小驱动板 TX / RX | P10_5 / P10_6（UART2） | 460800 bps |
| GPS TX / RX | P14_0 / P14_1（UART0） | 初始化在 `Ttop_funtion.c:285` 里是**注释状态**，按需打开 |
| 按键 KEY1~4 | P13_0 / P13_3 / P13_1 / P13_2 | |
| 蜂鸣器 | P33_10 | GPS 提示音 |

> ⚠ **P10_5 / P10_6 是 TC264 的 boot 相关引脚**，`尽量不要使用的引脚.txt`
> 里明确列了出来。本项目把小驱动板 UART2 放在这两个脚上，实测可用，
> 但上电瞬间如果被外部拉低可能进不了程序。复刻时建议挪到别的 UART。

## 任务调度

1 ms 硬件定时器打底，切成 5 ms 时间片：

| 时间槽 | 任务 | 函数 | 实测耗时 |
|---|---|---|---|
| 0~1 ms | 编码器更新 | `ttop_encoder_task_5ms()` | ~135 µs |
| 1~2 ms | 按键采样 | `ttop_key_task_5ms()` | ~2 µs |
| 2~3 ms | 姿态解算 | `imu_task_handler_2ms()` | ~26 µs |
| 3~4 ms | PID 计算 | `ttop_pid_task_5ms()` | ~20 µs |
| 4~5 ms | 预留 | — | — |

菜单刷新、UI 绘制、GPS 解析放在 `while(TRUE)` 主循环里，不占实时时间片。

## 控制链路

```
目标航向 yaw_target
      ↓
  [Yaw PID]      航向环（外环）→ 输出俯仰目标
      ↓
  [Pitch PID]    角度环（外环）→ 输出角速度目标
      ↓
  [Rate PID]     角速度环（内环）→ 输出舵机 PWM
      ↓
    舵机 P33_9

  [Speed PID]    速度环（独立）→ 输出占空比 → 小驱动板 → 电机
```

三层 PID 的详细推导、每层为什么这么串、参数怎么整定，见
[`航向角PID计算方式总结.md`](航向角PID计算方式总结.md)。

## 12 种赛事模式

按键切换 `funtion_kemu` 变量：

| # | 模式 | # | 模式 |
|---|---|---|---|
| 1 | 直线 200（串级 PID） | 7 | kemu3 固定 5 段路径 |
| 2 | 变速（CASCADE_EX2） | 8 | 遥控器控制 |
| 3 | 直线 55（阻尼 PID 组 2） | 9 | 惯导 — 示教录制 |
| 4 | 直线 45（阻尼 PID 组 3） | 10 | 惯导 — Pure Pursuit 回放 |
| 5 | 直线 200 + 转弯 | 11 | RC 预留 |
| 6 | 8 字绕行 | 12 | 预留 |

惯导模式 9/10 是一对：9 手动推车把路径录进 Flash，10 自动按录好的路径跑。

## 关键参数

| 参数 | 值 |
|---|---|
| 编码器每米计数 | 18813 |
| 轮周编码器计数 | 2813（≈16 cm） |
| 舵机频率 / 脉宽 | 300 Hz / 1000~2000 µs |
| 小驱动板波特率 | 460800 |
| 速度环 + 平衡环周期 | 5 ms |
| AHRS 更新周期 | 2 ms |

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

烧录用 ADS 的调试配置 `Seekfree_TC264_Opensource_Library Debug.launch`。

## 踩过的坑

写在这里免得别人重踩：

- **`Debug/` 不要提交。** 600 多个文件、80 多 MB，每次编译全变。
- **`.mdf` 和 `makefile` 是自动生成的。** 文件头写着 `Do not edit!`，
  改工程配置请改 `.cproject`，别手改 makefile。
- **`Lcf_Tasking_Tricore_Tc.lsl` 不能删。** 删了链接直接失败。
- **`wifi_spi_tcp_demo.c` 里的凭据是占位的。** 用之前换成你自己的热点，
  别把真实密码提交上来。
- **`quaternion_ahrs.c` 默认按 2 ms 调用。** 改调用周期要同步改里面的
  积分系数和阈值，否则姿态会飘。
- **`libraries/` 不要改。** 要封装的接口写在 `code/` 里，别动供应商代码。
- **`libraries/doc/version.txt` 是 GBK 编码**，在 GitHub 上看是乱码。
  那是逐飞原版文件，没动过。`code/` 和 `user/` 下的源文件全部是 UTF-8。

## 仓库里没有的东西

- `Debug/`：编译产物，见上。
- `日志/` 和参赛技术报告：属于个人调车记录，留在本地没公开。
- 逐飞库的 BDF 源字库之类的大文件：本来就不在本工程里。

如果你想参考调车过程，README 里的调度表、控制链路和
[`航向角PID计算方式总结.md`](航向角PID计算方式总结.md) 已经把设计讲完了。

## 许可

[GNU GPL v3](LICENSE)。因为 `libraries/` 下的逐飞 TC264 开源库是 GPLv3，
链接它的程序必须同样以 GPLv3 分发。

第三方组件的具体许可见 [NOTICE.md](NOTICE.md)。
