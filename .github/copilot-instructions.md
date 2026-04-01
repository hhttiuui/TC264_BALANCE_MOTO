本仓库为基于 Infineon TC264 平台的嵌入式示例库（GPLv3）。下面的说明针对 AI 编码助手，帮助快速定位、修改与构建代码。

主要目标
- 代码运行在 TC264（Tricore）硬件，使用 TASKING (cctc) 工具链；请勿尝试在标准 x86/Linux 环境直接运行或假设 POSIX API 可用。
- 保持对供应商 iLLD 与 `libraries/` 下驱动代码的最小改动；项目结构依赖这些库的编译单元与链接设置。

关键位置（快速定位）
- 入口与主循环：`user/cpu0_main.c`（CPU0 主程序，初始化外设并调用周期任务）。
- 另一个核心：`user/cpu1_main.c`（如果需要处理双核协作，应查阅该文件与 `isr.c`）。
- 应用碼：`code/` 目录下为主要功能模块，例如 `motor_controller.c`、`quaternion_ahrs.c`、`imu_filter.c`。
- 构建脚本与工程文件：`Debug/makefile`（自动生成）、`Seekfree_TC264_Opensource_Library.mdf`（TASKING 工程）。

大体架构与运行时约定
- 多核：代码分布在 `user/`（启动/入口）和 `code/`（功能模块）；中断/定时器驱动多数周期任务。
- 周期任务命名约定：多数模块使用 `*_init()` 初始化，周期执行函数以 `*_task_<ms>()` 命名（例如 `motor_control_task_5ms`、`led_bzw_show_task_20ms`），修改时请保持此约定。
- demo/开关：带 `*_demo` 或 `*_flag` 的变量常用于示例或临时代码（例如 `wifi_spi_demo_flag`、`wifi_ok`），AI 不应把 demo 代码误认为生产逻辑。

构建与调试（可执行指令示例）
- 构建依赖 TASKING 工具链（cctc、elfsize 等）和 make 在 PATH 中。常用方法：在项目根或 `Debug` 目录下运行 make：

  # 在 PowerShell 中（假设 TASKING 命令在 PATH）
  cd Debug; make

- 或打开 `Seekfree_TC264_Opensource_Library.mdf` 在 TASKING/ADS IDE 中构建。链接使用 `Lcf_Tasking_Tricore_Tc.lsl`，请勿删除该文件。
- 注意：本仓库没有通用的 host-side单元测试；修改后请在目标硬件或仿真器上进行运行验证。

代码改动注意事项（AI 需遵守）
- 切勿替换或重写 `libraries/infineon_libraries` 下的供应商文件；如果需要改动，优先在 `code/` 或 `user/` 中封装/重定向接口。
- 保持中断与定时器相关函数的签名与频率。许多算法（例如 AHRS）默认按 2ms / 5ms 调用；修改调用频率时须同时调整相关计数/阈值（见 `quaternion_ahrs.c` 中关于 2ms 的说明）。
- 命名与数值常量多为宏或全局变量（例如 PID 参数、滤波 alpha）。若改动全局常量，尽量把变更放在单独的 setter/初始化函数以便回退。

集成点与外设
- 显示：`ips200_*` 函数用于屏幕（例：`ips200_init` 在 `cpu0_main.c` 中被调用）。
- 电机：`small_driver_*` 命名的接口控制驱动（例：`small_driver_set_duty`），速度由 `encoder_*` 读取。
- IMU：`imu660rb` 相关头/源被 AHRS 算法直接引用（`zf_device_imu660rb.h`、`quaternion_ahrs.c`）。
- 无线：`zf_device_wifi_spi` 提供 WiFi-over-SPI 的演示/通信，使用 `wifi_spi_demo_flag` 控制发送示例。

示例引用（便于 AI 生成更精确改动）
- 在 `user/cpu0_main.c` 中：初始化定时器 `pit_init(CCU61_CH0,5000)`（用于舵机 PID/编码器），以及 `pit_init(CCU60_CH1,2000)`（2ms 姿态解算）。
- 在 `code/motor_controller.c` 中：主控制周期函数为 `motor_control_task_5ms()`，PID 计算封装于 `motor_pid_compute`。
- 在 `code/quaternion_ahrs.c` 中：Mahony AHRS 的实现带有自动置零与 yaw unwrap，注意默认假设 2ms 调用频率（见注释）。

限制与风险
- 本仓库使用 GPLv3；生成或修改代码时注意许可证合规（保留版权/许可声明）。
- 编译/烧录需要厂商工具与硬件；AI 不要尝试提供未验证的 flashing 命令或修改 boot/链接脚本风险文件。

如何反馈
- 我已根据仓库代码生成此指南；如果你希望我把某个模块的实现细节（如 `wifi_spi` 或 `encoder`）加入示例片段，告诉我具体文件或场景，我会更新并合并进本文件。

（备注）仓库中没有发现现成的 `.github/copilot-instructions.md`，因此本文件为新建。若要合并其他人工编写的 agent 指南，请指出路径。
