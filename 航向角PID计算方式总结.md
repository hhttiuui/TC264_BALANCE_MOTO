# 航向角 PID 计算方式总结

> 基于 `Seekfree_TC264_Opensource_Library` 项目，分析航向角（Yaw）相关 PID 控制的计算流程与实现细节。

---

## 一、整体架构概览

航向角控制采用 **串级 PID + 多速率调度** 架构，分为三层：

```
┌─────────────────────────────────────────────────────────┐
│                   GPS 导航层 (gps_nav.c)                  │
│  计算目标航向 g_yaw_target_deg，写入全局变量               │
└──────────────────────────┬──────────────────────────────┘
                           │ g_yaw_target_deg
                           ▼
┌─────────────────────────────────────────────────────────┐
│              Yaw 外环 PID (common_flie.c)                 │
│  输入: g_yaw_target_deg - yaw_rel_deg                     │
│  输出: g_yaw_pid_out_deg → pid_target                     │
└──────────────────────────┬──────────────────────────────┘
                           │ pid_target
                           ▼
┌─────────────────────────────────────────────────────────┐
│          Pitch 角度环 PID → 角速度环 PID → 舵机输出         │
│  (或 ANGLE_ONLY / ANGLE_DAMPED 替代策略)                  │
└─────────────────────────────────────────────────────────┘
```

**关键文件：**
| 文件 | 作用 |
|------|------|
| `code/common_flie.c` | 核心 PID 计算、姿态解算、多速率调度 |
| `code/common_flie.h` | PID 模式枚举、配置表结构体定义 |
| `code/gps_nav.c` | GPS 航向计算、坐标系转换、目标航向生成 |
| `code/gps_Azimuth.c` | GPS 两点间航向角几何计算 |
| `code/Ttop_funtion.c` | 顶层赛事模式调度，按模式选择 PID 配置 |

---

## 二、核心 PID 计算函数

### 2.1 `pid_compute()` — 通用 PID 计算

位于 `common_flie.c` 第 175 行，是本项目所有 PID 回路的基础计算单元。

```c
static float pid_compute(pid_t *pid, float setpoint, float measurement, float dt)
```

**计算公式：**

$$e(k) = \text{setpoint} - \text{measurement}$$

**P 项（比例）：**
$$P = K_p \cdot e(k)$$

**I 项（梯形积分，抗饱和）：**
$$I(k) = I(k-1) + \frac{1}{2} \cdot K_i \cdot \Delta t \cdot \big(e(k) + e(k-1)\big)$$

积分项受 `integrator_min` / `integrator_max` 限幅（anti-windup）。

**D 项（基于测量值微分，降低噪声敏感度）：**
$$D = -K_d \cdot \frac{\text{measurement}(k) - \text{measurement}(k-1)}{\Delta t}$$

> 注意：D 项使用 **测量值的负微分**（Derivative on Measurement），而非误差微分。这避免了设定值阶跃变化时 D 项产生尖峰（derivative kick）。

**总输出：**
$$\text{out} = P + I + D$$

输出受 `out_min` / `out_max` 限幅。

**PID 结构体定义：**
```c
typedef struct {
    float kp, ki, kd;           // PID 增益
    float integrator;           // 积分累计值
    float prev_error;           // 上一次误差
    float prev_measurement;     // 上一次测量值
    float out_min, out_max;     // 输出限幅
    float integrator_min, integrator_max; // 积分限幅（anti-windup）
} pid_t;
```

### 2.2 四个 PID 实例

| 实例名 | 用途 | 策略 |
|--------|------|------|
| `g_pid_yaw` | Yaw 外环：航向角 → pitch 角度目标 | 所有模式共用 |
| `g_pid_angle` | Pitch 角度环：角度目标 → 角速度目标 | CASCADE / ANGLE_ONLY |
| `g_pid_rate` | Pitch 角速度环：角速度目标 → 舵机输出 | 仅 CASCADE |
| `g_pid_angle_two` | Pitch 角度环（无角速度环版本） | ANGLE_DAMPED |

---

## 三、Yaw 外环 PID 计算

### 3.1 门控逻辑（Yaw Gate）

Yaw 环设有 **pitch 角度门控**：当小车倾斜（pitch）过大时，关闭 yaw 环以避免侧翻或失控。

```c
static float pid_compute_yaw_target_with_gate(const pid_mode_cfg_t *cfg,
                                               float pitch_deg, float yaw_rel_deg)
```

- **关闭条件：** $|\text{pitch}| > \text{yaw\_gate\_off\_abs\_pitch\_deg}$（如 6.0°）
- **打开条件：** $|\text{pitch}| < \text{yaw\_gate\_on\_abs\_pitch\_deg}$（如 4.0°）
- 门控使用**滞回比较**（施密特触发器），防止在阈值附近频繁开关

实际代码中，门控状态变量 `s_yaw_enable` 目前**未被实际用于屏蔽 PID 输出**（`(void)s_yaw_enable`），但保留了门控开关的逻辑框架。

### 3.2 Yaw PID 计算公式

$$\text{error}_{\text{yaw}} = g\_yaw\_target\_deg - yaw\_rel\_deg$$

$$g\_yaw\_pid\_out\_deg = pid\_compute(g\_pid\_yaw,\ g\_yaw\_target\_deg,\ yaw\_rel\_deg,\ \Delta t)$$

$$pid\_target = g\_yaw\_pid\_out\_deg$$

**默认 Yaw PID 参数（CASCADE 模式）：**

| 参数 | 值 | 说明 |
|------|-----|------|
| $K_p$ | 0.7 | 比例增益 |
| $K_i$ | 0.0 | 积分增益（通常关闭） |
| $K_d$ | 0.0012 | 微分增益 |
| 输出范围 | [-20, 20] deg | yaw 环输出的 pitch 目标角度范围 |
| 积分限幅 | [-10, 10] deg | |

### 3.3 多速率调度下的 Yaw PID

在 `pid_timer_handler_multirate_2ms()` 中，yaw 外环采用 **8ms 周期**（每 4 次 2ms 调用执行 1 次）：

```c
#define PID_MR_YAW_DT_S  (0.008f)   // yaw 外环：8ms
```

此外还有一个简化的门控：当 $|\text{pitch}| > 30°$ 时，直接将 `s_yaw_target_hold` 清零。

---

## 四、三种控制策略

### 4.1 CASCADE（串级 PID）

**链路：** Yaw PID → Angle PID → Rate PID → 舵机

```
yaw_target ──→ [Yaw PID] ──→ pid_target ──→ [Angle PID] ──→ rate_sp ──→ [Rate PID] ──→ servo
                ↑ yaw_rel                        ↑ pitch_deg                  ↑ pitch_rate_dps
```

**CASCADE 模式默认参数（PID_MODE_CASCADE）：**

| 回路 | $K_p$ | $K_i$ | $K_d$ | 输出范围 | 积分限幅 |
|------|-------|-------|-------|----------|----------|
| Yaw | 0.7 | 0.0 | 0.0012 | [-20, 20] | [-10, 10] |
| Angle | 18.0 | 0.0 | 0.0008 | [-700, 700] | [-150, 150] |
| Rate | 1.8 | 0.0 | 0.0035 | [-600, 600] | [-100, 100] |

调用周期：5ms（`PID_TASK_DT_S = 0.005f`），或通过多速率调度 2ms/4ms/8ms。

### 4.2 ANGLE_ONLY（仅角度环）

**链路：** Yaw PID → Angle PID + 角速度前馈 → 舵机

```
yaw_target ──→ [Yaw PID] ──→ pid_target ──→ [Angle PID] ──→ + ──→ servo
                ↑ yaw_rel                        ↑ pitch_deg    ↑
                                                        1.5 × pitch_rate_dps (前馈)
```

- 去掉角速度环 PID，用固定系数 1.5 的角速度前馈替代
- 适用于对响应速度要求不高的低速场景

**ANGLE_ONLY 模式默认参数（PID_MODE_ANGLE）：**

| 回路 | $K_p$ | $K_i$ | $K_d$ | 输出范围 |
|------|-------|-------|-------|----------|
| Yaw | 0.25 | 0.0 | 0.0012 | [-5, 5] |
| Angle | 90.0 | 0.0 | 0.0028 | [-600, 600] |

### 4.3 ANGLE_DAMPED（角度环 + 阻尼）

**链路：** Yaw PID → Angle PID + 阻尼项 → 舵机

```
yaw_target ──→ [Yaw PID] ──→ pid_target ──→ [Angle PID] ──→ + ──→ [限幅] ──→ servo
                ↑ yaw_rel                        ↑ pitch_deg    ↑
                                                    damp_k × pitch_rate_dps (阻尼)
                                                    (限幅 ±damp_max)
```

- 角速度作为**阻尼项**叠加到角度环输出上
- 阻尼项限幅：$|\text{damp}| \le \text{damp\_max}$
- 适用于需要抑制振荡的高速场景

**DAMPED_TWO 模式默认参数（PID_MODE_DAMPED_TWO）：**

| 回路 | $K_p$ | $K_i$ | $K_d$ | 输出范围 |
|------|-------|-------|-------|----------|
| Yaw | 0.25 | 0.0 | 0.0002 | [-3.5, 3.5] |
| Angle | 195.0 | 0.0 | 0.0028 | [-700, 700] |
| damp_k | 4.5 | — | — | — |
| damp_max | 120.0 | — | — | — |

---

## 五、辅助处理函数

### 5.1 死区函数 `apply_deadzone()`

```c
static inline float apply_deadzone(float x, float dz)
```

$$f(x) = \begin{cases} x - dz & x > dz \\ x + dz & x < -dz \\ 0 & |x| \le dz \end{cases}$$

- **角速度死区** `GYRO_RATE_DEADZONE_DPS`：抑制陀螺仪零点附近的噪声（默认 0.0 deg/s）
- **舵机输出死区** `SERVO_OUT_DEADZONE`：抑制舵机零点附近的抖动（默认 2.0）

### 5.2 姿态解算（IMU 任务，2ms）

在 `imu_task_handler_2ms()` 中，每个 2ms 周期执行：

1. 读取 IMU 加速度计 + 陀螺仪原始数据
2. **Mahony AHRS** 6 轴姿态解算 → roll / pitch / yaw（欧拉角）
3. Yaw 相对化处理：`ahrs_yaw_get_rel_deg()` + `ahrs_yaw_rel_unwrap_720_update_deg()` → ±720° 连续角
4. Pitch 零点校正 + 低通滤波（可配置 α）
5. 角速度 = `-gyro_y`（安装方向），经死区处理
6. 写入全局缓存供 PID 任务读取

### 5.3 舵机输出 `servo_output_cb()`

```c
CENTER = 500
PWM = CENTER + round(out)      // out 为正 → 右转，out 为负 → 左转
```

PWM 频率 300Hz，脉宽 1000~2000µs 对应位置 0~1000。

---

## 六、GPS 导航层航向计算

### 6.1 GPS 航向角几何计算 (`gps_Azimuth.c`)

使用 WGS84 球面几何公式，根据两点经纬度计算正北基准航向角（0~360°）：

$$\Delta\lambda = (\text{lon}_2 - \text{lon}_1) \cdot \frac{\pi}{180}$$

$$y = \sin(\Delta\lambda) \cdot \cos(\text{lat}_2)$$

$$x = \cos(\text{lat}_1) \cdot \sin(\text{lat}_2) - \sin(\text{lat}_1) \cdot \cos(\text{lat}_2) \cdot \cos(\Delta\lambda)$$

$$\theta = \text{atan2}(y, x) \cdot \frac{180}{\pi}$$

结果归一化到 $[0, 360)$。

### 6.2 坐标系转换 (`gps_nav.c`)

GPS 航向（正北基准，顺时针正）→ IMU 航向（上电前方基准，逆时针正）：

$$\text{imu\_yaw} = -(\text{gps\_heading} - \text{gps\_heading\_0})$$

归一化到 $[-180°, +180°)$。

### 6.3 导航目标航向生成

`gps_nav_task_100ms()`（10Hz）中：

1. **校准阶段**：小车直线行驶，GPS 根据运动方向自动确定 `heading_0`
2. **导航阶段**：
   - 计算当前位置到目标点的 GPS 航向
   - 坐标系转换 → IMU 目标航向
   - 计算航向偏差：$\Delta = \text{wrap\_180}(\text{imu\_target} - \text{show\_yaw\_deg})$
   - 死区过滤：$|\Delta| < \text{deadzone}$ → 置零
   - **限速平滑**（Slew Rate Limiter）：防止目标航向突变
   - 写入 `g_yaw_target_deg`

### 6.4 航向漂移校正

利用 GPS 运动航向与 IMU 估计航向的偏差，逐步修正 `heading_0`：

$$\text{drift\_error} = \text{wrap\_180}(\text{gps\_motion\_heading} - \text{imu\_heading\_est})$$

$$\text{correction} = \text{clamp}(\text{drift\_error} \times \text{gain},\ \pm\text{max\_step})$$

$$\text{heading\_0} \leftarrow \text{heading\_0} - \text{correction}$$

---

## 七、PID 配置表

所有 PID 参数集中在 `s_pid_mode_cfg[]` 配置表中，按 `pid_mode_t` 枚举索引：

| 模式 | 策略 | 典型速度 | 说明 |
|------|------|----------|------|
| `PID_MODE_CASCADE` (0) | CASCADE | 200 counts/5ms | 高速串级 PID |
| `PID_MODE_ANGLE` (1) | ANGLE_ONLY | 100 | 低速仅角度环 |
| `PID_MODE_DAMPED_TWO` (2) | ANGLE_DAMPED | 55 | 中速阻尼控制 |
| `PID_MODE_DAMPED_THREE` (3) | ANGLE_DAMPED | 45 | 低速阻尼控制 |
| `PID_MODE_DAMPED_FOUR` (4) | ANGLE_DAMPED | 45 | 预留 |
| `PID_MODE_CASCADE_EX1~7` (5~11) | CASCADE | 200 | CASCADE 扩展参数组 |

配置表结构体 `pid_mode_cfg_t` 包含：
- 控制策略 `strategy`
- AHRS Mahony 增益 `ahrs_kp` / `ahrs_ki`
- Yaw 门控阈值 `yaw_gate_off/on_abs_pitch_deg`
- 三组 PID 参数 `yaw` / `angle` / `rate`
- 阻尼参数 `damp_k` / `damp_max`
- IMU 滤波系数、舵机初值、电机目标速度

---

## 八、关键设计特点总结

1. **串级 PID 架构**：外环（Yaw）→ 中环（Angle）→ 内环（Rate），逐级提高响应带宽
2. **D 项基于测量值**：避免设定值阶跃时的 derivative kick
3. **梯形积分 + 独立积分限幅**：防止积分饱和（anti-windup），积分限幅与输出限幅独立
4. **Yaw 门控（滞回比较）**：pitch 过大时关闭 yaw 环，防止侧翻
5. **多速率调度**：Yaw 8ms、Angle 4ms、Rate 2ms，匹配各回路带宽需求
6. **死区处理**：角速度死区 + 舵机输出死区，抑制噪声和抖动
7. **限速平滑（Slew Rate Limiter）**：防止 GPS 导航目标航向突变
8. **配置表驱动**：所有参数集中管理，支持热切换（`PID_HOT_SWITCH_BY_MODE`）
9. **GPS 漂移校正**：利用 GPS 运动航向持续修正 IMU yaw 漂移
10. **$K_i$ 默认为 0**：当前所有模式均关闭积分项，纯 PD 控制

---

## 九、PID 模式初始化与切换流程

```
Ttop_funtion.c: ttop_funtion_mode_once_apply()
  │
  ├─ funtion_kemu == 1  →  PID_INIT_BY_MODE(PID_MODE_CASCADE)
  ├─ funtion_kemu == 2  →  PID_INIT_BY_MODE(PID_MODE_CASCADE_EX3)
  ├─ funtion_kemu == 3  →  PID_INIT_BY_MODE(PID_MODE_DAMPED_TWO)
  ├─ funtion_kemu == 4  →  PID_INIT_BY_MODE(PID_MODE_DAMPED_THREE)
  └─ ...
         │
         ▼
  common_flie.c: PID_INIT_BY_MODE(mode)
    1. 查表获取 pid_mode_cfg_t
    2. 初始化 Mahony AHRS (kp, ki)
    3. pid_apply_gains() 写入 yaw/angle/rate PID 参数
    4. 初始化 IMU 滤波器
    5. 清零 yaw 输出、pid_target
    6. 设置舵机初始位置、电机目标速度
```

---

*文档生成日期：2026-07-03*
