# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

Raspberry Pi Pico 2 (RP2350) 机器狗 — 基于 MicroPython 的四足机器人，使用 sensor-driven AFSM (Augmented Finite State Machine) 控制器。课程：4FTC2135-0105-2025 Robot Design and Build Project B，Assignment 2（占总分 50%）。

## 开发环境

- **IDE:** Thonny（连接 Pico，直接在设备上编辑/保存文件）
- **语言:** MicroPython（非 CPython — 标准库受限，无 `asyncio`，无 pip）
- **入口点:** 保存为 Pico 上的 `main.py`，上电自动运行
- **仿真器:** Wokwi (wokwi.com)，无需硬件即可测试
- **参考代码:** `reference/` 目录存放课件中的各组件测试脚本；最终实现在项目根目录
- **无构建系统、无包管理器、无虚拟环境。**

## Pin 映射

| 组件 | Pin | 备注 |
|---|---|---|
| RR_Servo（右后腿） | 未定义 | PWM, 50Hz |
| LR_Servo（左后腿） | 未定义 | PWM, 50Hz |
| RF_Servo（右前腿） | 未定义 | PWM, 50Hz |
| LF_Servo（左前腿） | 未定义 | PWM, 50Hz |
| US_Servo（超声波云台） | 未定义 | PWM, 50Hz |
| LDR（光敏传感器） | 未定义 | ADC，外接 10kΩ 下拉电阻 |
| Ultrasonic Trig | 未定义 | 数字输出 |
| Ultrasonic Echo | 未定义 | 数字输入 |
| TCS3200 S2 | 未定义 | |
| TCS3200 S3 | 未定义 | |
| TCS3200 OUT | 未定义 | |
| TCS3200 LED | 未定义 | |
| Status LED | 未定义 | 板载 LED 或外接 |

## AFSM 架构

整个控制器分为三层：**Mission States**（顶层状态机）、**Parallel Sub-States**（并行子状态，在 Explore/Low Charge 期间持续运行）、**Obstacle Avoidance Sub-States**（避障子状态，由 Scan 触发）。

---

### 1. Mission States（顶层状态机）

```
                    ┌─────────────────────────────────────┐
                    │                                     │
                    ▼                                     │
              ┌──────────┐    init_complete    ┌─────────┐│
              │  Start   │ ──────────────────► │ Explore ││
              └──────────┘                    └─────────┘│
                                                  │  │   │
                       ┌──────────────────────────┘  │   │
                       │ explore_timer >= 2 min       │   │
                       ▼                              │   │
                 ┌────────────┐    red_detected       │   │
                 │ Low Charge │ ──────────┐           │   │
                 └────────────┘           │           │   │
                      │    │              │           │   │
                      │    │ red_detected │           │   │
   LDR > threshold    │    └──────────────┼─────┐     │   │
   ┌──────────────────┘                   │     │     │   │
   ▼                                      ▼     ▼     │   │
┌──────────┐                       ┌────────────────┐  │   │
│ Charging │                       │Target Detected │  │   │
└──────────┘                       └────────────────┘  │   │
   │    │                              (终止状态)       │   │
   │    │ light_interrupted                             │   │
   │    └──────────────► Low Charge                     │   │
   │                                                   │   │
   │    charging_timer >= 1 min & light_present         │   │
   └────────────────────► Explore (重置 explore_timer)  │   │
                                                       │   │
                                                       │   │
```

**各状态描述：**

| State | 描述 |
|---|---|
| **Start** | 初始化所有硬件：腿部舵机、云台舵机、ultrasonic sensor (HC-SR04)、colour sensor (TCS3200)、light sensor (LDR)、status LED。云台归中，重置所有 timer。完成后转入 Explore。 |
| **Explore** | 正常探索模式，全速行走。并行运行 Forward（全速）+ Scan（常开）。TCS3200 持续检测红色目标；explore_timer 入状态时启动（2 min）。 |
| **Low Charge** | 探索 2 分钟后未找到红色目标进入此状态。Forward 减速，Scan 降低扫描频率。LDR 和 TCS3200 保持活动。 |
| **Charging** | 模拟充电：停止所有运动，Scan 和 Forward 均不活动，status LED 常亮。LDR 持续监测；charging_timer 入状态时启动（1 min）。 |
| **Target Detected** | 最高优先级终止状态。TCS3200 检测到红色：立即停止，status LED 以 500ms 间隔闪烁（led_blink_timer）。无出边转换。 |

**Mission State 转换表：**

| Source | Target | Condition | 说明 |
|---|---|---|---|
| Start | Explore | `init_complete` | 硬件就绪，所有 timer 重置 |
| Explore | Low Charge | `explore_timer >= 2 min` | 探索超时，未找到红色目标 |
| Explore | Target Detected | `red_detected` | TCS3200 检测到红色 |
| Low Charge | Charging | `LDR > threshold` | 检测到强光，开始模拟充电 |
| Low Charge | Target Detected | `red_detected` | 低电量模式下检测到红色目标 |
| Charging | Explore | `charging_timer >= 1 min && light_present` | 持续光照下充电完成，重置 explore_timer |
| Charging | Low Charge | `light_interrupted` (charging_timer < 1 min) | 未满 1 分钟光照中断 |

---

### 2. Parallel Sub-States（并行子状态）

仅在 **Explore** 和 **Low Charge** 中激活，在 Charging 中不活动。

| Sub-State | 激活条件 | 描述 |
|---|---|---|
| **Scan** | Explore / Low Charge 中始终开启 | Ultrasonic sensor (HC-SR04) 在云台上测量三个方向：**FD (90°)**、**LD (30°)**、**RD (150°)**。由 scan_timer 驱动周期性扫描。Low Charge 中 scan_timer 周期变慢。 |
| **Forward** | Explore / Low Charge 的默认 entry | 四足行走步态，由 gait_step_timer 驱动，优先级最低。Explore 中全速，Low Charge 中减速。Scan 检测到障碍时被抢占。 |

---

### 3. Obstacle Avoidance Sub-States（避障子状态）

由 **Scan** 根据距离读数触发。优先级：**Recovery > Turn > Veer**。

**距离判定阈值：**

- FD >= 15 cm → 安全，维持 Forward
- 10 cm <= FD < 15 cm → 轻微障碍，触发 Veer（边行进边调整）
- FD < 10 cm → 严重障碍，触发 Turn（先停止再转弯）

**子状态描述：**

| Sub-State | 触发条件 | 行为 |
|---|---|---|
| **Turn Left** | FD < 10 cm && LD > RD | 完全停止 → 原地左转 90°（调整 gait_step_timer）。完成后 → Scan 重新评估。若路径仍阻塞，turn_count 递增。只有 Turn 可触发 Recovery（turn_count >= 3）。 |
| **Turn Right** | FD < 10 cm && RD > LD | 同上，方向相反。 |
| **Veer Left** | 10 cm <= FD < 15 cm && LD > RD | 不停止，行进中轻微左转 15°–30°（调整 gait_step_timer）。完成后 → Scan。Veer 不计数，不触发 Recovery。 |
| **Veer Right** | 10 cm <= FD < 15 cm && RD > LD | 同上，方向相反。 |
| **Recovery** | (LD<10 && RD<10 && FD<10) 或 LD<5 或 RD<5 或 FD<5 或 turn_count >= 3 | 卡住脱困序列：停止 → 后退约 0.8s（gait_step_timer）→ 随机转向。完成后 → Scan 重新评估。recovery_count >= 3 → Fault Stop（等候人工干预）。 |
| **Fault Stop** | recovery_count >= 3 | 完全停止，等待人工干预。 |

**Obstacle Avoidance 转换表：**

| Source | Target | Condition |
|---|---|---|
| Scan | Forward | FD >= 15 cm |
| Scan | Turn Left | FD < 10 cm && LD > RD |
| Scan | Turn Right | FD < 10 cm && RD > LD |
| Scan | Veer Left | 10 cm <= FD < 15 cm && LD > RD |
| Scan | Veer Right | 10 cm <= FD < 15 cm && RD > LD |
| Scan | Recovery | LD<10 && RD<10 && FD<10，或 LD<5 或 RD<5 或 FD<5，或 turn_count >= 3 |
| Turn Left / Right | Scan | 转弯完成，重新扫描 |
| Turn Left / Right | Recovery | turn_count >= 3 |
| Veer Left / Right | Scan | 微调完成，重新扫描 |
| Recovery | Scan | recovery 完成（后退 + 随机转向完毕），重新评估 |
| Recovery | Fault Stop | recovery_count >= 3 |

**注意：** LD == RD 时随机选择方向。只有 Turn → Recovery（通过 turn_count），Veer 不能触发 Recovery。

---

### 4. 优先级与 Timer

**优先级表：**

| 优先级 | 行为 | 涉及 State | 备注 |
|---|---|---|---|
| 1（最高） | Target Detection | Target Detected | 抢占所有其他行为 |
| 2 | Obstacle Avoidance（内部: Recovery > Turn > Veer） | Scan, Turn L/R, Veer L/R, Recovery | 仅在 Explore / Low Charge 中生效 |
| 3 | Charging Logic | Charging, Low Charge | 基于 LDR 检测和模拟充电 |
| 4（最低） | Normal Movement | Forward | 默认步态，最容易被中断 |

**Timer 表：**

| Timer | 时长/周期 | 触发 | 行为 |
|---|---|---|---|
| `explore_timer` | 2 分钟 | 进入 Explore | 到期 → Low Charge；从 Charging 返回时重置 |
| `charging_timer` | 1 分钟 | 进入 Charging | 到期且光照仍在 → Explore；中断 → Low Charge |
| `scan_timer` | Explore 中快 / Low Charge 中慢 | 进入 Scan（并行） | 周期性 HC-SR04 扫描，更新 FD/LD/RD |
| `gait_step_timer` | 每步（可配置），Explore 快 / Low Charge 慢 | Forward, Turn, Veer, Recovery reverse | Non-blocking 步态节拍，到时推进下一腿相位 |
| `led_blink_timer` | 500ms toggle | 进入 Target Detected | Non-blocking LED 闪烁（ticks_ms()） |
| `turn_count` | 阈值 >= 3 | 每次失败的 Turn | >= 3 → Recovery；返回 Forward 时清零 |
| `recovery_count` | 阈值 >= 3 | 每次进入 Recovery | >= 3 → Fault Stop（人工干预） |

---

### 5. avoid_obstacle() 伪代码

```
void avoid_obstacle():
    read FD, LD, RD from latest scan_timer cycle

    if (LD<10 && RD<10 && FD<10) || LD<5 || RD<5 || FD<5 || turn_count>=3:
        state = "Recovery"
    elif FD < 10 cm:
        if LD > RD:       state = "Turn Left"
        elif RD > LD:     state = "Turn Right"
        else:             state = random Turn Left/Right
    elif FD < 15 cm:
        if LD > RD:       state = "Veer Left"
        elif RD > LD:     state = "Veer Right"
        else:             state = random Veer Left/Right

    # Turn: 先停止再转弯，Veer: 行进中微调
    # 仅 Turn 失败后 turn_count++；Veer 完成后直接回到 Scan
```

---

### 6. Mission State 伪代码

**Explore:**
```
if state == "Explore":
    start explore_timer
    while explore_timer < 2 min AND target != found:
        # 并行: Forward (gait_step_timer) + Scan (scan_timer)
        if FD >= 15 cm:
            go forward (full speed)
        else:
            avoid_obstacle()
    if target == found:
        state = "Target Detected"
    else:
        reset explore_timer
        state = "Low Charge"
```

**Low Charge:**
```
if state == "Low Charge":
    while target != found:
        # 并行: Forward (减速, gait_step_timer) + Scan (慢 scan_timer)
        if LDR > threshold:
            state = "Charging"
        elif FD >= 15 cm:
            go forward (reduced speed)
        else:
            avoid_obstacle()
    if target == found:
        state = "Target Detected"
```

**Charging:**
```
if state == "Charging":
    start charging_timer
    stop movement  # 无 Scan, 无 gait_step_timer
    while charging_timer < 1 min:
        if light interrupted:
            state = "Low Charge"
    if light still present:
        reset explore_timer
        state = "Explore"
```

**Target Detected:**
```
if state == "Target Detected":
    stop all movement
    while True:
        blink LED every 500 ms  # led_blink_timer, non-blocking
```

## 关键代码模式

**舵机角度转 duty cycle：**
```python
min_duty = 1638  # 0° = 0.5ms pulse
max_duty = 8192  # 180° = 2.5ms pulse
duty = int(min_duty + (angle / 180) * (max_duty - min_duty))
servo.duty_u16(duty)
```

**Non-blocking 计时（循环中始终使用此模式，禁止 `sleep()`）：**
```python
last_time = utime.ticks_ms()
def periodic_action():
    global last_time
    now = utime.ticks_ms()
    if utime.ticks_diff(now, last_time) > interval_ms:
        last_time = now
        # 执行操作
```

**红色检测（TCS3200）：** pulse 越短 = 颜色越强。当 `r < (g - 50) and r < (b - 50)` 时确认检测到红色。数值为 `time_pulse_us` 结果 — 值越小表示该颜色分量越强。

**超声波测距：** `(pulse_us * 0.0343) / 2` cm。超时设为 12000µs（约 2m）。无回波时返回 100cm。

**LDR：** `read_u16()` 值越大 = 光线越亮。需根据实际环境标定（dark/ambient/bright 参考值）。

## 待解决问题

- **所有 Pin 尚未分配** — 需要根据实际接线确定引脚映射。
- **最终 `main.py` 尚未编写** — `reference/` 中的文件是各组件独立测试，完整的 AFSM 集成逻辑仍有待实现。
