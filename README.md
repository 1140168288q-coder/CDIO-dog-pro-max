# CDIO Robot Dog

Raspberry Pi Pico 2 (RP2350) 四足机器狗 — MicroPython 实现，基于传感器驱动的 AFSM (Augmented Finite State Machine) 控制器。

课程：4FTC2135-0105-2025 Robot Design and Build Project B，Assignment 2。

## 文件说明

### `main.py` — 主控程序

完整的自动导航机器狗控制器，上电即运行。核心架构：

- **5 个 Mission 状态**：Start → Explore ↔ Low Charge ↔ Charging，任意状态检测到红色目标后进入 Target Detected（终止状态）
- **并行行为**：Scan（超声波扫描三方向）+ Forward（四足步态），遇到障碍时自动进入 Turn / Veer / Recovery 避障
- **Non-blocking 计时**：全程使用 `ticks_ms()` 驱动周期性任务（步态、扫描、LED 闪烁），无阻塞 `sleep()`

关键参数全部集中在文件顶部的 **SECTION 2**（`LDR_THRESHOLD`、`DIST_CLEAR`、`RED_THRESHOLD` 等），修改无需深入逻辑层。

### `test.py` — 硬件调试与参数标定工具

交互式菜单，逐项测试传感器和舵机，生成建议参数值直接写回 `main.py`。

| 序号 | 测试项目 | 产出参数 |
|---|---|---|
| 1 | 舵机测试 | `SERVO_OFFSETS` |
| 2 | 超声波测距 | `DIST_CLEAR` / `DIST_TURN` / `DIST_DANGER` |
| 3 | LDR 光敏 | `LDR_THRESHOLD` |
| 4 | 颜色传感器 | `RED_THRESHOLD` |
| 5 | 状态灯 | 确认接线 |
| 6 | 云台舵机 | `SCAN_ANGLES` |
| 7 | 脱困参数 | `REVERSE_MS` / `TURN_STEPS` |

## 快速上手

1. **分配引脚** — 按实际接线填写 `main.py` 和 `test.py` 顶部的 `PIN_*`
2. **标定参数** — 在 Pico 上运行 `test.py`，逐项完成 7 个测试，将建议值填回 `main.py` 的 SECTION 2
3. **部署运行** — 将 `main.py` 保存为 Pico 上的 `main.py`，上电自动运行

## 开发环境

- **IDE**: Thonny（直接连接 Pico 编辑文件）
- **语言**: MicroPython（非 CPython，无 asyncio / pip）
- **仿真**: Wokwi (wokwi.com) 支持无硬件测试
- **参考代码**: `reference/` 目录包含各组件独立测试脚本
