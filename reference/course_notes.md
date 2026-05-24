# 课件笔记汇总

> 本文件汇总自课堂 PPT、PDF 和 docx 课件内容，供开发时快速参考。

---

## 1. 课程概述

课程编号：4FTC2135-0105-2025 - Robot Design and Build Project B

课程目标：完成小型移动机器人的电子和软件部分，包括：
- 电路焊接与接线
- 电子技术图纸
- 编程控制机器人执行指定任务
- 项目成果展示

指导教师：Salman Khan & Matt Goodro

---

## 2. Raspberry Pi Pico 基础

### 概述
- Pico 是微控制器板（不是微型计算机）
- Pico 2 使用 RP2350 芯片，530KB SRAM
- 使用 MicroPython 编程
- 通过 Thonny IDE 开发
- 保存为 main.py 时上电自动运行

### 有用链接
- Thonny IDE
- Wokwi 仿真器: wokwi.com
- MicroPython 文档: docs.micropython.org
- Pico 引脚图: pico2.pinout.xyz

### 刷写 Pico
1. 按住 BOOTSEL 按钮
2. 连接 USB
3. 释放按钮
4. 拖放 MicroPython .uf2 文件到 Pico 驱动器

---

## 3. PWM 与舵机控制

### PWM 基础
- PWM = Pulse Width Modulation
- 通过开关信号模拟模拟输出
- Duty cycle = 高电平占比

### 标准舵机参数
- 频率：50Hz（周期 20ms）
- 位置由脉冲宽度决定：

| Angle | Pulse Width | Duty Cycle | duty_u16() |
| --- | --- | --- | --- |
| 0° | 0.5 ms | 2.5% | ≈ 1638 |
| 90° | 1.5 ms | 7.5% | ≈ 4915 |
| 180° | 2.5 ms | 12.5% | ≈ 8192 |

### 舵机控制代码
```python
def set_position(servo, angle):
    min_duty = 1638  # 0° = 0.5 ms pulse
    max_duty = 8192  # 180° = 2.5 ms pulse
    duty = int(min_duty + (angle / 180) * (max_duty - min_duty))
    servo.duty_u16(duty)
```

---

## 4. 步态设计

### 四步态行走（推荐方案）

使用 step array 管理步态：

```python
step_time = 200  # ms
arc = 18         # 度
mid_point = 90
one = int(mid_point - arc/2)    # 81
two = int(mid_point - arc/6)    # 87
three = int(mid_point + arc/6)  # 93
four = int(mid_point + arc/2)   # 99

steps = [
    (four, one, three, two),    # Step 1
    (three, four, four, three), # Step 2
    (two, three, one, four),    # Step 3
    (one, two, two, one)        # Step 4
]
```

腿部排列：
- RF_Servo (Right Forward) → GP2
- RR_Servo (Right Rear) → GP0
- LF_Servo (Left Forward) → GP3
- LR_Servo (Left Rear) → GP1

### Trot Gait（对角步态，备选）
- LF + RR 同相位
- RF + LR 反相位（使用 180 - angle）
- 角度范围：60° - 120°
- 步进：2°，延迟：0.01s

### 两步态行走（最简方案）
1. 抬起 LF + RR
2. 全部回中（90°）
3. 抬起 RF + LR
4. 全部回中（90°）

---

## 5. LDR 光敏传感器

### 原理
- LDR 是可变电阻，阻值随光照变化
- Pico 不能直接测电阻，需要分压电路
- ADC 将模拟电压转为 16 位数字值（0-65535）

### 接线
- LDR 一端 → 3V3(OUT)（Pin 36，不是 VBUS）
- LDR 另一端 → GP26（ADC0）
- 10kΩ 电阻从 GP26 → GND
- 可选：0.1µF 电容并联电阻（滤波）

### 读取代码
```python
from machine import ADC, Pin
ldr = ADC(Pin(26))
value = ldr.read_u16()  # 0-65535
percent = round((value / 65535) * 100, 1)
```

### 标定建议
- 遮住传感器：记录暗值
- 环境光：记录环境值
- 手电靠近：记录亮值
- 手电直照：记录最亮值
- 据此设定 threshold

---

## 6. TCS3200 颜色传感器

### 原理
- TCS3200 是 light-to-frequency converter
- 内置 RGB + Clear 滤波器阵列
- 通过 S2/S3 选择通道
- 输出方波，频率正比于颜色强度
- 用 `time_pulse_us` 测量脉冲宽度
- 短脉冲 = 高频 = 颜色强；长脉冲 = 低频 = 颜色弱

### 接线
| Pin | 连接 |
| --- | --- |
| VCC | Pico VCC |
| GND | Pico GND |
| S0 | 3V3(OUT) |
| S1 | Pico GND |
| S2 | GP10 |
| S3 | GP11 |
| OUT | GP12 |
| LED | GP13 |

### S2/S3 通道选择
| S2 | S3 | 通道 |
| --- | --- | --- |
| LOW | LOW | Red |
| LOW | HIGH | Blue |
| HIGH | HIGH | Green |
| HIGH | LOW | Clear |

### 读取代码
```python
def get_averaged_colour(samples=10):
    led_pin.on()
    utime.sleep_ms(50)
    total_r, total_g, total_b = 0, 0, 0
    try:
        for _ in range(samples):
            s2.low(); s3.low()
            total_r += time_pulse_us(signal, 1)
            s2.low(); s3.high()
            total_b += time_pulse_us(signal, 1)
            s2.high(); s3.high()
            total_g += time_pulse_us(signal, 1)
            utime.sleep_ms(2)
    finally:
        led_pin.off()
    return total_r // samples, total_g // samples, total_b // samples
```

### 红色判断
```python
if r < (g - 50) and r < (b - 50):
    # Red detected
```

---

## 7. 超声波传感器

### 原理
- 发送 10us 高电平触发脉冲
- 测量 Echo 引脚高电平持续时间
- 距离 = (脉冲时间 × 0.0343) / 2 cm

### 接线
- Trig → GP14
- Echo → GP15

### 读取代码
```python
def get_distance(samples=5):
    total_pulse_time = 0
    valid_samples = 0
    for _ in range(samples):
        trig.value(0)
        utime.sleep_us(2)
        trig.value(1)
        utime.sleep_us(10)
        trig.value(0)
        pulse = machine.time_pulse_us(echo, 1, 12000)
        if pulse > 0:
            total_pulse_time += pulse
            valid_samples += 1
        utime.sleep_ms(10)
    if valid_samples == 0:
        return 100
    avg_pulse_time = total_pulse_time / valid_samples
    distance = (avg_pulse_time * 0.0343) / 2
    return distance
```



---

