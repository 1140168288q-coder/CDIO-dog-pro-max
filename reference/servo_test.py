# 舵机步态测试
# 来源：step_gait_basic.py + step_gait_with_ldr.py + trot_gait_continuous.py
# 功能：测试四步态行走 + trot gait + 初始化

import machine, utime
from machine import Pin, PWM
from time import sleep

# ============================================================
# 引脚定义（测试用，正式项目请使用 pins.py）
# ============================================================
RR_Servo = PWM(Pin(0))  # Right Rear Servo
LR_Servo = PWM(Pin(1))  # Left Rear Servo
RF_Servo = PWM(Pin(2))  # Right Forward Servo
LF_Servo = PWM(Pin(3))  # Left Forward Servo

# Set Frequency for each servo (50 Hz is standard for hobby servos)
RR_Servo.freq(50)
LR_Servo.freq(50)
RF_Servo.freq(50)
LF_Servo.freq(50)

# ============================================================
# 舵机控制函数
# ============================================================
def set_position(servo, angle):
    angle = max(0, min(180, angle))  # clamp to 0-180
    min_duty = 1638  # 0° = 0.5 ms pulse
    max_duty = 8192  # 180° = 2.5 ms pulse
    duty = int(min_duty + (angle / 180) * (max_duty - min_duty))
    servo.duty_u16(duty)

def initialize():
    """Set all servos to 90 degrees (neutral position)"""
    set_position(RR_Servo, 90)
    set_position(LR_Servo, 90)
    set_position(RF_Servo, 90)
    set_position(LF_Servo, 90)
    print("All servos initialized to 90 degrees")

# ============================================================
# 四步态行走（non-blocking 版本）
# ============================================================
step_time = 200  # ms
arc = 18
mid_point = 90
one = int(mid_point - arc/2)
two = int(mid_point - arc/6)
three = int(mid_point + arc/6)
four = int(mid_point + arc/2)

steps = [
    (four, one, three, two),    # Step 1
    (three, four, four, three), # Step 2
    (two, three, one, four),    # Step 3
    (one, two, two, one)        # Step 4
]

current_step = 0
last_step_time = utime.ticks_ms()

def walk(now):
    """Non-blocking walk function using timer"""
    global last_step_time, current_step
    if utime.ticks_diff(now, last_step_time) > step_time:
        last_step_time = now
        s = steps[current_step]
        set_position(RF_Servo, s[0])
        set_position(RR_Servo, s[1])
        set_position(LF_Servo, s[2])
        set_position(LR_Servo, s[3])
        current_step = (current_step + 1) % len(steps)

# ============================================================
# Trot Gait（连续版本，blocking）
# ============================================================
def trot_gait(duration_sec=5):
    """Trot gait: diagonal legs in sync, blocking version for testing"""
    MIN_ANGLE = 60
    MAX_ANGLE = 120
    STEP_SPEED = 2
    DELAY = 0.01

    start = utime.ticks_ms()
    print(f"Running trot gait for {duration_sec} seconds...")

    while utime.ticks_diff(utime.ticks_ms(), start) < duration_sec * 1000:
        for angle in range(MIN_ANGLE, MAX_ANGLE + 1, STEP_SPEED):
            set_position(LF_Servo, angle)
            set_position(RR_Servo, angle)
            set_position(RF_Servo, 180 - angle)
            set_position(LR_Servo, 180 - angle)
            sleep(DELAY)

        for angle in range(MAX_ANGLE, MIN_ANGLE - 1, -STEP_SPEED):
            set_position(LF_Servo, angle)
            set_position(RR_Servo, angle)
            set_position(RF_Servo, 180 - angle)
            set_position(LR_Servo, 180 - angle)
            sleep(DELAY)

    initialize()
    print("Trot gait complete.")

# ============================================================
# 测试主程序
# ============================================================
try:
    print("=== Servo Gait Test ===")
    print()

    # Test 1: Initialize
    print("1. Initializing all servos to 90 degrees...")
    initialize()
    sleep(1)

    # Test 2: Four-step gait (non-blocking)
    print("\n2. Four-step gait test (5 seconds, non-blocking):")
    print(f"   arc={arc}, step_time={step_time}ms")
    start = utime.ticks_ms()
    while utime.ticks_diff(utime.ticks_ms(), start) < 5000:
        now = utime.ticks_ms()
        walk(now)
        utime.sleep_ms(10)

    initialize()
    sleep(1)

    # Test 3: Trot gait (blocking)
    print("\n3. Trot gait test (5 seconds, blocking):")
    trot_gait(5)

    print("\n=== All tests complete ===")

except KeyboardInterrupt:
    initialize()
    print("Test stopped. Servos reset to 90 degrees.")
