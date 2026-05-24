# 超声波传感器测试
# 来源：classroom_code.py 中的超声波相关代码
# 功能：测量距离 + 多角度扫描

import machine, utime
from machine import Pin, PWM, time_pulse_us

# ============================================================
# 引脚定义（测试用，正式项目请使用 pins.py）
# ============================================================
trig = Pin(22, Pin.OUT)
echo = Pin(9, Pin.IN)

# 超声波扫描舵机
US_Servo = PWM(Pin(18))
US_Servo.freq(50)

# ============================================================
# 舵机控制函数
# ============================================================
def set_position(servo, angle):
    min_duty = 1638  # 0° = 0.5 ms pulse
    max_duty = 8192  # 180° = 2.5 ms pulse
    duty = int(min_duty + (angle / 180) * (max_duty - min_duty))
    servo.duty_u16(duty)

# ============================================================
# 测距函数（5次采样取平均）
# ============================================================
def get_distance(samples=5):
    total_pulse_time = 0
    valid_samples = 0
    for _ in range(samples):
        trig.value(0)
        utime.sleep_us(2)
        # Send 10us high pulse
        trig.value(1)
        utime.sleep_us(10)
        trig.value(0)

        # Measure the duration of the high pulse on the echo pin
        # 12000us is a ~2-meter timeout; if no echo, it returns -2 or -1
        pulse = machine.time_pulse_us(echo, 1, 12000)
        if pulse > 0:
            total_pulse_time += pulse
            valid_samples += 1
        # Small delay between samples so waves can dissipate
        utime.sleep_ms(10)
    if valid_samples == 0:
        return 100  # No valid echoes received
    avg_pulse_time = total_pulse_time / valid_samples
    # Speed of sound ~0.0343 cm/us
    # Distance equals (time * speed) / 2 (for send and return)
    distance = (avg_pulse_time * 0.0343) / 2
    return distance

# ============================================================
# 多角度扫描函数（non-blocking）
# ============================================================
scan_angles = [5, 90, 175, 90]
distances = [100, 100, 100, 100]
scan_index = 1  # set to 1 as the US servo initializes to 90 degrees
scan_counter = 0
last_scan_time = utime.ticks_ms()
step_time = 300  # ms

def US_Scan(now):
    global scan_index, last_scan_time, scan_counter

    if utime.ticks_diff(now, last_scan_time) > step_time/2:
        if scan_counter % 2 == 0:  # even: take measurement
            distances[scan_index] = get_distance()
            scan_index = (scan_index + 1) % 4
            scan_counter = (scan_counter + 1)
            last_scan_time = now
            return True  # new reading added
        else:  # odd: move servo to next position
            set_position(US_Servo, scan_angles[scan_index])
            scan_counter = (scan_counter + 1)
            last_scan_time = now
            return False  # no reading, servo moved

# ============================================================
# 测试主程序
# ============================================================
try:
    set_position(US_Servo, 90)
    utime.sleep_ms(500)

    print("=== Ultrasonic Sensor Test ===")
    print("1. Single distance reading:")
    dist = get_distance()
    print(f"   Distance: {dist:.1f} cm")

    print("\n2. Multi-angle scan test (press Ctrl+C to stop):")
    while True:
        now = utime.ticks_ms()
        result = US_Scan(now)
        if result:
            print(f"   Distances: {distances}")
        utime.sleep_ms(50)

except KeyboardInterrupt:
    print("Test stopped.")
