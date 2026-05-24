# TCS3200 颜色传感器测试
# 来源：color_sensor_reader.py + colour sensor details.docx
# 功能：读取 RGB 值，检测红色目标

from machine import Pin, time_pulse_us
import utime

# ============================================================
# 引脚定义（测试用，正式项目请使用 pins.py）
# ============================================================
# S0 → 3V3 (硬件固定)
# S1 → GND (硬件固定)
s2 = Pin(10, Pin.OUT)
s3 = Pin(11, Pin.OUT)
signal = Pin(12, Pin.IN)
led_pin = Pin(13, Pin.OUT)  # Control pin for the onboard LEDs

# ============================================================
# 颜色读取函数（10次采样取平均）
# ============================================================
def get_averaged_colour(samples=10):
    # 1. Turn on lights
    led_pin.on()
    utime.sleep_ms(50)  # sleep for 50ms for stability

    total_r, total_g, total_b = 0, 0, 0

    try:
        for _ in range(samples):
            # Red: S2=LOW, S3=LOW
            s2.low(); s3.low()
            total_r += time_pulse_us(signal, 1)

            # Blue: S2=LOW, S3=HIGH
            s2.low(); s3.high()
            total_b += time_pulse_us(signal, 1)

            # Green: S2=HIGH, S3=HIGH
            s2.high(); s3.high()
            total_g += time_pulse_us(signal, 1)

            utime.sleep_ms(2)

    finally:
        # 2. Always turn lights off when done (even if there's an error)
        led_pin.off()

    return total_r // samples, total_g // samples, total_b // samples

# ============================================================
# 红色检测判断
# ============================================================
def is_red(r, g, b, threshold=50):
    """判断是否为红色：红色脉冲短（值小）且明显小于绿色和蓝色"""
    return r < (g - threshold) and r < (b - threshold)

# ============================================================
# 测试主程序
# ============================================================
try:
    print("=== Color Sensor Test ===")
    print("S2/S3 channel selection:")
    print("  S2=LOW,  S3=LOW  → Red")
    print("  S2=LOW,  S3=HIGH → Blue")
    print("  S2=HIGH, S3=HIGH → Green")
    print()
    print("Note: shorter pulse = stronger colour presence")
    print()
    print("Continuous reading (Ctrl+C to stop):")
    print()

    while True:
        r, g, b = get_averaged_colour()
        red_detected = is_red(r, g, b)
        status = " *** RED DETECTED ***" if red_detected else ""
        print("R:{} G:{} B:{}{}".format(r, g, b, status))

        if red_detected:
            # 二次确认
            utime.sleep_ms(10)
            r, g, b = get_averaged_colour()
            if is_red(r, g, b):
                print("RED CONFIRMED!")

        utime.sleep(0.1)

except KeyboardInterrupt:
    led_pin.off()
    print("Test stopped.")
