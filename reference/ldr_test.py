# LDR 光敏传感器测试
# 来源：ldr_reader.py + classroom_code.py 中 LDR 相关代码
# 功能：读取 LDR 值，显示原始值和百分比

from machine import ADC, Pin
import utime

# ============================================================
# 引脚定义（测试用，正式项目请使用 pins.py）
# ============================================================
# LDR connected to GP26 (ADC0)
# 接线：LDR 一端接 3V3(OUT)，另一端接 GP26，10kΩ 电阻从 GP26 接 GND
ldr = ADC(Pin(26))

# ============================================================
# LDR 读取函数
# ============================================================
def read_ldr():
    value = ldr.read_u16()  # Read the LDR value (16-bit unsigned integer)
    percent = round((value / 65535) * 100, 1)
    return value, percent

# ============================================================
# 标定辅助函数
# ============================================================
def calibrate_ldr(samples=20, interval_ms=100):
    """采集多次读数，返回平均值，用于标定 threshold"""
    total = 0
    for i in range(samples):
        total += ldr.read_u16()
        utime.sleep_ms(interval_ms)
    avg = total // samples
    percent = round((avg / 65535) * 100, 1)
    print(f"   Average: {avg} | Brightness: {percent}%")
    return avg

# ============================================================
# 测试主程序
# ============================================================
try:
    print("=== LDR Sensor Test ===")
    print()
    print("1. Calibration mode:")
    print("   Cover the sensor, then press Enter...")
    input()
    print("   Reading dark value:")
    dark_val = calibrate_ldr()

    print()
    print("   Uncover the sensor (ambient light), then press Enter...")
    input()
    print("   Reading ambient value:")
    ambient_val = calibrate_ldr()

    print()
    print("   Shine torch on sensor, then press Enter...")
    input()
    print("   Reading bright value:")
    bright_val = calibrate_ldr()

    print()
    suggested_threshold = (ambient_val + bright_val) // 2
    print(f"   Suggested threshold: {suggested_threshold}")

    print()
    print("2. Continuous reading (Ctrl+C to stop):")
    while True:
        value, percent = read_ldr()
        status = "BRIGHT" if value > suggested_threshold else "normal"
        print(f"   Raw: {value} | Brightness: {percent}% | {status}")
        utime.sleep(0.5)

except KeyboardInterrupt:
    print("Test stopped.")
