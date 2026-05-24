import utime
import machine

# =============================================================================
# test.py — 机器狗硬件调试与参数标定工具
# 作用：
#   1. 单独检查各传感器/舵机是否接线正确
#   2. 为 main.py 测出更合适的参数
# 使用方式：
#   1. 先把本文件顶部 PIN_* 改成和 main.py 一致
#   2. 上传到 Pico 后在 Thonny 运行
#   3. 按菜单逐项测试
#   4. 将测试结果填回 main.py 的参数区
# =============================================================================

# =============================================================================
# PIN 定义（需与 main.py 保持一致）
# =============================================================================
PIN_RF         = 10
PIN_RR         = 9
PIN_LF         = 21
PIN_LR         = 22
PIN_US_SERVO   = 6
PIN_LDR        = 28
PIN_TRIG       = 18
PIN_ECHO       = 13
PIN_TCS_S2     = 16
PIN_TCS_S3     = 17
PIN_TCS_OUT    = 14
PIN_TCS_LED    = 15
PIN_STATUS_LED = 11

# =============================================================================
# 与 main.py 保持一致的基础参数
# =============================================================================
MIN_DUTY  = 1638
MAX_DUTY  = 8192
MID_ANGLE = 90
ARC       = 18

_a = MID_ANGLE - ARC // 2
_b = MID_ANGLE - ARC // 6
_c = MID_ANGLE + ARC // 6
_d = MID_ANGLE + ARC // 2

FORWARD_STEPS = [
    (_d, _a, _c, _b),
    (_c, _d, _d, _c),
    (_b, _c, _a, _d),
    (_a, _b, _b, _a),
]
REVERSE_STEPS = list(reversed(FORWARD_STEPS))
TURN_L_STEPS  = [(_d, _d, _a, _a), (_a, _a, _d, _d)]
TURN_R_STEPS  = [(_a, _a, _d, _d), (_d, _d, _a, _a)]


def _line():
    print("-" * 60)


def _title(text):
    print()
    print("=" * 60)
    print(" " + text)
    print("=" * 60)


def _pause(text="按 Enter 继续"):
    input("  {}: ".format(text))


def _check_pin(pin_num, label):
    if pin_num is None:
        print("  [跳过] {} 未设置，请先填写顶部 PIN_*".format(label))
        return False
    return True


def _make_pwm(pin_num):
    if pin_num is None:
        return None
    pwm = machine.PWM(machine.Pin(pin_num))
    pwm.freq(50)
    return pwm


def _duty(angle):
    angle = max(0, min(180, angle))
    return int(MIN_DUTY + (angle / 180.0) * (MAX_DUTY - MIN_DUTY))


def _set_servo(pwm, angle):
    if pwm is not None:
        pwm.duty_u16(_duty(angle))


def _print_writeback(lines):
    print()
    print("  建议写回 main.py 的参数：")
    for line in lines:
        print("    {}".format(line))


def _dist_once():
    if PIN_TRIG is None or PIN_ECHO is None:
        return 100
    trig = machine.Pin(PIN_TRIG, machine.Pin.OUT)
    echo = machine.Pin(PIN_ECHO, machine.Pin.IN)
    trig.value(0)
    utime.sleep_us(2)
    trig.value(1)
    utime.sleep_us(10)
    trig.value(0)
    pulse = machine.time_pulse_us(echo, 1, 12000)
    return round((pulse * 0.0343) / 2.0, 1) if pulse > 0 else 100


def _tcs_channel(s2_val, s3_val, pin_s2, pin_s3, pin_out):
    pin_s2.value(s2_val)
    pin_s3.value(s3_val)
    utime.sleep_ms(1)
    return machine.time_pulse_us(pin_out, 0, 200_000)


def _read_rgb(pin_s2, pin_s3, pin_out, pin_led, samples=10):
    pin_led.value(1)
    utime.sleep_ms(50)
    r = g = b = 0
    for _ in range(samples):
        r += _tcs_channel(0, 0, pin_s2, pin_s3, pin_out)
        g += _tcs_channel(1, 1, pin_s2, pin_s3, pin_out)
        b += _tcs_channel(0, 1, pin_s2, pin_s3, pin_out)
    pin_led.value(0)
    return r // samples, g // samples, b // samples


def test_servo():
    _title("舵机测试")
    print("  目标：")
    print("  1. 检查四条腿舵机是否能正常动作")
    print("  2. 测出每条腿的中位补偿值 SERVO_OFFSETS")
    print("  3. 简单预览前进步态是否顺畅")

    servo_defs = [
        ("RF 右前腿", PIN_RF, 0),
        ("RR 右后腿", PIN_RR, 1),
        ("LF 左前腿", PIN_LF, 2),
        ("LR 左后腿", PIN_LR, 3),
    ]
    offsets = [0, 0, 0, 0]

    for label, pin_num, idx in servo_defs:
        if not _check_pin(pin_num, label):
            continue

        pwm = _make_pwm(pin_num)
        print()
        _line()
        print("  当前测试：{}".format(label))
        _line()
        print("  观察这条腿是否能平滑到达 0° / 90° / 180°。")

        for angle in [90, 0, 90, 180, 90]:
            _set_servo(pwm, angle)
            print("    移动到 {:3d}°  duty={}".format(angle, _duty(angle)))
            utime.sleep_ms(700)

        print()
        print("  现在舵机停在 90°。")
        raw = input("  如果机械中位有偏差，请输入补偿角度（例如 -4、+6，直接回车表示 0）: ").strip()
        try:
            offset = int(raw) if raw else 0
        except:
            offset = 0
        offsets[idx] = offset

        corrected = MID_ANGLE + offset
        _set_servo(pwm, corrected)
        print("  已应用补偿 {:+d}°，当前角度约为 {}°".format(offset, corrected))
        _pause("确认效果后按 Enter")
        pwm.deinit()

    print()
    _line()
    print("  前进步态预览")
    _line()
    print("  机器人将按当前步态前进 10 轮，用于观察步态是否协调。")
    _pause("准备好后按 Enter 开始")

    pwms = [_make_pwm(PIN_RF), _make_pwm(PIN_RR), _make_pwm(PIN_LF), _make_pwm(PIN_LR)]
    try:
        for step in FORWARD_STEPS * 10:
            for i, pwm in enumerate(pwms):
                _set_servo(pwm, step[i] + offsets[i])
            utime.sleep_ms(200)
    except KeyboardInterrupt:
        print("  已手动停止预览。")

    for i, pwm in enumerate(pwms):
        _set_servo(pwm, MID_ANGLE + offsets[i])
    utime.sleep_ms(400)
    for pwm in pwms:
        if pwm:
            pwm.deinit()

    print()
    print("  结果汇总：SERVO_OFFSETS = {}".format(offsets))
    _print_writeback(["SERVO_OFFSETS = {}".format(offsets)])


def test_ultrasonic():
    _title("超声波测试")
    if not _check_pin(PIN_TRIG, "TRIG") or not _check_pin(PIN_ECHO, "ECHO"):
        return

    print("  目标：")
    print("  1. 检查 HC-SR04 是否能稳定测距")
    print("  2. 为 main.py 标定 DIST_CLEAR / DIST_TURN / DIST_DANGER")
    print()
    print("  默认逻辑：")
    print("    FD >= 15 cm  -> 前进")
    print("    10~15 cm     -> Veer")
    print("    < 10 cm      -> Turn")
    print("    < 5 cm       -> Recovery")
    print()
    print("  先进行实时读数观察，按 Ctrl+C 结束。")
    _pause("按 Enter 开始实时读数")

    try:
        while True:
            dist = _dist_once()
            if dist >= 15:
                status = "前方安全"
            elif dist >= 10:
                status = "轻微障碍（Veer）"
            elif dist >= 5:
                status = "严重障碍（Turn）"
            else:
                status = "危险过近（Recovery）"
            bar = "#" * min(40, int(dist))
            print("  {:6.1f} cm  {:18s} |{}".format(dist, status, bar))
            utime.sleep_ms(200)
    except KeyboardInterrupt:
        print("  已停止实时读数。")

    print()
    print("  接下来测三个关键阈值。")
    recorded = {}
    cases = [
        ("DIST_CLEAR", 15, "把障碍物放在“刚好还应该继续前进”的位置"),
        ("DIST_TURN", 10, "把障碍物放在“刚好应该开始转向”的位置"),
        ("DIST_DANGER", 5, "把障碍物放在“非常近，应进入 Recovery”的位置"),
    ]

    for key, suggested, hint in cases:
        print()
        _line()
        print("  测试 {}".format(key))
        print("  {}".format(hint))
        print("  参考距离约 {} cm".format(suggested))
        _pause("摆好障碍物后按 Enter")

        samples = []
        for _ in range(5):
            dist = _dist_once()
            samples.append(dist)
            print("    {:.1f} cm".format(dist))
            utime.sleep_ms(300)
        avg = round(sum(samples) / len(samples), 1)
        recorded[key] = avg
        print("  平均值：{} cm".format(avg))

    clear_v = max(12, int(recorded.get("DIST_CLEAR", 15)))
    turn_v = max(7, int(recorded.get("DIST_TURN", 10)))
    danger_v = max(3, int(recorded.get("DIST_DANGER", 5)))

    print()
    print("  结果汇总：")
    print("    DIST_CLEAR  ≈ {}".format(clear_v))
    print("    DIST_TURN   ≈ {}".format(turn_v))
    print("    DIST_DANGER ≈ {}".format(danger_v))
    _print_writeback([
        "DIST_CLEAR  = {}".format(clear_v),
        "DIST_TURN   = {}".format(turn_v),
        "DIST_DANGER = {}".format(danger_v),
    ])


def test_ldr():
    _title("LDR 光敏测试")
    if not _check_pin(PIN_LDR, "LDR"):
        return

    adc = machine.ADC(machine.Pin(PIN_LDR))
    print("  目标：测出 Charging 触发阈值 LDR_THRESHOLD。")
    print("  数值越大表示越亮。")
    print()
    print("  依次测三种情况：")
    print("    1. 遮挡传感器")
    print("    2. 环境光")
    print("    3. 强光直射")

    results = {}
    cases = [
        ("dark", "遮住传感器"),
        ("ambient", "正常环境光"),
        ("bright", "手电/手机灯直射"),
    ]

    for key, label in cases:
        print()
        _line()
        print("  当前条件：{}".format(label))
        _pause("准备好后按 Enter")
        samples = []
        for i in range(10):
            value = adc.read_u16()
            samples.append(value)
            print("    {:2d}: {:5d}".format(i + 1, value))
            utime.sleep_ms(200)
        avg = int(sum(samples) / len(samples))
        results[key] = avg
        print("  平均值：{}".format(avg))

    suggested = (results["ambient"] + results["bright"]) // 2

    print()
    print("  建议阈值：{}".format(suggested))
    print("  接下来做实时验证，按 Ctrl+C 结束。")
    _pause("按 Enter 开始验证")

    try:
        while True:
            value = adc.read_u16()
            status = "达到 Charging 条件" if value >= suggested else "未达到 Charging 条件"
            print("  LDR = {:5d}  {}".format(value, status))
            utime.sleep_ms(300)
    except KeyboardInterrupt:
        print("  已停止验证。")

    print()
    print("  结果汇总：")
    print("    遮挡：  {}".format(results["dark"]))
    print("    环境光：{}".format(results["ambient"]))
    print("    强光：  {}".format(results["bright"]))
    print("    建议 LDR_THRESHOLD = {}".format(suggested))
    _print_writeback(["LDR_THRESHOLD = {}".format(suggested)])


def test_colour():
    _title("颜色传感器测试")
    for label, pin_num in [
        ("TCS_S2", PIN_TCS_S2),
        ("TCS_S3", PIN_TCS_S3),
        ("TCS_OUT", PIN_TCS_OUT),
        ("TCS_LED", PIN_TCS_LED),
    ]:
        if not _check_pin(pin_num, label):
            return

    pin_s2 = machine.Pin(PIN_TCS_S2, machine.Pin.OUT)
    pin_s3 = machine.Pin(PIN_TCS_S3, machine.Pin.OUT)
    pin_out = machine.Pin(PIN_TCS_OUT, machine.Pin.IN)
    pin_led = machine.Pin(PIN_TCS_LED, machine.Pin.OUT)
    pin_led.value(0)

    print("  目标：测出 RED_THRESHOLD，减少误判红色。")
    print("  判定逻辑：r < (g - RED_THRESHOLD) 且 r < (b - RED_THRESHOLD)")
    print("  注意：TCS3200 的 pulse 越小，说明该颜色越强。")

    red_data = []
    bg_data = []
    groups = [
        ("red", "红色目标"),
        ("bg", "非红色背景"),
    ]

    for key, label in groups:
        print()
        _line()
        print("  当前测试表面：{}".format(label))
        print("  请将传感器与表面保持大约 1~5 cm。")
        _pause("准备好后按 Enter")

        for i in range(5):
            r, g, b = _read_rgb(pin_s2, pin_s3, pin_out, pin_led, 10)
            gr = g - r
            br = b - r
            print("    {:2d}: R={:6d} G={:6d} B={:6d}  G-R={:5d} B-R={:5d}".format(
                i + 1, r, g, b, gr, br
            ))
            if key == "red":
                red_data.append((r, g, b))
            else:
                bg_data.append((r, g, b))
            utime.sleep_ms(200)

    print()
    print("  实时观察模式：在红色目标和背景之间来回移动传感器，按 Ctrl+C 结束。")
    print("  同时显示阈值 30 / 50 / 70 下是否判定为红色。")
    _pause("按 Enter 开始实时观察")

    try:
        while True:
            r, g, b = _read_rgb(pin_s2, pin_s3, pin_out, pin_led, 5)
            gr = g - r
            br = b - r
            t30 = "红" if (r < g - 30 and r < b - 30) else "-"
            t50 = "红" if (r < g - 50 and r < b - 50) else "-"
            t70 = "红" if (r < g - 70 and r < b - 70) else "-"
            print("  R={:5d} G={:5d} B={:5d}  G-R={:5d} B-R={:5d}  [30:{} 50:{} 70:{}]".format(
                r, g, b, gr, br, t30, t50, t70
            ))
            utime.sleep_ms(150)
    except KeyboardInterrupt:
        pin_led.value(0)
        print("  已停止实时观察。")

    if red_data and bg_data:
        red_gr = [g - r for r, g, b in red_data]
        red_br = [b - r for r, g, b in red_data]
        bg_gr = [g - r for r, g, b in bg_data]
        bg_br = [b - r for r, g, b in bg_data]
        min_red_signal = min(min(red_gr), min(red_br))
        max_bg_signal = max(max(bg_gr + bg_br), 0)
        if min_red_signal > max_bg_signal:
            suggested = (min_red_signal + max_bg_signal) // 2
        else:
            suggested = max(10, min_red_signal // 2)
    else:
        suggested = 50

    print()
    print("  结果汇总：")
    print("    建议 RED_THRESHOLD = {}".format(suggested))
    print("    如果背景也经常误判为红色，请适当调大。")
    print("    如果红色不容易触发，请适当调小。")
    _print_writeback(["RED_THRESHOLD = {}".format(suggested)])


def test_led():
    _title("状态灯测试")
    if not _check_pin(PIN_STATUS_LED, "STATUS_LED"):
        return

    led = machine.Pin(PIN_STATUS_LED, machine.Pin.OUT)
    print("  目标：确认状态灯能正常亮、灭、闪烁。")

    led.value(1)
    print("  现在常亮。")
    _pause()

    led.value(0)
    print("  现在熄灭。")
    _pause()

    print("  现在按 500 ms 间隔闪烁，按 Ctrl+C 停止。")
    state = False
    try:
        while True:
            state = not state
            led.value(1 if state else 0)
            utime.sleep_ms(500)
    except KeyboardInterrupt:
        led.value(0)
        print("  已停止闪烁测试。")

    print()
    print("  如果状态灯没有反应，请检查：")
    print("    1. PIN_STATUS_LED 是否填写正确")
    print("    2. 是否使用了板载 LED 或带限流电阻的外接 LED")


def test_scan_servo():
    _title("超声波云台舵机测试")
    if not _check_pin(PIN_US_SERVO, "US_SERVO"):
        return

    pwm = _make_pwm(PIN_US_SERVO)
    print("  目标：确认扫描角度是否合适，并辅助检查云台朝向。")
    print("  main.py 默认：SCAN_ANGLES = [30, 90, 150]")

    confirmed = {}
    items = [
        ("LD", 30, "左侧"),
        ("FD", 90, "正前方"),
        ("RD", 150, "右侧"),
    ]

    for key, default_angle, direction in items:
        print()
        _line()
        print("  当前位置：{}  默认 {}°".format(key, default_angle))
        _set_servo(pwm, default_angle)
        utime.sleep_ms(600)
        raw = input("  如果当前不是朝向{}，请输入修正角度；直接回车保留默认值: ".format(direction)).strip()
        try:
            angle = int(raw) if raw else default_angle
        except:
            angle = default_angle
        confirmed[key] = angle
        _set_servo(pwm, angle)
        utime.sleep_ms(400)

    if PIN_TRIG is not None and PIN_ECHO is not None:
        print()
        _line()
        print("  可选：扫描后稳定时间参考")
        print("  把障碍物放在正前方约 30 cm 处。")
        _pause("准备好后按 Enter")

        for delay in [50, 100, 150, 200, 300]:
            _set_servo(pwm, confirmed.get("LD", 30))
            utime.sleep_ms(300)
            _set_servo(pwm, confirmed.get("FD", 90))
            utime.sleep_ms(delay)
            dist = _dist_once()
            print("  舵机移动后等待 {:3d} ms -> {:.1f} cm".format(delay, dist))
            utime.sleep_ms(200)

        print("  一般选择最短但又稳定的等待时间即可。")

    pwm.deinit()

    ld_angle = confirmed.get("LD", 30)
    fd_angle = confirmed.get("FD", 90)
    rd_angle = confirmed.get("RD", 150)
    print()
    print("  结果汇总：SCAN_ANGLES = [{}, {}, {}]".format(ld_angle, fd_angle, rd_angle))
    _print_writeback([
        "SCAN_ANGLES = [{}, {}, {}]".format(ld_angle, fd_angle, rd_angle),
    ])


def test_recovery():
    _title("脱困参数测试")
    print("  目标：为 main.py 测出更合适的 REVERSE_MS 和 TURN_STEPS。")
    print("  这两个参数直接影响 Recovery 是否能真正脱困。")

    pwms = [_make_pwm(PIN_RF), _make_pwm(PIN_RR), _make_pwm(PIN_LF), _make_pwm(PIN_LR)]

    def stand():
        for pwm in pwms:
            _set_servo(pwm, MID_ANGLE)
        utime.sleep_ms(400)

    def run_steps(step_table, count, step_ms):
        for i in range(count):
            step = step_table[i % len(step_table)]
            for j, pwm in enumerate(pwms):
                _set_servo(pwm, step[j])
            utime.sleep_ms(step_ms)

    stand()

    print()
    _line()
    print("  第一部分：后退时间 REVERSE_MS")
    print("  目标效果：后退约 10~15 cm，能脱离卡住位置。")

    suggested_reverse = 800
    for reverse_ms in [400, 600, 800, 1000]:
        print("  测试后退 {} ms".format(reverse_ms))
        _pause("按 Enter 开始")
        stand()
        start = utime.ticks_ms()
        index = 0
        while utime.ticks_diff(utime.ticks_ms(), start) < reverse_ms:
            step = REVERSE_STEPS[index % len(REVERSE_STEPS)]
            for j, pwm in enumerate(pwms):
                _set_servo(pwm, step[j])
            index += 1
            utime.sleep_ms(200)
        stand()
        answer = input("  这个后退距离是否合适？输入 y 确认，其他任意键继续试下一个: ").strip().lower()
        if answer == "y":
            suggested_reverse = reverse_ms
            break

    print()
    _line()
    print("  第二部分：转向步数 TURN_STEPS")
    print("  目标效果：原地转向大约 90°。")

    suggested_turns = 6
    for steps in [4, 6, 8, 10]:
        print("  测试转向步数 {}".format(steps))
        _pause("按 Enter 开始")
        stand()
        run_steps(TURN_L_STEPS, steps, 200)
        stand()
        answer = input("  是否接近 90°？输入 y 确认，其他任意键继续试下一个: ").strip().lower()
        if answer == "y":
            suggested_turns = steps
            break

    for pwm in pwms:
        if pwm:
            pwm.deinit()

    print()
    print("  结果汇总：")
    print("    REVERSE_MS = {}".format(suggested_reverse))
    print("    TURN_STEPS = {}".format(suggested_turns))
    _print_writeback([
        "REVERSE_MS = {}".format(suggested_reverse),
        "TURN_STEPS = {}".format(suggested_turns),
    ])


def main():
    menu = [
        ("1", "舵机测试（SERVO_OFFSETS）", test_servo),
        ("2", "超声波测试（DIST_CLEAR / DIST_TURN / DIST_DANGER）", test_ultrasonic),
        ("3", "LDR 光敏测试（LDR_THRESHOLD）", test_ldr),
        ("4", "颜色传感器测试（RED_THRESHOLD）", test_colour),
        ("5", "状态灯测试", test_led),
        ("6", "超声波云台测试（SCAN_ANGLES）", test_scan_servo),
        ("7", "脱困参数测试（REVERSE_MS / TURN_STEPS）", test_recovery),
    ]

    print()
    print("=" * 60)
    print("  机器狗调试与参数标定工具")
    print("  用途：为 main.py 测参数，不是最终运行程序")
    print("=" * 60)
    print("  运行前请先确认顶部 PIN_* 与 main.py 一致。")

    while True:
        print()
        print("  请选择测试项目：")
        for key, label, _ in menu:
            print("    {}. {}".format(key, label))
        print("    q. 退出")

        choice = input("  输入选项: ").strip().lower()
        matched = False
        for key, _, func in menu:
            if choice == key:
                func()
                matched = True
                break

        if matched:
            continue
        if choice == "q":
            print()
            print("  已退出。记得把建议值填回 main.py。")
            break
        print("  输入无效，请输入 1-7 或 q。")


main()
