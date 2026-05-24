# ============================================================
# main.py — AFSM Robot Dog Controller (单文件完整实现)
# 课程: 4FTC2135-0105-2025 Robot Design and Build Project B
# 严格遵循 AFSM_Documentation_updated.pdf 的状态机逻辑
#
# 使用前:
#   1. 将所有 PIN_xxx 改为实际 GPIO 编号
#   2. 用 test.py 标定 LDR_THRESHOLD / RED_THRESHOLD
#   3. 用 test.py 验证舵机方向、步态和转向
# ============================================================

import utime
from machine import Pin, PWM, ADC, time_pulse_us

# ============================================================
# 1. PIN 定义 — 根据实际接线修改每个 None
# ============================================================
PIN_RR_SERVO    = None   # Right Rear  leg servo,  PWM @ 50Hz
PIN_LR_SERVO    = None   # Left Rear   leg servo,  PWM @ 50Hz
PIN_RF_SERVO    = None   # Right Front leg servo,  PWM @ 50Hz
PIN_LF_SERVO    = None   # Left Front  leg servo,  PWM @ 50Hz
PIN_US_SERVO    = None   # Ultrasonic scan servo,   PWM @ 50Hz
PIN_LDR         = None   # LDR light sensor,  ADC (需外接 10kΩ 下拉到 GND)
PIN_TRIG        = None   # HC-SR04 Trig,  digital out
PIN_ECHO        = None   # HC-SR04 Echo,  digital in
PIN_TCS_S2      = None   # TCS3200 S2,   digital out
PIN_TCS_S3      = None   # TCS3200 S3,   digital out
PIN_TCS_OUT     = None   # TCS3200 OUT,  digital in
PIN_TCS_LED     = None   # TCS3200 LED,  digital out
PIN_STATUS_LED  = None   # Status LED (板载 LED = 25, 或外接 GPIO)

# ============================================================
# 2. 舵机参数
# ============================================================
SERVO_FREQ       = 50       # Hz
MIN_DUTY         = 1638     # 0°   = 0.5ms  pulse (duty_u16)
MAX_DUTY         = 8192     # 180° = 2.5ms  pulse (duty_u16)
MID_ANGLE        = 90       # 舵机中立点

# 各腿 offset（机械偏差补偿；若某腿歪斜，微调对应值 ±2~8°）
RF_OFFSET = 0
RR_OFFSET = 0
LF_OFFSET = 0
LR_OFFSET = 0
US_OFFSET = 0

# ============================================================
# 3. 步态参数 — 参考值可微调
# ============================================================
ARC              = 18       # 腿部摆动弧度 (°), 参考范围 18~26
STEP_TIME_FAST   = 200      # ms, Explore 每步间隔
STEP_TIME_SLOW   = 400      # ms, Low Charge 每步间隔
TURN_STEPS       = 6        # 90° 原地转弯所需步数
VEER_STEPS       = 4        # 行进中微调所需步数

# ============================================================
# 4. 超声波扫描参数
# ============================================================
SCAN_ANGLES         = [30, 90, 150]  # LD(左), FD(前), RD(右) 角度
IDX_LD, IDX_FD, IDX_RD = 0, 1, 2
SCAN_INTERVAL_FAST  = 150  # ms, Explore 中每步扫描间隔
SCAN_INTERVAL_SLOW  = 300  # ms, Low Charge 中每步扫描间隔
ULTRASONIC_SAMPLES  = 5    # 每次测距的采样次数

# ============================================================
# 5. 障碍物距离阈值 (cm) — 严格遵循 PDF §3.3
# ============================================================
OBSTACLE_CLEAR   = 15       # FD >= 15 → 安全，直行 Forward
OBSTACLE_TURN    = 10       # FD < 10 → Turn; 10 <= FD < 15 → Veer
DANGER_CLOSE     = 5        # 任一方向 < 5 → 立即触发 Recovery

# ============================================================
# 6. Mission 定时器 (ms) — 严格遵循 PDF §4.2
# ============================================================
EXPLORE_TIME_MS  = 120000   # 2 分钟，探索计时器
CHARGE_TIME_MS   = 60000    # 1 分钟，充电计时器
BLINK_INTERVAL   = 500      # ms, Target Detected 状态 LED 闪烁

# ============================================================
# 7. 传感器参数
# ============================================================
COLOR_READ_INTERVAL = 500   # ms, 颜色传感器读取间隔
LDR_READ_INTERVAL   = 100   # ms, LDR 读取间隔
COLOR_SAMPLES       = 10    # 颜色传感器每次采样的次数
LDR_SAMPLES         = 5     # LDR 每次采样的次数
COLOR_PULSE_TIMEOUT = 5000  # us, time_pulse_us 超时

# ============================================================
# 8. 阈值 — 必须用 test.py 在实际环境标定后填入!
# ============================================================
LDR_THRESHOLD      = 50000  # !! [需标定] LDR bright light 阈值
RED_THRESHOLD      = 50     # !! [需标定] 红色判定: r < g-margin and r < b-margin

# ============================================================
# 9. 计数阈值 & Recovery — 严格遵循 PDF §4.2
# ============================================================
TURN_COUNT_MAX     = 3      # 连续 Turn 失败 >= 3 → Recovery
RECOVERY_COUNT_MAX = 3      # 连续 Recovery >= 3 → Fault Stop
RECOVERY_REVERSE_MS = 800   # Recovery 阶段后退时长 (ms)

# ============================================================
# 10. 硬件初始化
# ============================================================

_servos   = {}
_sensors  = {}
_offsets  = {}  # servo object → offset angle

def _safe_pwm(pin_num):
    if pin_num is None:
        return None
    p = PWM(Pin(pin_num))
    p.freq(SERVO_FREQ)
    return p

def init_hardware():
    global _servos, _sensors, _offsets

    # 腿部舵机 + offset
    leg_map = [
        ("RF", PIN_RF_SERVO, RF_OFFSET),
        ("RR", PIN_RR_SERVO, RR_OFFSET),
        ("LF", PIN_LF_SERVO, LF_OFFSET),
        ("LR", PIN_LR_SERVO, LR_OFFSET),
    ]
    for name, pin_num, offset in leg_map:
        p = _safe_pwm(pin_num)
        if p:
            _servos[name] = p
            _offsets[p] = offset

    # 超声波云台舵机
    p = _safe_pwm(PIN_US_SERVO)
    if p:
        _servos["US"] = p
        _offsets[p] = US_OFFSET

    # LDR
    if PIN_LDR is not None:
        _sensors["ldr"] = ADC(Pin(PIN_LDR))

    # HC-SR04
    if PIN_TRIG is not None and PIN_ECHO is not None:
        _sensors["trig"] = Pin(PIN_TRIG, Pin.OUT)
        _sensors["echo"] = Pin(PIN_ECHO, Pin.IN)

    # TCS3200 (S0→3V3, S1→GND 硬件固定)
    if all(p is not None for p in [PIN_TCS_S2, PIN_TCS_S3, PIN_TCS_OUT, PIN_TCS_LED]):
        _sensors["s2"]     = Pin(PIN_TCS_S2, Pin.OUT)
        _sensors["s3"]     = Pin(PIN_TCS_S3, Pin.OUT)
        _sensors["sig"]    = Pin(PIN_TCS_OUT, Pin.IN)
        _sensors["led"]    = Pin(PIN_TCS_LED, Pin.OUT)
        _sensors["has_tcs"] = True
    else:
        _sensors["has_tcs"] = False

    # Status LED
    if PIN_STATUS_LED is not None:
        _sensors["status_led"] = Pin(PIN_STATUS_LED, Pin.OUT)
        _sensors["has_led"] = True
    else:
        _sensors["has_led"] = False

    print("[HW] Servos:", list(_servos.keys()))
    print("[HW] LDR:", "ok" if "ldr" in _sensors else "missing")
    print("[HW] Ultrasonic:", "ok" if "trig" in _sensors else "missing")
    print("[HW] TCS3200:", "ok" if _sensors.get("has_tcs") else "missing")
    print("[HW] Status LED:", "ok" if _sensors.get("has_led") else "missing")

# ============================================================
# 11. 工具函数
# ============================================================

def set_position(servo, angle):
    """舵机转到指定角度 (clamp 0-180)，自动叠加 offset"""
    if servo is None:
        return
    angle += _offsets.get(servo, 0)
    angle = max(0, min(180, angle))
    duty = int(MIN_DUTY + (angle / 180) * (MAX_DUTY - MIN_DUTY))
    servo.duty_u16(duty)

def all_legs_to(angle):
    """所有腿部舵机归位"""
    for k in ("RF", "RR", "LF", "LR"):
        if k in _servos:
            set_position(_servos[k], angle)

def _led_off():
    """关闭 status LED"""
    if _sensors.get("has_led"):
        _sensors["status_led"].off()

def _random_dir():
    """LD==RD 时随机选方向"""
    # MicroPython 无 random.randint; 用 ticks_us 末位模拟
    return "left" if (utime.ticks_us() & 1) == 0 else "right"

# ============================================================
# 12. 传感器函数
# ============================================================

def read_ldr():
    """LDR 读取 (多次平均)，返回 0-65535；值越大越亮。未配置返回 0"""
    if "ldr" not in _sensors:
        return 0
    total = 0
    for _ in range(LDR_SAMPLES):
        total += _sensors["ldr"].read_u16()
    return total // LDR_SAMPLES

def is_bright():
    """当前 LDR 值是否超过阈值"""
    return read_ldr() > LDR_THRESHOLD

def get_distance():
    """HC-SR04 超声波测距 (cm)，多次采样取平均。无回波返回 100"""
    if "trig" not in _sensors:
        return 100
    trig = _sensors["trig"]
    echo = _sensors["echo"]
    total = 0
    valid = 0
    for _ in range(ULTRASONIC_SAMPLES):
        trig.value(0)
        utime.sleep_us(2)
        trig.value(1)
        utime.sleep_us(10)
        trig.value(0)
        pulse = time_pulse_us(echo, 1, 12000)
        if pulse > 0:
            total += pulse
            valid += 1
        utime.sleep_ms(10)
    if valid == 0:
        return 100
    return (total / valid * 0.0343) / 2

def _color_pulse():
    """读一次 TCS3200 当前通道的 pulse 宽度"""
    p = time_pulse_us(_sensors["sig"], 1, COLOR_PULSE_TIMEOUT)
    return p if p > 0 else COLOR_PULSE_TIMEOUT

def get_averaged_colour():
    """TCS3200 多次采样取平均 RGB。返回 (r, g, b)，值越小该颜色越强"""
    if not _sensors.get("has_tcs"):
        return 999, 999, 999
    s2, s3 = _sensors["s2"], _sensors["s3"]
    _sensors["led"].on()
    utime.sleep_ms(50)
    tr = tg = tb = 0
    try:
        for _ in range(COLOR_SAMPLES):
            s2.low();  s3.low();   tr += _color_pulse()
            s2.low();  s3.high();  tb += _color_pulse()
            s2.high(); s3.high();  tg += _color_pulse()
            utime.sleep_ms(2)
    finally:
        _sensors["led"].off()
    n = COLOR_SAMPLES
    return tr // n, tg // n, tb // n

def is_red_detected(r, g, b):
    """红色判定：r 明显小于 g 和 b"""
    return r < (g - RED_THRESHOLD) and r < (b - RED_THRESHOLD)

# ============================================================
# 13. 步态计算 & 运动执行
# ============================================================

def _step_angles():
    m = MID_ANGLE
    a = ARC
    return (
        int(m - a / 2),  # one
        int(m - a / 6),  # two
        int(m + a / 6),  # three
        int(m + a / 2),  # four
    )

def build_forward_steps():
    """返回四步前进步态 [(RF,RR,LF,LR), ...]"""
    one, two, three, four = _step_angles()
    return [
        (four,  one,   three, two  ),
        (three, four,  four,  three),
        (two,   three, one,   four ),
        (one,   two,   two,   one  ),
    ]

def build_turn_steps(direction):
    """返回原地旋转步态 (2 步循环)。direction: 'left' | 'right'"""
    m = MID_ANGLE
    a = ARC
    fwd  = int(m + a / 2)
    back = int(m - a / 2)
    if direction == "left":
        # 右侧腿前推 + 左侧腿后拉 → 左转
        return [(fwd, back, back, fwd), (back, fwd, fwd, back)]
    else:
        return [(back, fwd, fwd, back), (fwd, back, back, fwd)]

def build_veer_steps(direction):
    """返回行进中微调步态 (2 步循环)。一侧步幅大、一侧小 → 微调方向"""
    m = MID_ANGLE
    a = ARC
    bias = int(a / 4)
    if direction == "left":
        rf_f, rr_b = int(m + a/2), int(m - a/2)             # 右侧大步
        lf_f, lr_b = int(m + a/2 - bias), int(m - a/2 + bias)  # 左侧小步
    else:
        rf_f, rr_b = int(m + a/2 - bias), int(m - a/2 + bias)
        lf_f, lr_b = int(m + a/2), int(m - a/2)
    rf_m, rr_m = int(m + a/6), int(m - a/6)
    lf_m, lr_m = int(m + a/6), int(m - a/6)
    return [
        (rf_f, rr_b, lf_f, lr_b),
        (rf_m, rr_m, lf_m, lr_m),
    ]

def build_reverse_steps():
    """返回后退步态 (4 步循环)"""
    one, two, three, four = _step_angles()
    return [
        (one,  four, two,   three),
        (two,  three, one,  four ),
        (one,  four, two,   three),
        (two,  three, one,   four),
    ]

def _exec_step(step_tuple):
    """执行一步: 写 (RF,RR,LF,LR) 到对应舵机"""
    for key, angle in zip(("RF", "RR", "LF", "LR"), step_tuple):
        if key in _servos:
            set_position(_servos[key], angle)

# ============================================================
# 14. 超声波扫描系统 (non-blocking)
# ============================================================

_scan_phase    = 0     # 0=移动云台去下一个角度, 1=测量当前角度
_scan_index    = 0     # 0→LD, 1→FD, 2→RD
_scan_dists    = [100, 100, 100]  # [LD, FD, RD] cm
_scan_last_ms  = 0
_scan_interval = SCAN_INTERVAL_FAST
_scan_new_data = False  # 新一圈扫描刚完成

def reset_scan():
    global _scan_phase, _scan_index, _scan_dists, _scan_last_ms, _scan_new_data
    _scan_phase  = 0
    _scan_index  = 0
    _scan_dists  = [100, 100, 100]
    _scan_last_ms = utime.ticks_ms()
    _scan_new_data = False

def scan_update(now):
    """Non-blocking 扫描一轮: 交替移动云台→测距。
       当 scan_index 绕回 0 时表示一圈完成，_scan_new_data 置 True"""
    global _scan_phase, _scan_index, _scan_dists, _scan_last_ms, _scan_new_data

    if "US" not in _servos or "trig" not in _sensors:
        return

    _scan_new_data = False
    if utime.ticks_diff(now, _scan_last_ms) < _scan_interval:
        return

    _scan_last_ms = now

    if _scan_phase == 0:
        # 移动云台 → 测量角度
        set_position(_servos["US"], SCAN_ANGLES[_scan_index])
        _scan_phase = 1
    else:
        # 测量 → 存结果 → 推进 index
        _scan_dists[_scan_index] = get_distance()
        _scan_index = (_scan_index + 1) % 3
        _scan_phase = 0
        _scan_new_data = (_scan_index == 0)  # 回到 0 = 完成一圈

def _ld(): return _scan_dists[IDX_LD]
def _fd(): return _scan_dists[IDX_FD]
def _rd(): return _scan_dists[IDX_RD]

# ============================================================
# 15. 避障 evaluate_obstacle() — 严格遵循 PDF §5.5 & §3.3
# ============================================================

_turn_count     = 0
_recovery_count = 0

def _pick_direction(ld, rd, prefix):
    """LD > RD 选左, RD > LD 选右, 相等则随机"""
    if ld > rd: return prefix + "_left"
    if rd > ld: return prefix + "_right"
    return prefix + ("_left" if _random_dir() == "left" else "_right")

def evaluate_obstacle():
    """根据最新 FD/LD/RD 判定避障子状态。
       返回: "forward" | "turn_left" | "turn_right" |
              "veer_left" | "veer_right" | "recovery"
    """
    global _turn_count

    fd, ld, rd = _fd(), _ld(), _rd()

    # Recovery: 三面堵死 / 任一方向<5cm / 连续 Turn 失败>=3
    if ((ld < OBSTACLE_TURN and rd < OBSTACLE_TURN and fd < OBSTACLE_TURN)
        or ld < DANGER_CLOSE or rd < DANGER_CLOSE or fd < DANGER_CLOSE
        or _turn_count >= TURN_COUNT_MAX):
        return "recovery"

    # Turn: FD < 10cm
    if fd < OBSTACLE_TURN:
        return _pick_direction(ld, rd, "turn")

    # Veer: 10cm <= FD < 15cm
    if fd < OBSTACLE_CLEAR:
        return _pick_direction(ld, rd, "veer")

    # 安全
    return "forward"

# ============================================================
# 16. AFSM 全局状态
# ============================================================

M_START           = "start"
M_EXPLORE         = "explore"
M_LOW_CHARGE      = "low_charge"
M_CHARGING        = "charging"
M_TARGET_DETECTED = "target_detected"
M_FAULT_STOP      = "fault_stop"

_mission       = M_START

# 计时
_explore_start  = 0
_charge_start   = 0
_last_step      = 0
_last_color     = 0
_last_ldr       = 0
_last_led       = 0
_led_on         = False

# 步态
_cur_step       = 0
_fwd_steps      = []
_rev_steps      = []
_step_time      = STEP_TIME_FAST
_step_active    = True

# 避障子状态机
_obs_state      = "forward"  # forward | turn_l/r | veer_l/r | recovery
_obs_step       = 0         # 当前子状态已执行步数
_obs_total      = 0         # 当前子状态总步数
_obs_steps      = []        # 当前子状态使用的步态数组
_rec_phase      = 0         # Recovery 阶段: 1=后退, 2=随机转
_rec_start      = 0
_rec_dir        = ""        # Recovery 随机转向方向, 进入 phase 2 时选定

# 传感器缓存
_ldr_bright     = False
_red_found      = False

# ============================================================
# 17. Mission 状态切换
# ============================================================

def _goto(new_state):
    """Mission 状态切换, 同时配置移动参数"""
    global _mission, _explore_start, _charge_start, _step_active
    global _step_time, _scan_interval, _obs_state, _obs_step, _turn_count
    global _recovery_count, _rec_phase, _red_found, _led_on, _last_led

    print(f"[AFSM] {_mission} -> {new_state}")
    _mission = new_state
    now = utime.ticks_ms()

    if new_state == M_EXPLORE:
        _explore_start = now
        _step_time, _scan_interval = STEP_TIME_FAST, SCAN_INTERVAL_FAST
        _step_active, _obs_state, _obs_step, _turn_count = True, "forward", 0, 0
        _recovery_count, _rec_phase, _red_found = 0, 0, False
        reset_scan()

    elif new_state == M_LOW_CHARGE:
        _step_time, _scan_interval = STEP_TIME_SLOW, SCAN_INTERVAL_SLOW
        _step_active, _obs_state, _obs_step, _turn_count = True, "forward", 0, 0
        _rec_phase = 0
        reset_scan()

    elif new_state == M_CHARGING:
        _charge_start = now
        _step_active  = False
        all_legs_to(MID_ANGLE)
        if _sensors.get("has_led"):
            _sensors["status_led"].on()

    elif new_state == M_TARGET_DETECTED:
        _step_active = False
        all_legs_to(MID_ANGLE)
        _led_on   = False
        _last_led = now
        _led_off()

    elif new_state == M_FAULT_STOP:
        _step_active = False
        all_legs_to(MID_ANGLE)

# ============================================================
# 18. 通用步态推进
# ============================================================

def _gait_tick(now, steps, interval):
    """Non-blocking 步态节拍: 每 interval ms 推进一步。
       返回 True 表示实际推进了一步。"""
    global _last_step, _cur_step
    if not _step_active or not steps:
        return False
    if utime.ticks_diff(now, _last_step) >= interval:
        _last_step = now
        _exec_step(steps[_cur_step % len(steps)])
        _cur_step = (_cur_step + 1) % len(steps)
        return True
    return False

# ============================================================
# 19. Recovery 处理
# ============================================================

def _handle_recovery(now):
    """Recovery 脱困序列: 后退 → 随机转弯 → 回到 Scan 重新评估"""
    global _rec_phase, _rec_start, _rec_dir, _obs_state, _obs_step, _obs_steps, _obs_total
    global _recovery_count, _step_active

    if _rec_phase == 0:
        _rec_start = now
        _rec_phase = 1
        _obs_step  = 0
        _recovery_count += 1
        if _recovery_count >= RECOVERY_COUNT_MAX:
            _goto(M_FAULT_STOP)
        return

    elif _rec_phase == 1:
        # 后退 ~0.8s
        if utime.ticks_diff(now, _rec_start) < RECOVERY_REVERSE_MS:
            _gait_tick(now, _rev_steps, STEP_TIME_SLOW)
        else:
            _rec_phase = 2
            _obs_step  = 0
            _rec_dir   = _random_dir()  # 进入转弯前选定一次方向
        return

    elif _rec_phase == 2:
        # 按选定方向原地转弯 90°
        if _obs_step == 0:
            _obs_steps = build_turn_steps(_rec_dir)
            _obs_total = TURN_STEPS
        if _gait_tick(now, _obs_steps, STEP_TIME_SLOW):
            _obs_step += 1
        if _obs_step >= _obs_total:
            _rec_phase   = 0
            _obs_state   = "forward"
            _obs_step    = 0
            _step_active = True
            reset_scan()
        return

# ============================================================
# 20. 避障动作执行 (Explore / Low Charge 共用)
# ============================================================

def _run_obstacle_action(now):
    """根据 _obs_state 执行对应的步态动作"""
    global _obs_state, _obs_step, _obs_steps, _obs_total, _turn_count

    if _obs_state == "forward":
        _gait_tick(now, _fwd_steps, _step_time)

    elif _obs_state in ("turn_left", "turn_right"):
        d = "left" if _obs_state == "turn_left" else "right"
        if _obs_step == 0:
            _obs_steps = build_turn_steps(d)
            _obs_total = TURN_STEPS
        if _gait_tick(now, _obs_steps, STEP_TIME_SLOW):
            _obs_step += 1
        if _obs_step >= _obs_total:
            _turn_count += 1
            _obs_state = "forward"
            _obs_step  = 0
            reset_scan()

    elif _obs_state in ("veer_left", "veer_right"):
        d = "left" if _obs_state == "veer_left" else "right"
        if _obs_step == 0:
            _obs_steps = build_veer_steps(d)
            _obs_total = VEER_STEPS
        if _gait_tick(now, _obs_steps, STEP_TIME_FAST):
            _obs_step += 1
        if _obs_step >= _obs_total:
            # Veer 不递增 turn_count
            _obs_state = "forward"
            _obs_step  = 0
            reset_scan()

    elif _obs_state == "recovery":
        _handle_recovery(now)

# ============================================================
# 21. Explore / Low Charge 共用逻辑
# ============================================================

def _apply_scan_obstacle(now):
    """新一轮扫描完成后更新避障子状态。Forward/Veer 与 Scan 并行可更新; Turn/Recovery 不可中断。"""
    global _obs_state, _obs_step, _turn_count
    if not _scan_new_data or _obs_state in ("turn_left", "turn_right", "recovery"):
        return
    new_obs = evaluate_obstacle()
    _obs_state = new_obs
    _obs_step = 0
    if new_obs == "forward":
        _turn_count = 0  # 路径畅通时清零 Turn 失败计数

def _run_mobile(now):
    """Explore / Low Charge 共用逻辑: 守卫检查 + 避障 + 运动"""
    global _mission
    if _mission == M_EXPLORE:
        if utime.ticks_diff(now, _explore_start) >= EXPLORE_TIME_MS:
            _goto(M_LOW_CHARGE)
            return
    elif _mission == M_LOW_CHARGE:
        if _ldr_bright:
            _goto(M_CHARGING)
            return
    _apply_scan_obstacle(now)
    _run_obstacle_action(now)

# ============================================================
# 22. Charging 状态处理
# ============================================================

def _run_charging(now):
    """模拟充电: 计时满 1 分钟回 Explore, 中途光灭回 Low Charge"""
    global _mission

    elapsed = utime.ticks_diff(now, _charge_start)

    # 充电完成 → Explore (重置 explore_timer)
    if elapsed >= CHARGE_TIME_MS:
        _led_off()
        if _ldr_bright:
            _goto(M_EXPLORE)   # 光照仍在 → 充满回探索
        else:
            _goto(M_LOW_CHARGE)  # 光已灭 → 回低电量
        return

    # 未满 1 分钟时光照中断 → Low Charge
    if not _ldr_bright:
        _led_off()
        _goto(M_LOW_CHARGE)

# ============================================================
# 23. Target Detected & Fault Stop
# ============================================================

def _run_target_detected(now):
    """终止状态: LED 以 500ms 间隔闪烁, 无出边转换"""
    global _last_led, _led_on
    if _sensors.get("has_led"):
        if utime.ticks_diff(now, _last_led) >= BLINK_INTERVAL:
            _last_led = now
            _led_on = not _led_on
            _sensors["status_led"].value(1 if _led_on else 0)

def _run_fault_stop(now):
    """故障停止: 完全锁死, 等待人工干预"""
    pass

# ============================================================
# 24. 主循环
# ============================================================

def _poll_sensors(now):
    """读取 LDR 和颜色传感器, 更新缓存变量"""
    global _last_ldr, _last_color, _ldr_bright, _red_found
    if utime.ticks_diff(now, _last_ldr) >= LDR_READ_INTERVAL:
        _last_ldr, _ldr_bright = now, is_bright()
    if (_sensors.get("has_tcs")
            and _mission not in (M_TARGET_DETECTED, M_CHARGING, M_FAULT_STOP)
            and utime.ticks_diff(now, _last_color) >= COLOR_READ_INTERVAL):
        _last_color = now
        r, g, b = get_averaged_colour()
        if is_red_detected(r, g, b):
            utime.sleep_ms(10)
            r2, g2, b2 = get_averaged_colour()
            if is_red_detected(r2, g2, b2):
                _red_found = True

def main():
    global _last_step, _last_color, _last_ldr, _fwd_steps, _rev_steps

    print("=== AFSM Robot Dog Controller ===")
    print("Initializing...")

    init_hardware()
    _fwd_steps = build_forward_steps()
    _rev_steps = build_reverse_steps()

    # 初始化计时器
    now = utime.ticks_ms()
    _last_step  = now
    _last_color = now
    _last_ldr   = now

    # 进入 Start → 立即转 Explore
    all_legs_to(MID_ANGLE)
    if "US" in _servos:
        set_position(_servos["US"], MID_ANGLE)
    _goto(M_EXPLORE)

    print("Main loop running. Press Ctrl+C to stop.")

    try:
        while True:
            now = utime.ticks_ms()
            _poll_sensors(now)
            # 红色目标检测 → 最高优先级状态切换
            if _red_found and _mission in (M_EXPLORE, M_LOW_CHARGE):
                _goto(M_TARGET_DETECTED)
                _red_found = False
            if _mission in (M_EXPLORE, M_LOW_CHARGE):
                scan_update(now)
                _run_mobile(now)
            elif _mission == M_CHARGING:
                _run_charging(now)
            elif _mission == M_TARGET_DETECTED:
                _run_target_detected(now)
            elif _mission == M_FAULT_STOP:
                _run_fault_stop(now)
            utime.sleep_ms(5)

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        all_legs_to(MID_ANGLE)
        if "US" in _servos:
            set_position(_servos["US"], MID_ANGLE)
        _led_off()
        if _sensors.get("has_tcs"):
            _sensors["led"].off()
        print("All centered. Done.")

# ============================================================
main()
# ============================================================
