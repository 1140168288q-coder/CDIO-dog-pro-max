# =============================================================================
# AFSM Robot Dog Controller — Raspberry Pi Pico 2 (RP2350) — MicroPython
# Course: 4FTC2135-0105-2025 Robot Design and Build Project B, Assignment 2
# =============================================================================
# Mission states:    Start → Explore ↔ Low Charge ↔ Charging
#                    Any active state → Target Detected (terminal)
# Behaviour states:  Forward | Turn L/R | Veer L/R | Recovery (Explore/Low Charge only)
# Priority:          Target Detection > Obstacle Avoidance > Charging Logic > Forward
# =============================================================================

import utime
import machine
import urandom

# =============================================================================
# SECTION 1 — PIN MAPPING
# All None until you confirm wiring.  Fill these before flashing to hardware.
#
# classroom_code.py reference values (use as starting point):
#   RF=10, RR=5, LF=21, LR=2, US_SERVO=18
#   LDR=26, TRIG=14, ECHO=15
#   TCS_S2=10, TCS_S3=11, TCS_OUT=12, TCS_LED=13
#   STATUS_LED=25  (onboard LED on Pico)
# =============================================================================
PIN_RF         = None   # TODO: right-front leg servo
PIN_RR         = None   # TODO: right-rear  leg servo
PIN_LF         = None   # TODO: left-front  leg servo
PIN_LR         = None   # TODO: left-rear   leg servo
PIN_US_SERVO   = None   # TODO: ultrasonic pan servo
PIN_LDR        = None   # TODO: LDR ADC input
PIN_TRIG       = None   # TODO: HC-SR04 trigger
PIN_ECHO       = None   # TODO: HC-SR04 echo
PIN_TCS_S2     = None   # TODO: TCS3200 S2
PIN_TCS_S3     = None   # TODO: TCS3200 S3
PIN_TCS_OUT    = None   # TODO: TCS3200 signal output
PIN_TCS_LED    = None   # TODO: TCS3200 LED enable
PIN_STATUS_LED = None   # TODO: status LED

# =============================================================================
# SECTION 2 — TUNABLE PARAMETERS
# Change these to match your hardware without touching the logic below.
# =============================================================================

# Servo PWM
SERVO_FREQ = 50        # Hz
MIN_DUTY   = 1638      # duty_u16 at 0°  (0.5 ms pulse)
MAX_DUTY   = 8192      # duty_u16 at 180° (2.5 ms pulse)

# Per-servo angle trim in degrees — add to commanded angle (+ or -)
# Order: [RF, RR, LF, LR, US_SERVO]  — TODO: tune on real hardware
SERVO_OFFSETS = [0, 0, 0, 0, 0]

# Gait geometry
MID_ANGLE = 90   # neutral stand position
ARC       = 18   # step sweep half-arc; larger = bigger steps

# Gait timing
STEP_TIME_FAST = 200   # ms per gait step in Explore (full speed)
STEP_TIME_SLOW = 400   # ms per gait step in Low Charge (reduced speed)
VEER_STEP_TIME = 450   # ms per veer step (slow to ease direction change)
TURN_HOLD_MS   = 200   # ms hard stop before starting turn gait
TURN_STEPS     = 6     # gait steps to complete a 90° turn
VEER_STEPS     = 3     # gait steps to complete a 15–30° veer
REVERSE_MS     = 800   # ms of reverse during Recovery

# Ultrasonic scan
SCAN_ANGLES = [30, 90, 150]   # pan servo angles for [LD, FD, RD]
SCAN_FAST   = 150             # ms between scan phases in Explore
SCAN_SLOW   = 300             # ms between scan phases in Low Charge

# Obstacle distance thresholds (cm)
DIST_CLEAR  = 15   # >= 15 cm: path clear, maintain Forward
DIST_TURN   = 10   # <  10 cm: major blockage → Turn (stop + 90°)
DIST_DANGER =  5   # <   5 cm: dangerously close → Recovery

# Mission timers
EXPLORE_MS = 120_000   # 2 minutes before Explore → Low Charge
CHARGE_MS  =  60_000   # 1 minute charging before → Explore
BLINK_MS   =    500    # LED toggle interval in Target Detected

# Sensor tuning
LDR_THRESHOLD = 50_000   # read_u16 level for "bright light detected"
                          # TODO: calibrate with reference/ldr_test.py
RED_THRESHOLD = 50        # TCS3200 pulse-width difference to confirm red
                          # TODO: verify against red target before demo
COLOR_SAMPLES = 10        # samples averaged per colour reading
US_SAMPLES    =  5        # samples averaged per distance reading
US_TIMEOUT    = 12_000    # µs echo timeout (~2 m range)
RED_CHECK_MS  =   100     # min ms between TCS3200 checks
LDR_CHECK_MS  =   200     # min ms between LDR checks

# Safety counters
TURN_MAX     = 3   # failed turns before → Recovery
RECOVERY_MAX = 3   # Recovery attempts before → Fault Stop

# =============================================================================
# SECTION 3 — GAIT STEP TABLES
# Each row: (RF_angle, RR_angle, LF_angle, LR_angle)
# =============================================================================
_a = MID_ANGLE - ARC // 2   # smallest angle
_b = MID_ANGLE - ARC // 6
_c = MID_ANGLE + ARC // 6
_d = MID_ANGLE + ARC // 2   # largest angle
_e = ARC // 4                # veer bias

FORWARD_STEPS = [
    (_d, _a, _c, _b),
    (_c, _d, _d, _c),
    (_b, _c, _a, _d),
    (_a, _b, _b, _a),
]
REVERSE_STEPS = list(reversed(FORWARD_STEPS))

# Right side forward, left side backward → turns robot left
TURN_L_STEPS = [
    (_d, _d, _a, _a),
    (_a, _a, _d, _d),
]
# Left side forward, right side backward → turns robot right
TURN_R_STEPS = [
    (_a, _a, _d, _d),
    (_d, _d, _a, _a),
]

# Asymmetric walk: slight left bias
VEER_L_STEPS = [
    (_c + _e, _b,     _b,     _c + _e),
    (_b,      _c + _e, _c + _e, _b   ),
]
# Asymmetric walk: slight right bias
VEER_R_STEPS = [
    (_b,      _c + _e, _c + _e, _b   ),
    (_c + _e, _b,     _b,     _c + _e),
]

# =============================================================================
# SECTION 4 — HARDWARE HANDLES
# Populated by init_hardware(); all None until then.
# =============================================================================
_pwm_rf = _pwm_rr = _pwm_lf = _pwm_lr = None
_pwm_us = None
_adc_ldr  = None
_pin_trig = _pin_echo = None
_pin_s2 = _pin_s3 = _pin_tcs_out = _pin_tcs_led = None
_pin_led = None

def init_hardware():
    global _pwm_rf, _pwm_rr, _pwm_lf, _pwm_lr, _pwm_us
    global _adc_ldr, _pin_trig, _pin_echo
    global _pin_s2, _pin_s3, _pin_tcs_out, _pin_tcs_led, _pin_led

    def _make_pwm(pin_num, label):
        if pin_num is None:
            print("[WARN] Skipping", label, "— pin not set")
            return None
        p = machine.PWM(machine.Pin(pin_num))
        p.freq(SERVO_FREQ)
        return p

    _pwm_rf = _make_pwm(PIN_RF, "RF servo")
    _pwm_rr = _make_pwm(PIN_RR, "RR servo")
    _pwm_lf = _make_pwm(PIN_LF, "LF servo")
    _pwm_lr = _make_pwm(PIN_LR, "LR servo")
    _pwm_us = _make_pwm(PIN_US_SERVO, "US pan servo")

    if PIN_LDR is not None:
        _adc_ldr = machine.ADC(machine.Pin(PIN_LDR))
    else:
        print("[WARN] Skipping LDR — pin not set")

    if PIN_TRIG is not None and PIN_ECHO is not None:
        _pin_trig = machine.Pin(PIN_TRIG, machine.Pin.OUT)
        _pin_echo = machine.Pin(PIN_ECHO, machine.Pin.IN)
    else:
        print("[WARN] Skipping HC-SR04 — pin(s) not set")

    if None not in (PIN_TCS_S2, PIN_TCS_S3, PIN_TCS_OUT, PIN_TCS_LED):
        _pin_s2      = machine.Pin(PIN_TCS_S2,  machine.Pin.OUT)
        _pin_s3      = machine.Pin(PIN_TCS_S3,  machine.Pin.OUT)
        _pin_tcs_out = machine.Pin(PIN_TCS_OUT, machine.Pin.IN)
        _pin_tcs_led = machine.Pin(PIN_TCS_LED, machine.Pin.OUT)
        _pin_tcs_led.value(0)
    else:
        print("[WARN] Skipping TCS3200 — pin(s) not set")

    if PIN_STATUS_LED is not None:
        _pin_led = machine.Pin(PIN_STATUS_LED, machine.Pin.OUT)
        _pin_led.value(0)
    else:
        print("[WARN] Skipping status LED — pin not set")

    # Centre pan servo
    if _pwm_us:
        _set_servo_raw(_pwm_us, 90, 4)
    print("[INIT] Hardware ready")

# =============================================================================
# SECTION 5 — SERVO HELPERS
# =============================================================================
def _set_servo_raw(pwm, angle, offset_idx=None):
    if pwm is None:
        return
    if offset_idx is not None:
        angle += SERVO_OFFSETS[offset_idx]
    angle = max(0, min(180, angle))
    pwm.duty_u16(int(MIN_DUTY + (angle / 180.0) * (MAX_DUTY - MIN_DUTY)))

def _apply_step(step):
    """Send one gait step tuple (RF, RR, LF, LR) to the leg servos."""
    _set_servo_raw(_pwm_rf, step[0], 0)
    _set_servo_raw(_pwm_rr, step[1], 1)
    _set_servo_raw(_pwm_lf, step[2], 2)
    _set_servo_raw(_pwm_lr, step[3], 3)

def stop_legs():
    """Return all legs to neutral stand — call before Turn and on state exits."""
    _set_servo_raw(_pwm_rf, MID_ANGLE, 0)
    _set_servo_raw(_pwm_rr, MID_ANGLE, 1)
    _set_servo_raw(_pwm_lf, MID_ANGLE, 2)
    _set_servo_raw(_pwm_lr, MID_ANGLE, 3)

# =============================================================================
# SECTION 6 — SENSOR READING
# =============================================================================
def get_distance():
    """HC-SR04 averaged distance in cm. Returns 100 if no sensor or timeout."""
    if _pin_trig is None:
        return 100
    total = 0; count = 0
    for _ in range(US_SAMPLES):
        _pin_trig.value(0); utime.sleep_us(2)
        _pin_trig.value(1); utime.sleep_us(10)
        _pin_trig.value(0)
        p = machine.time_pulse_us(_pin_echo, 1, US_TIMEOUT)
        if p > 0:
            total += (p * 0.0343) / 2.0
            count += 1
        utime.sleep_ms(10)
    return round(total / count, 1) if count else 100

def _read_tcs_channel(s2, s3):
    _pin_s2.value(s2); _pin_s3.value(s3)
    utime.sleep_ms(1)
    return machine.time_pulse_us(_pin_tcs_out, 0, 200_000)

def get_averaged_colour():
    """Return (r, g, b) average pulse widths. Smaller = stronger colour component."""
    if _pin_tcs_led is None:
        return (9999, 9999, 9999)
    _pin_tcs_led.value(1)
    utime.sleep_ms(50)   # LED settle — the only intentional blocking sleep in this file
    r = g = b = 0
    for _ in range(COLOR_SAMPLES):
        r += _read_tcs_channel(0, 0)   # Red:   S2=0 S3=0
        g += _read_tcs_channel(1, 1)   # Green: S2=1 S3=1
        b += _read_tcs_channel(0, 1)   # Blue:  S2=0 S3=1
    _pin_tcs_led.value(0)
    n = COLOR_SAMPLES
    return (r // n, g // n, b // n)

def is_red(r, g, b):
    """True when red pulse is clearly shorter (stronger) than green and blue."""
    return r < (g - RED_THRESHOLD) and r < (b - RED_THRESHOLD)

def read_ldr():
    """Raw LDR ADC value (0–65535). Higher = brighter. Returns 0 if no sensor."""
    return _adc_ldr.read_u16() if _adc_ldr else 0

# =============================================================================
# SECTION 7 — GLOBAL STATE VARIABLES
# =============================================================================

# Mission state names
M_START      = "START"
M_EXPLORE    = "EXPLORE"
M_LOW_CHARGE = "LOW_CHARGE"
M_CHARGING   = "CHARGING"
M_TARGET     = "TARGET_DETECTED"
M_FAULT      = "FAULT_STOP"

# Behaviour state names (active inside Explore / Low Charge)
B_FORWARD  = "FORWARD"
B_TURN_L   = "TURN_LEFT"
B_TURN_R   = "TURN_RIGHT"
B_VEER_L   = "VEER_LEFT"
B_VEER_R   = "VEER_RIGHT"
B_RECOVERY = "RECOVERY"

mission   = M_START
behaviour = B_FORWARD

# Latest scan readings (cm) — initialised safe
fd = 100; ld = 100; rd = 100

# Scan internals
_scan_index      = 0       # 0=LD, 1=FD, 2=RD
_scan_phase      = 0       # 0=move servo, 1=measure
_scan_timer_last = 0
scan_fresh       = False   # True once full LD/FD/RD sweep is done

# Gait internals
_gait_table      = FORWARD_STEPS
_gait_index      = 0
_gait_steps_done = 0
_gait_timer_last = 0

# Manoeuvre internals
_turn_phase        = 0   # 0=entry(stop), 1=stepping
_turn_hold_start   = 0
_turn_needs_check  = False
_recovery_phase    = 0   # 0=init, 1=reverse, 2=turn, 3=done
_recovery_start    = 0
_recovery_turn_tbl = TURN_L_STEPS

# Mission timers
_explore_start  = 0
_charging_start = 0

# Status LED
_led_on         = False
_led_timer_last = 0

# Sensor poll rate limiters
_red_last = 0
_ldr_last = 0

# Counters
turn_count     = 0
recovery_count = 0

# =============================================================================
# SECTION 8 — SCAN UPDATE  (non-blocking)
# Only called when behaviour == B_FORWARD (paused during manoeuvres)
# =============================================================================
def scan_update(now):
    global _scan_phase, _scan_index, scan_fresh, _scan_timer_last, fd, ld, rd

    interval = SCAN_FAST if mission == M_EXPLORE else SCAN_SLOW
    if utime.ticks_diff(now, _scan_timer_last) < interval:
        return
    _scan_timer_last = now

    if _scan_phase == 0:
        # Move pan servo to current position
        _set_servo_raw(_pwm_us, SCAN_ANGLES[_scan_index], 4)
        _scan_phase = 1
    else:
        # Take measurement and store
        dist = get_distance()
        if   _scan_index == 0: ld = dist
        elif _scan_index == 1: fd = dist
        else:                  rd = dist; scan_fresh = True   # full sweep done
        _scan_index = (_scan_index + 1) % 3
        _scan_phase = 0

def _reset_scan():
    """Restart the scan cycle from LD after completing a manoeuvre."""
    global _scan_index, _scan_phase, scan_fresh
    _scan_index = 0
    _scan_phase = 0
    scan_fresh  = False

# =============================================================================
# SECTION 9 — GAIT STEP EXECUTION  (non-blocking)
# =============================================================================
def _do_step(now, step_time):
    """Execute one gait step when step_time elapses. Returns True on step fired."""
    global _gait_index, _gait_timer_last, _gait_steps_done
    if utime.ticks_diff(now, _gait_timer_last) < step_time:
        return False
    _gait_timer_last = now
    _apply_step(_gait_table[_gait_index % len(_gait_table)])
    _gait_index      = (_gait_index + 1) % len(_gait_table)
    _gait_steps_done += 1
    return True

def _load_gait(table):
    """Switch active gait table and reset counters."""
    global _gait_table, _gait_index, _gait_steps_done
    _gait_table      = table
    _gait_index      = 0
    _gait_steps_done = 0

def _prepare_forward():
    """Reset gait and scan state when resuming normal movement."""
    _load_gait(FORWARD_STEPS)
    _reset_scan()

def _finish_manoeuvre():
    """Return to Forward after any manoeuvre completes."""
    global behaviour
    behaviour = B_FORWARD
    _prepare_forward()

# =============================================================================
# SECTION 10 — OBSTACLE EVALUATION
# Called once scan_fresh and behaviour == B_FORWARD
# Reads fd/ld/rd and sets behaviour + prepares gait table
# =============================================================================
def _coin():
    """Return 0 or 1 randomly for tie-breaking (LD == RD)."""
    try:    return urandom.getrandbits(1)
    except: return utime.ticks_us() & 1

def evaluate_obstacle():
    global behaviour, turn_count, recovery_count
    global _turn_phase, _recovery_phase

    # ── Priority 1 in avoidance: Recovery ────────────────────────────────────
    boxed  = (ld < DIST_TURN and rd < DIST_TURN and fd < DIST_TURN)
    danger = (ld < DIST_DANGER or rd < DIST_DANGER or fd < DIST_DANGER)
    if boxed or danger or turn_count >= TURN_MAX:
        recovery_count += 1
        if recovery_count >= RECOVERY_MAX:
            _goto_fault_stop()
            return
        behaviour       = B_RECOVERY
        _recovery_phase = 0
        return

    # ── Priority 2: Turn (major blockage) ────────────────────────────────────
    if fd < DIST_TURN:
        if ld > rd:     b = B_TURN_L
        elif rd > ld:   b = B_TURN_R
        else:           b = B_TURN_L if _coin() == 0 else B_TURN_R
        behaviour    = b
        _turn_phase  = 0
        _load_gait(TURN_L_STEPS if b == B_TURN_L else TURN_R_STEPS)
        return

    # ── Priority 3: Veer (minor blockage) ────────────────────────────────────
    if fd < DIST_CLEAR:
        if ld > rd:     b = B_VEER_L
        elif rd > ld:   b = B_VEER_R
        else:           b = B_VEER_L if _coin() == 0 else B_VEER_R
        behaviour = b
        _load_gait(VEER_L_STEPS if b == B_VEER_L else VEER_R_STEPS)
        return

    # ── Clear path: maintain Forward ─────────────────────────────────────────
    turn_count = 0

# =============================================================================
# SECTION 11 — BEHAVIOUR HANDLERS
# =============================================================================
def handle_forward(now):
    step_time = STEP_TIME_FAST if mission == M_EXPLORE else STEP_TIME_SLOW
    _do_step(now, step_time)

def handle_turn(now):
    """Phase 0: hard stop + hold.  Phase 1: execute turn gait."""
    global _turn_phase, _turn_hold_start, _turn_needs_check

    if _turn_phase == 0:
        stop_legs()
        _turn_hold_start = now
        _turn_phase = 1
        return

    # Hold stop briefly before stepping
    if utime.ticks_diff(now, _turn_hold_start) < TURN_HOLD_MS:
        return

    if _do_step(now, STEP_TIME_FAST):
        if _gait_steps_done >= TURN_STEPS:
            _turn_needs_check = True
            _finish_manoeuvre()

def handle_veer(now):
    """Slow asymmetric walk — no stop, just gradual heading change."""
    _do_step(now, VEER_STEP_TIME)
    if _gait_steps_done >= VEER_STEPS:
        _finish_manoeuvre()   # Veer never increments turn_count

def handle_recovery(now):
    """Phase 0: init/stop.  Phase 1: reverse 0.8s.  Phase 2: random turn."""
    global _recovery_phase, _recovery_start, _recovery_turn_tbl

    if _recovery_phase == 0:
        stop_legs()
        _recovery_turn_tbl = TURN_L_STEPS if _coin() == 0 else TURN_R_STEPS
        _load_gait(REVERSE_STEPS)
        _recovery_start = now
        _recovery_phase = 1

    elif _recovery_phase == 1:
        _do_step(now, STEP_TIME_FAST)
        if utime.ticks_diff(now, _recovery_start) >= REVERSE_MS:
            _load_gait(_recovery_turn_tbl)
            _recovery_phase = 2

    elif _recovery_phase == 2:
        _do_step(now, STEP_TIME_FAST)
        if _gait_steps_done >= TURN_STEPS:
            _recovery_phase = 3

    else:   # phase 3: done
        _finish_manoeuvre()

# =============================================================================
# SECTION 12 — MISSION STATE TRANSITIONS
# Each goto_* function sets mission, resets relevant timers and counters
# =============================================================================
def _set_led(val):
    if _pin_led: _pin_led.value(val)

def goto_explore():
    global mission, behaviour, _explore_start, turn_count, recovery_count
    global _turn_phase, _turn_needs_check, _recovery_phase
    mission        = M_EXPLORE
    behaviour      = B_FORWARD
    _explore_start = utime.ticks_ms()
    turn_count     = 0
    recovery_count = 0
    _turn_phase       = 0
    _turn_needs_check = False
    _recovery_phase   = 0
    _prepare_forward()
    _set_led(0)
    print("[STATE] → Explore")

def goto_low_charge():
    global mission, behaviour, turn_count, recovery_count
    global _turn_phase, _turn_needs_check, _recovery_phase
    mission        = M_LOW_CHARGE
    behaviour      = B_FORWARD
    turn_count     = 0
    recovery_count = 0
    _turn_phase       = 0
    _turn_needs_check = False
    _recovery_phase   = 0
    _prepare_forward()
    _set_led(0)
    print("[STATE] → Low Charge")

def goto_charging():
    global mission, behaviour, _charging_start, _turn_needs_check
    mission          = M_CHARGING
    behaviour        = B_FORWARD
    _charging_start  = utime.ticks_ms()
    _turn_needs_check = False
    stop_legs()
    _set_led(1)   # LED steady on = charging
    print("[STATE] → Charging")

def goto_target_detected():
    global mission, behaviour, _led_on, _led_timer_last, _turn_needs_check
    mission         = M_TARGET
    behaviour       = B_FORWARD
    _led_on         = False
    _led_timer_last = utime.ticks_ms()
    _turn_needs_check = False
    stop_legs()
    _set_led(0)
    print("[STATE] → TARGET DETECTED — LED blinking")

def _goto_fault_stop():
    global mission, behaviour, _turn_needs_check
    mission = M_FAULT
    behaviour = B_FORWARD
    _turn_needs_check = False
    stop_legs()
    _set_led(0)
    print("[STATE] → FAULT STOP — awaiting human intervention")

def start_ready():
    """Hook for future physical start switch; return True when mission may begin."""
    return True

def handle_start(now):
    """Start state: initialise hardware has completed, then wait for start condition."""
    if start_ready():
        goto_explore()

# =============================================================================
# SECTION 13 — PERIODIC SENSOR CHECKS  (rate-limited)
# =============================================================================
def _poll_red(now):
    """Check TCS3200 at most every RED_CHECK_MS. Returns True if red confirmed."""
    global _red_last
    if utime.ticks_diff(now, _red_last) < RED_CHECK_MS:
        return False
    _red_last = now
    if _pin_tcs_led is None:
        return False
    r, g, b = get_averaged_colour()
    if is_red(r, g, b):
        utime.sleep_ms(10)          # brief delay for double-confirmation
        r2, g2, b2 = get_averaged_colour()
        return is_red(r2, g2, b2)
    return False

def _poll_ldr(now):
    """Check LDR at most every LDR_CHECK_MS. Returns value, or None if not due."""
    global _ldr_last
    if utime.ticks_diff(now, _ldr_last) < LDR_CHECK_MS:
        return None
    _ldr_last = now
    return read_ldr()

def _handle_led_blink(now):
    """Non-blocking 500ms LED toggle for Target Detected state."""
    global _led_on, _led_timer_last
    if utime.ticks_diff(now, _led_timer_last) >= BLINK_MS:
        _led_timer_last = now
        _led_on = not _led_on
        _set_led(1 if _led_on else 0)

def _handle_charging(now):
    """Charging state handler: monitor LDR; exit when timer fires or light lost."""
    ldr = _poll_ldr(now)
    if ldr is not None and ldr < LDR_THRESHOLD:
        # Light interrupted before 1 minute → back to Low Charge
        print("[CHARGE] Light lost — returning to Low Charge")
        goto_low_charge()
        return
    if utime.ticks_diff(now, _charging_start) >= CHARGE_MS:
        # 1-minute timer expired — verify light still present
        if read_ldr() >= LDR_THRESHOLD:
            print("[CHARGE] Complete — returning to Explore")
            goto_explore()
        else:
            print("[CHARGE] Timer expired, no light — Low Charge")
            goto_low_charge()

# =============================================================================
# SECTION 14 — MAIN LOOP
# =============================================================================
def main():
    global scan_fresh, turn_count, _turn_needs_check

    init_hardware()
    print("[STATE] → Start")

    while True:
        now = utime.ticks_ms()

        # ── PRIORITY 1: Red target (checked every tick, never gated) ─────────
        if mission not in (M_TARGET, M_FAULT):
            if _poll_red(now):
                goto_target_detected()
                utime.sleep_ms(5)
                continue

        # ── Mission state dispatch ────────────────────────────────────────────
        if mission == M_START:
            handle_start(now)

        elif mission == M_TARGET:
            _handle_led_blink(now)

        elif mission == M_FAULT:
            stop_legs()   # stay stopped, await human

        elif mission == M_CHARGING:
            _handle_charging(now)

        elif mission in (M_EXPLORE, M_LOW_CHARGE):

            # Scan runs only while walking Forward (paused during manoeuvres)
            if behaviour == B_FORWARD:
                scan_update(now)

            # Low Charge: check LDR for light source → Charging
            if mission == M_LOW_CHARGE:
                ldr = _poll_ldr(now)
                if ldr is not None and ldr >= LDR_THRESHOLD:
                    goto_charging()
                    utime.sleep_ms(5)
                    continue

            # Explore: check 2-minute timeout → Low Charge
            if mission == M_EXPLORE:
                if utime.ticks_diff(now, _explore_start) >= EXPLORE_MS:
                    goto_low_charge()
                    utime.sleep_ms(5)
                    continue

            # Evaluate obstacles once fresh scan data is available (Forward only)
            if behaviour == B_FORWARD and scan_fresh:
                scan_fresh = False
                if _turn_needs_check:
                    if fd < DIST_TURN:
                        turn_count += 1
                    else:
                        turn_count = 0
                    _turn_needs_check = False
                evaluate_obstacle()   # may change behaviour away from Forward

            # Execute current behaviour
            if behaviour == B_FORWARD:
                handle_forward(now)
            elif behaviour in (B_TURN_L, B_TURN_R):
                handle_turn(now)
            elif behaviour in (B_VEER_L, B_VEER_R):
                handle_veer(now)
            elif behaviour == B_RECOVERY:
                handle_recovery(now)

        utime.sleep_ms(5)

# =============================================================================
main()
