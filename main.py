from motor_driver import motor_driver
from encoder      import encoder
from task_motor   import task_motor
from task_user    import task_user
from task_share   import Share, Queue
from cotask       import Task, task_list
from gc           import collect

from line_sensor import LineSensorArray
try:
    from bno055 import BNO055
except ImportError:
    BNO055 = None
    print("bno055.py not found -- IMU disabled")

from pyb import Timer, Pin, I2C
from utime import ticks_us, ticks_diff
import micropython
import math

# ============================================================
# HARDWARE
# ============================================================
pwm_tim  = Timer(3, freq=20000)
encL_tim = Timer(2, prescaler=0, period=0xFFFF)
encR_tim = Timer(1, prescaler=0, period=0xFFFF)

leftMotor  = motor_driver(PWM_pin=Pin.cpu.A6, DIR_pin=Pin.cpu.B5, nSLP_pin=Pin.cpu.B6, tim=pwm_tim, chan=1)
rightMotor = motor_driver(PWM_pin=Pin.cpu.A7, DIR_pin=Pin.cpu.B3, nSLP_pin=Pin.cpu.B4, tim=pwm_tim, chan=2)
leftMotor.enable();  rightMotor.enable()
leftMotor.set_effort(0); rightMotor.set_effort(0)

leftEncoder  = encoder(encL_tim, Pin.cpu.A1, Pin.cpu.A0)
rightEncoder = encoder(encR_tim, Pin.cpu.A8, Pin.cpu.A9)

try:
    _i2c = I2C(1, I2C.MASTER, baudrate=400000)
    imu  = BNO055(_i2c, reset_pin=Pin.cpu.A10, do_reset=True)
    imu.set_mode(0x0C)
    _imu_ok = True
    print("IMU OK")
except Exception as e:
    imu = None; _imu_ok = False
    print("IMU FAILED:", e)

# ============================================================
# SHARES & QUEUES
# ============================================================
leftMotorGo  = Share("B", name="LMotGo");  leftMotorGo.put(False)
rightMotorGo = Share("B", name="RMotGo");  rightMotorGo.put(False)
Kp_share = Share("f", name="Kp");  Kp_share.put(0.2)
Ki_share = Share("f", name="Ki");  Ki_share.put(0.0)
SP_share = Share("f", name="SP");  SP_share.put(0.0)

N = 300
leftData  = Queue("f", N, name="LVel");  leftTime  = Queue("L", N, name="LTime")
rightData = Queue("f", N, name="RVel");  rightTime = Queue("L", N, name="RTime")

leftMotorTask  = task_motor(leftMotor,  leftEncoder,  leftMotorGo,  leftData,  leftTime,  Kp_share, Ki_share, SP_share, vel_sign=-1.0)
rightMotorTask = task_motor(rightMotor, rightEncoder, rightMotorGo, rightData, rightTime, Kp_share, Ki_share, SP_share, vel_sign=-1.0)

lf_enable     = Share("B", name="LF Enable");     lf_enable.put(False)
garage_enable = Share("B", name="Garage Enable");  garage_enable.put(False)
cal_cmd       = Share("B", name="Cal Cmd");        cal_cmd.put(0)
course_phase  = Share("B", name="Course Phase");   course_phase.put(0)
bumper_armed  = Share("B", name="Bumper Armed");   bumper_armed.put(False)
btn_state     = Share("B", name="Btn State");      btn_state.put(0)

x_pos_share   = Share("f", name="X");  x_pos_share.put(0.0)
y_pos_share   = Share("f", name="Y");  y_pos_share.put(0.0)
heading_share = Share("f", name="H");  heading_share.put(0.0)

cenData = Queue("f", 1600, name="CenData")
cenTime = Queue("L", 1600, name="CenTime")

# ============================================================
# LINE SENSOR
# ============================================================
sensor_pins = (Pin.cpu.A2, Pin.cpu.A3, Pin.cpu.A4, Pin.cpu.A5, Pin.cpu.B0)
line = LineSensorArray(sensor_pins, oversample=6)

def _apply_cal(cal_data):
    line._min = list(cal_data['min'])
    line._max = list(cal_data['max'])
    ranges = [line._max[i] - line._min[i] for i in range(5)]
    print("Cal ranges:", ranges)

try:
    import ujson
    with open('line_cal.json', 'r') as f:
        _apply_cal(ujson.load(f))
except:
    print("No calibration -- press btn1 to calibrate")

# ============================================================
# BUMPER
# ============================================================
bump0 = Pin(Pin.cpu.B2, mode=Pin.IN, pull=Pin.PULL_UP)
bump1 = Pin(Pin.cpu.B1, mode=Pin.IN, pull=Pin.PULL_UP)
bump2 = Pin(Pin.cpu.C4, mode=Pin.IN, pull=Pin.PULL_UP)

def any_bumper():
    if not bumper_armed.get(): return False
    return (not bump0.value()) or (not bump1.value()) or (not bump2.value())

# ============================================================
# HELPERS
# ============================================================
_MPC_M = (math.pi * 2 * 0.035) / (12 * 120)

def clip(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)

def set_motors(effL, effR):
    leftMotor.set_effort(int(effL))
    rightMotor.set_effort(int(effR))

def _mm_to_counts(mm):
    return int((mm / 1000.0) / _MPC_M)

# ============================================================
# LINE FOLLOW CONSTANTS
# ============================================================
LF_PERIOD_MS   = 20
OUTER_KP       = 400.0
OUTER_KI       = 0.0
OUTER_KD       = 0.0
STEER_DV_MAX   = 300.0
DV_SLEW_PER_S  = 600.0
ALPHA          = 0.12
STRENGTH_MIN   = 0.60
STEER_DEADBAND = 0.02
VEL_PER_EFFORT = 10.0
EFFORT_MAX     = 70
LINE_SETPOINT  = 0.0
KICK_EFFORT    = 0
KICK_TIME_MS   = 0

BASE_VEL_FAST  = 380.0
BASE_VEL_SLOW  = 160.0
LOST_BASE_VEL  = 200.0
CURVE_ERR_MAX  = 0.10
CURVE_VEL_MIN  = 80.0
FAST_DIST_MM   = 1200
START_DIST_MM  = 200
START_VEL      = 200.0

GARAGE_HANDOFF_MM = 1480

# Post-garage curvy section -- slow and precise
CURVY_VEL = 120.0   # slow for squiggly line
CURVY_KP  = 250.0   # enough gain to track curves

# ============================================================
# COURSE CONSTANTS
# ============================================================
GARAGE_DRIVE_EFFORT  = 25
GARAGE_TURN_EFFORT   = 18
GARAGE_DRIVE_BIAS    = 2

# CP0->CP1 straight: 1375mm = 9003 counts
CP0_TO_CP1_COUNTS = 9100

# 200mm quarter circle arc -- average distance traveled
# Theoretical = 2057 counts, tune up/down as needed
ARC_LEFT_COUNTS  = 2057
ARC_RIGHT_COUNTS = 2057  # unused now, kept for reference

# Hardcoded garage entry after arc
GAR_ALIGN_COUNTS = 0   # left motor only spin to align -- tune this
GAR_ENTRY_FWD_MM = 75   # forward into garage before left turn

# Hardcoded exit sequence after bump
EXIT_BACK_MM     = 10
EXIT_FWD_MM      = 350
EXIT_TURN_COUNTS = 726   # 90 deg point turn
GAR_ENTRY_TURN_COUNTS = 605  # 75 deg point turn (726 * 75/90)

CUP_BACKUP_MM = 50
CUP_SIDE_MM   = 150
CUP_EFFORT    = 25

# ============================================================
# TASK: IMU + ODOMETRY
# ============================================================
def task_imu_odo():
    prev_posL = leftEncoder.get_position()
    prev_posR = rightEncoder.get_position()
    heading = 0.0; prev_wrap = 0.0; offset = 0.0
    x = 0.0; y = 0.0; first = True
    while True:
        if _imu_ok:
            try:
                h_deg, _, _ = imu.read_euler()
                wrap = h_deg * math.pi / 180.0
                if first:
                    offset = wrap; prev_wrap = wrap; first = False
                d = wrap - prev_wrap
                while d >  math.pi: d -= 2*math.pi
                while d < -math.pi: d += 2*math.pi
                heading += d; prev_wrap = wrap
                heading_share.put(heading - offset)
            except: pass
        leftEncoder.update(); rightEncoder.update()
        posL = leftEncoder.get_position()
        posR = rightEncoder.get_position()
        dL = (posL - prev_posL) * _MPC_M
        dR = (posR - prev_posR) * _MPC_M
        prev_posL = posL; prev_posR = posR
        h = float(heading_share.get())
        ds = (dL + dR) / 2.0
        x += math.cos(h) * ds
        y += math.sin(h) * ds
        x_pos_share.put(x); y_pos_share.put(y)
        yield 0

# ============================================================
# TASK: LINE FOLLOW
# ============================================================
def task_line_follow():
    outer_i = 0.0; outer_prev = 0.0; pos_f = 0.0
    dv_prev = 0.0; last_dir = 1.0; base_cmd = 0.0
    was_enabled = False; kick_until_ms = 0
    last_t = ticks_us(); dbg_t = ticks_us()
    uL = 0.0; uR = 0.0; dv = 0.0
    line_lost = False
    intersection_seen_t = 0
    past_intersection   = False
    turn90_triggered    = False
    ph1_start_dist      = -1.0

    while True:
        # --- calibration ---
        cmd = int(cal_cmd.get())
        if cmd == 3:
            cal_cmd.put(0)
            line._min = [4095]*line.n; line._max = [0]*line.n
            print("Auto-calibrating -- sweep over line now...")
            line.auto_calibrate(duration_ms=2500, delay_ms=5)
            try:
                import ujson
                data = {'min': line._min, 'max': line._max}
                with open('line_cal.json', 'w') as f: ujson.dump(data, f)
                _apply_cal(data)
                print("Calibration saved.")
            except: print("Save failed")

        # --- disabled ---
        if not lf_enable.get():
            outer_i=0.0; outer_prev=0.0; pos_f=0.0
            dv_prev=0.0; base_cmd=0.0; was_enabled=False
            intersection_seen_t=0; past_intersection=False; turn90_triggered=False
            set_motors(0, 0)
            yield 0; continue

        # --- enable edge ---
        if not was_enabled:
            was_enabled = True
            leftMotor.enable(); rightMotor.enable()
            kick_until_ms = (ticks_us()//1000) + KICK_TIME_MS
            intersection_seen_t = 0
            past_intersection   = False
            turn90_triggered    = False
            ph1_start_dist      = -1.0
            outer_i = 0.0; outer_prev = 0.0; dv_prev = 0.0
            last_dir = 0.0; pos_f = 0.0

        leftMotorGo.put(False); rightMotorGo.put(False)

        # --- bumper check ---
        if any_bumper():
            bumper_armed.put(False)
            phase = int(course_phase.get())
            if phase == 2:
                lf_enable.put(False); set_motors(0,0)
                print("Course: cup hit!")
                course_phase.put(3); garage_enable.put(True)
            else:
                lf_enable.put(False); set_motors(0,0)
                print("BUMPER STOP phase={}".format(phase))
            yield 0; continue

        now = ticks_us()
        dt  = ticks_diff(now, last_t); last_t = now
        dt_s = dt/1e6 if dt > 0 else 0.02

        leftEncoder.update(); rightEncoder.update()
        avg_counts = (abs(leftEncoder.get_position()) + abs(rightEncoder.get_position())) / 2.0
        dist_mm = avg_counts * _MPC_M * 1000.0

        pos, strength = line.centroid()
        ph_thresh = 0.35 if int(course_phase.get()) in (1, 2) else STRENGTH_MIN
        totally_lost = (strength is None) or (strength < ph_thresh)

        # --- CP1 handoff (phase 0 only): encoder ticks say we're at CP1 ---
        if int(course_phase.get()) == 0 and avg_counts >= CP0_TO_CP1_COUNTS:
            print("LF: CP1 reached at {:.0f} counts -> garage sequence".format(avg_counts))
            set_motors(0, 0)
            lf_enable.put(False)
            course_phase.put(10)
            garage_enable.put(True)
            yield 0; continue

        # --- speed selection ---
        ph = int(course_phase.get())
        if ph >= 1:
            BASE_VEL = CURVY_VEL
        elif dist_mm < START_DIST_MM:
            BASE_VEL = START_VEL
        elif dist_mm < FAST_DIST_MM:
            BASE_VEL = BASE_VEL_FAST
        else:
            BASE_VEL = BASE_VEL_SLOW

        # --- Phase 1: follow line, detect intersection, then pivot right to find curvy line ---
        if ph == 1:
            if strength is not None and strength > 3.5:
                if not past_intersection:
                    past_intersection = True
                    print("LF ph1: intersection! disabling lost-line spin")
            if past_intersection and totally_lost:
                # Line ended after intersection -- slow right pivot to find curvy line
                set_motors(8, -8)  # pure pivot right
                yield 0; continue
            if past_intersection and not totally_lost:
                # Found the curvy line -- switch to phase 2 and follow it
                print("LF ph1: curvy line found str={:.2f} -> phase 2".format(
                    strength if strength is not None else 0.0))
                course_phase.put(2)

        # --- line follow control ---
        line_lost = (pos is None) or (strength is None) or (strength < ph_thresh)
        if line_lost:
            if ph >= 2:
                pos = last_dir * 0.5  # curvy: gentle steer toward last known line
            elif ph >= 1:
                pos = 0.0       # phase 1: go straight
            elif dist_mm >= FAST_DIST_MM:
                pos = 0.0
            else:
                pos = last_dir
        else:
            if pos >  0.02: last_dir =  1.0
            elif pos < -0.02: last_dir = -1.0

        pos_f = (1.0-ALPHA)*pos_f + ALPHA*pos
        err = pos_f - LINE_SETPOINT
        outer_i += err * dt_s
        d = (err - outer_prev)/dt_s if dt_s > 0 else 0.0
        outer_prev = err

        kp_use = CURVY_KP if ph >= 2 else OUTER_KP
        dv = kp_use*err + OUTER_KI*outer_i + OUTER_KD*d
        if -STEER_DEADBAND < dv < STEER_DEADBAND: dv = 0.0
        dv = clip(dv, -STEER_DV_MAX, STEER_DV_MAX)

        if strength is not None and strength < 2.0:
            trust = clip((strength-0.15)/(2.0-0.15), 0.0, 1.0)
            dv *= trust; dv_prev *= trust

        dv_step = DV_SLEW_PER_S * dt_s
        if dv > dv_prev+dv_step:       dv = dv_prev+dv_step
        elif dv < dv_prev-dv_step:     dv = dv_prev-dv_step
        dv_prev = dv

        if line_lost:
            target_base = LOST_BASE_VEL; dv = 0.0
        else:
            cf = clip(1.0 - abs(err)/CURVE_ERR_MAX, 0.0, 1.0)
            target_base = CURVE_VEL_MIN + cf*(BASE_VEL - CURVE_VEL_MIN)

        step = 600.0 * dt_s
        if base_cmd < target_base-step:   base_cmd += step
        elif base_cmd > target_base+step: base_cmd -= step
        else:                             base_cmd = target_base

        vL_t = clip(base_cmd-dv, 0.0, 9999)
        vR_t = clip(base_cmd+dv, 0.0, 9999)
        uL = clip(vL_t/VEL_PER_EFFORT, 0.0, float(EFFORT_MAX))
        uR = clip(vR_t/VEL_PER_EFFORT, 0.0, float(EFFORT_MAX))

        set_motors(uL, uR)

        try: cenData.put(pos_f); cenTime.put(now//1000)
        except: pass

        if ticks_diff(now, dbg_t) > 500_000:
            dbg_t = now
            s = strength if strength is not None else 0.0
            xp = float(x_pos_share.get()); yp = float(y_pos_share.get())
            hdg = float(heading_share.get())*180.0/math.pi
            print("[LF ph{}] pos={:+.3f} str={:.2f} dv={:+.1f} base={:.0f} uL={:.0f} uR={:.0f} x={:.2f}m y={:.2f}m hdg={:.1f}".format(
                ph, pos_f, s, dv, base_cmd, uL, uR, xp, yp, hdg))
            flags = []
            if totally_lost: flags.append("LOST")
            if line_lost:    flags.append("LINE_LOST")
            if flags: print("  >>> " + " ".join(flags))

        yield 0

# ============================================================
# TASK: COURSE STATE MACHINE
# ============================================================
_S_IDLE=0; _S_GAR_DRIVE=2
_S_ARC=10; _S_GAR_ALIGN=11; _S_GAR_FWD=12; _S_GAR_TURN_L=13
_S_EXIT_BACK=30; _S_EXIT_TURN_L=31; _S_EXIT_FWD=32; _S_EXIT_TURN_R=33
_S_CUP_BACKUP=7; _S_CUP_TURN_R=8; _S_CUP_FWD1=9; _S_CUP_TURN_L1=40
_S_CUP_FWD2=41; _S_CUP_TURN_L2=42; _S_CUP_FWD3=43; _S_CUP_FWD_1S=44; _S_CUP_TURN_L3=45

def task_course():
    state = _S_IDLE; startL = 0; startR = 0; timer_start = 0

    def snap():
        leftEncoder.update(); rightEncoder.update()
        return leftEncoder.get_position(), rightEncoder.get_position()

    def traveled(sL, sR):
        leftEncoder.update(); rightEncoder.update()
        return (abs(leftEncoder.get_position()-sL) + abs(rightEncoder.get_position()-sR))/2.0

    def fwd(effort, bias=GARAGE_DRIVE_BIAS):
        leftEncoder.update(); rightEncoder.update()
        set_motors(effort-bias, effort+bias)

    while True:
        if not garage_enable.get():
            state = _S_IDLE
            yield state; continue

        now_ms = ticks_us()//1000

        if state == _S_IDLE:
            print("[COURSE] IDLE phase={}".format(int(course_phase.get())))
            lf_enable.put(False); leftMotor.enable(); rightMotor.enable()
            set_motors(0,0); startL, startR = snap()
            phase = int(course_phase.get())
            if phase == 10:
                print("[COURSE] CP1 reached -> 200mm arc")
                state = _S_ARC
            elif phase == 3:
                print("[COURSE] Cup sequence")
                state = _S_CUP_BACKUP
            else:
                print("[COURSE] Garage entry")
                state = _S_ARC

        # ---- 200mm quarter circle arc into garage -- curve RIGHT ----
        # Left (outer) faster, right (inner) slower to curve right
        # Average traveled distance = 2057 counts. Tune ARC_LEFT_COUNTS.
        elif state == _S_ARC:
            if traveled(startL, startR) >= ARC_LEFT_COUNTS:
                set_motors(0,0); startL,startR = snap()
                print("[COURSE] Arc done -> align")
                state = _S_GAR_ALIGN
            else:
                set_motors(10, GARAGE_TURN_EFFORT)  # left faster, right slower = curve right

        # ---- Align: left motor only to get parallel with garage ----
        elif state == _S_GAR_ALIGN:
            if traveled(startL, startR) >= GAR_ALIGN_COUNTS:
                set_motors(0,0); startL,startR = snap()
                print("[COURSE] Align done -> forward {}mm".format(GAR_ENTRY_FWD_MM))
                state = _S_GAR_FWD
            else: set_motors(0, GARAGE_TURN_EFFORT)  # right motor only

        # ---- Forward into garage ----
        elif state == _S_GAR_FWD:
            if traveled(startL, startR) >= _mm_to_counts(GAR_ENTRY_FWD_MM):
                set_motors(0,0); startL,startR = snap()
                print("[COURSE] Fwd done -> turn LEFT 90")
                state = _S_GAR_TURN_L
            else: fwd(GARAGE_DRIVE_EFFORT, bias=0)

        # ---- Left 90 turn to face wall ----
        elif state == _S_GAR_TURN_L:
            if traveled(startL, startR) >= GAR_ENTRY_TURN_COUNTS:
                set_motors(0,0); startL,startR = snap()
                print("[COURSE] Turn L done -> drive to wall")
                state = _S_GAR_DRIVE
            else: set_motors(GARAGE_TURN_EFFORT, -GARAGE_TURN_EFFORT)

        # ---- Drive to wall ----
        elif state == _S_GAR_DRIVE:
            bumper_armed.put(True)
            if any_bumper():
                bumper_armed.put(False); set_motors(0,0); startL,startR = snap()
                print("[COURSE] BUMPER HIT -> back {}mm".format(EXIT_BACK_MM))
                state = _S_EXIT_BACK
            else: fwd(GARAGE_DRIVE_EFFORT)

        # ---- Hardcoded exit sequence ----
        elif state == _S_EXIT_BACK:
            if traveled(startL, startR) >= _mm_to_counts(EXIT_BACK_MM):
                set_motors(0,0); startL,startR = snap()
                print("[COURSE] Back done -> turn LEFT 90")
                state = _S_EXIT_TURN_L
            else: set_motors(-GARAGE_DRIVE_EFFORT, -GARAGE_DRIVE_EFFORT)

        elif state == _S_EXIT_TURN_L:
            if traveled(startL, startR) >= EXIT_TURN_COUNTS:
                set_motors(0,0); startL,startR = snap()
                print("[COURSE] Turn L done -> forward {}mm".format(EXIT_FWD_MM))
                state = _S_EXIT_FWD
            else: set_motors(-GARAGE_TURN_EFFORT, GARAGE_TURN_EFFORT)

        elif state == _S_EXIT_FWD:
            if traveled(startL, startR) >= _mm_to_counts(EXIT_FWD_MM):
                set_motors(0,0); startL,startR = snap()
                print("[COURSE] Fwd done -> turn RIGHT 90")
                state = _S_EXIT_TURN_R
            else: fwd(GARAGE_DRIVE_EFFORT, bias=0)

        elif state == _S_EXIT_TURN_R:
            if traveled(startL, startR) >= EXIT_TURN_COUNTS:
                set_motors(0,0)
                print("[COURSE] Turn R done -> line follow phase 2")
                course_phase.put(2)
                lf_enable.put(True); garage_enable.put(False); state = _S_IDLE
            else: set_motors(GARAGE_TURN_EFFORT, -GARAGE_TURN_EFFORT)

        elif state == _S_CUP_BACKUP:
            if traveled(startL, startR) >= _mm_to_counts(CUP_BACKUP_MM):
                set_motors(0,0); startL,startR = snap()
                print("[COURSE] Cup: backup done"); state = _S_CUP_TURN_R
            else: set_motors(-CUP_EFFORT, -CUP_EFFORT)

        elif state == _S_CUP_TURN_R:
            if traveled(startL, startR) >= EXIT_TURN_COUNTS:
                set_motors(0,0); startL,startR = snap()
                print("[COURSE] Cup: turn R done"); state = _S_CUP_FWD1
            else: set_motors(CUP_EFFORT, -CUP_EFFORT)

        elif state == _S_CUP_FWD1:
            if traveled(startL, startR) >= _mm_to_counts(CUP_SIDE_MM):
                set_motors(0,0); startL,startR = snap()
                print("[COURSE] Cup: fwd1 done"); state = _S_CUP_TURN_L1
            else: fwd(CUP_EFFORT, bias=0)

        elif state == _S_CUP_TURN_L1:
            if traveled(startL, startR) >= EXIT_TURN_COUNTS:
                set_motors(0,0); startL,startR = snap()
                print("[COURSE] Cup: turn L1 done"); state = _S_CUP_FWD2
            else: set_motors(-CUP_EFFORT, CUP_EFFORT)

        elif state == _S_CUP_FWD2:
            if traveled(startL, startR) >= _mm_to_counts(CUP_BACKUP_MM):
                set_motors(0,0); startL,startR = snap()
                print("[COURSE] Cup: fwd2 done"); state = _S_CUP_TURN_L2
            else: fwd(CUP_EFFORT, bias=0)

        elif state == _S_CUP_TURN_L2:
            if traveled(startL, startR) >= EXIT_TURN_COUNTS:
                set_motors(0,0); startL,startR = snap()
                print("[COURSE] Cup: turn L2 done"); state = _S_CUP_FWD3
            else: set_motors(-CUP_EFFORT, CUP_EFFORT)

        elif state == _S_CUP_FWD3:
            bumper_armed.put(True)
            if any_bumper():
                bumper_armed.put(False); set_motors(0,0)
                timer_start = now_ms
                print("[COURSE] Cup: 2nd hit"); state = _S_CUP_FWD_1S
            else: fwd(CUP_EFFORT, bias=0)

        elif state == _S_CUP_FWD_1S:
            if now_ms - timer_start >= 1000:
                set_motors(0,0); startL,startR = snap()
                print("[COURSE] Cup: 1s done"); state = _S_CUP_TURN_L3
            else: fwd(CUP_EFFORT, bias=0)

        elif state == _S_CUP_TURN_L3:
            if traveled(startL, startR) >= EXIT_TURN_COUNTS:
                set_motors(0,0); course_phase.put(4)
                print("[COURSE] Cup done -> phase 4")
                lf_enable.put(True); garage_enable.put(False); state = _S_IDLE
            else: set_motors(-CUP_EFFORT, CUP_EFFORT)

        yield state

# ============================================================
# BLUE BUTTON
# ============================================================
blue_btn = Pin(Pin.cpu.C13, mode=Pin.IN, pull=Pin.PULL_UP)

def task_btn():
    btn_was_pressed = False
    while True:
        pressed = (blue_btn.value() == 0)
        if pressed and not btn_was_pressed:
            s = int(btn_state.get())
            if s == 0:
                print("BTN1: calibrating")
                cal_cmd.put(3); btn_state.put(1)
            elif s == 1:
                print("BTN2: starting course")
                leftMotor.enable(); rightMotor.enable()
                course_phase.put(0); lf_enable.put(True)
                btn_state.put(2)
            else:
                print("BTN3: STOP")
                lf_enable.put(False); garage_enable.put(False)
                bumper_armed.put(False); set_motors(0,0)
                btn_state.put(1)
        btn_was_pressed = pressed
        yield 0

# ============================================================
# SCHEDULER
# ============================================================
userTask = task_user(leftMotorGo, rightMotorGo,
                     leftData, leftTime, rightData, rightTime,
                     Kp_share, Ki_share, SP_share,
                     lf_enable=lf_enable, cal_cmd=cal_cmd,
                     cenData=cenData, cenTime=cenTime)

task_list.append(Task(task_line_follow, name="LineFollow", priority=2, period=LF_PERIOD_MS))
task_list.append(Task(task_imu_odo,     name="IMU_Odo",    priority=2, period=20))
task_list.append(Task(task_course,      name="Course",     priority=1, period=20))
task_list.append(Task(task_btn,         name="Button",     priority=1, period=20))
task_list.append(Task(userTask.run,     name="UserTask",   priority=1, period=50))

collect()
print("Ready. Btn1=calibrate  Btn2=run  Btn3=stop")

while True:
    try:
        task_list.pri_sched()
    except KeyboardInterrupt:
        lf_enable.put(False); garage_enable.put(False)
        set_motors(0,0); leftMotor.disable(); rightMotor.disable()
        print("Stopped.")
        break