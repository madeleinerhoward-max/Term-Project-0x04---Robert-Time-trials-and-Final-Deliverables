from pyb import USB_VCP
from task_share import Share, Queue
import micropython
import math

S0_INIT   = micropython.const(0)
S1_CMD    = micropython.const(1)
S2_KP     = micropython.const(2)
S3_KI     = micropython.const(3)
S4_SP     = micropython.const(4)
S5_PICK   = micropython.const(5)
S6_WAIT   = micropython.const(6)
S7_PRINT  = micropython.const(7)
S8_LF_HDR   = micropython.const(8)
S9_LF_PRINT = micropython.const(9)
S10_STREAM  = micropython.const(10)

UI_PROMPT = ">: "
digits = set("0123456789")
term   = {"\r", "\n"}

_METERS_PER_COUNT = -(2.0 * math.pi * 0.035) / (12 * 120)

class task_user:
    def __init__(self,
                 leftMotorGo: Share, rightMotorGo: Share,
                 leftData: Queue, leftTime: Queue,
                 rightData: Queue, rightTime: Queue,
                 Kp_share: Share, Ki_share: Share, SP_share: Share,
                 lf_enable: Share = None,
                 cal_cmd: Share = None,
                 cenData: Queue = None,
                 cenTime: Queue = None,
                 heading_share: Share = None,
                 yawrate_share: Share = None,
                 obs_vL: Share = None,
                 obs_vR: Share = None,
                 obs_speed: Share = None,
                 obs_yaw: Share = None,
                 obs_xpos: Share = None,
                 obs_ypos: Share = None,
                 left_enc=None,
                 right_enc=None):

        self._state = S0_INIT

        self._leftMotorGo  = leftMotorGo
        self._rightMotorGo = rightMotorGo
        self._leftData  = leftData
        self._leftTime  = leftTime
        self._rightData = rightData
        self._rightTime = rightTime

        self._Kp = Kp_share
        self._Ki = Ki_share
        self._SP = SP_share
        self._heading = heading_share
        self._yawrate = yawrate_share

        self._lf_enable = lf_enable
        self._cal_cmd = cal_cmd
        self._cenData = cenData
        self._cenTime = cenTime

        # Observer output shares
        self._obs_vL    = obs_vL
        self._obs_vR    = obs_vR
        self._obs_speed = obs_speed
        self._obs_yaw   = obs_yaw
        self._obs_xpos  = obs_xpos
        self._obs_ypos  = obs_ypos

        # Encoders for raw velocity comparison
        self._left_enc  = left_enc
        self._right_enc = right_enc

        self._which = None
        self._ser = USB_VCP()
        self._buf = ""
        self._stream_t = 0

        self._ser.write("User Task object instantiated\r\n")

    def _help(self):
        self._ser.write(
            "+------------------------------------------------------+\r\n"
            "| ME 405 Romi UI (Lab 0x04 + Lab 0x05 + Lab 0x06)     |\r\n"
            "+---+--------------------------------------------------+\r\n"
            "| h | Help menu                                        |\r\n"
            "| k | Enter new Kp, Ki                                 |\r\n"
            "| s | Enter new setpoint                               |\r\n"
            "| g | Trigger step response and print data             |\r\n"
            "| i | Print IMU heading/yaw rate                        |\r\n"
            "| o | Print observer estimated states + position       |\r\n"
            "| p | Stream observer states live (any key to stop)    |\r\n"
            "| r | Run: line follow + stream observer together      |\r\n"
            "+---+--------------------------------------------------+\r\n"
            "| w | Calibrate WHITE (min) for line sensors           |\r\n"
            "| b | Calibrate BLACK (max) for line sensors           |\r\n"
            "| a | Auto-calibrate (move over white/black)           |\r\n"
            "| f | Toggle line following ON/OFF                     |\r\n"
            "| d | Dump centroid log (time_ms, centroid)            |\r\n"
            "+---+--------------------------------------------------+\r\n"
        )

    def _flush_rx(self):
        while self._ser.any():
            self._ser.read(1)

    def _input_start(self, msg):
        self._buf = ""
        self._ser.write(msg + "\r\n" + UI_PROMPT)

    def _input_process(self):
        if not self._ser.any():
            return (False, None)

        c = self._ser.read(1).decode()

        if c in digits:
            self._ser.write(c)
            self._buf += c

        elif c == "." and "." not in self._buf:
            self._ser.write(c)
            self._buf += c

        elif c == "-" and len(self._buf) == 0:
            self._ser.write(c)
            self._buf += c

        elif c == "\x7f" and len(self._buf) > 0:
            self._ser.write(c)
            self._buf = self._buf[:-1]

        elif c in term:
            self._ser.write("\r\n")
            if len(self._buf) == 0:
                return (True, None)
            if self._buf in {"-", "."}:
                return (False, None)
            try:
                return (True, float(self._buf))
            except ValueError:
                return (True, None)

        return (False, None)

    def run(self):
        while True:

            if self._state == S0_INIT:
                self._help()
                self._ser.write(UI_PROMPT)
                self._state = S1_CMD

            elif self._state == S1_CMD:
                if self._ser.any():
                    c = self._ser.read(1).decode()
                    self._ser.write(c + "\r\n")

                    if c in {"h", "H"}:
                        self._help()
                        self._ser.write(UI_PROMPT)

                    elif c in {"k", "K"}:
                        self._input_start("Enter proportional gain, Kp:")
                        self._state = S2_KP

                    elif c in {"s", "S"}:
                        self._input_start("Enter setpoint (speed units/s):")
                        self._state = S4_SP

                    elif c in {"g", "G"}:
                        self._ser.write("Which motor? (l/r)\r\n" + UI_PROMPT)
                        self._state = S5_PICK

                    elif c in {"i", "I"}:
                        if (self._heading is not None) and (self._yawrate is not None):
                            hd = self._heading.get()
                            yr = self._yawrate.get()
                            self._ser.write("Heading(rad) = {}\r\n".format(hd))
                            self._ser.write("YawRate(rad/s) = {}\r\n".format(yr))
                        else:
                            self._ser.write("IMU shares not configured.\r\n")
                        self._ser.write(UI_PROMPT)

                    elif c in {"o", "O"}:
                        if self._obs_vL is not None:
                            self._ser.write("--- Observer State Estimate ---\r\n")
                            self._ser.write("  vL    = {:.4f} m/s\r\n".format(self._obs_vL.get()))
                            self._ser.write("  vR    = {:.4f} m/s\r\n".format(self._obs_vR.get()))
                            self._ser.write("  speed = {:.4f} m/s\r\n".format(self._obs_speed.get()))
                            self._ser.write("  yaw   = {:.4f} rad/s\r\n".format(self._obs_yaw.get()))
                            self._ser.write("  x_pos = {:.4f} m\r\n".format(self._obs_xpos.get()))
                            self._ser.write("  y_pos = {:.4f} m\r\n".format(self._obs_ypos.get()))
                        else:
                            self._ser.write("Observer shares not configured.\r\n")
                        self._ser.write(UI_PROMPT)

                    elif c in {"p", "P"}:
                        if self._obs_vL is not None:
                            self._ser.write("Streaming observer... press any key to stop\r\n")
                            self._stream_t = 0
                            self._state = S10_STREAM
                        else:
                            self._ser.write("Observer shares not configured.\r\n")
                            self._ser.write(UI_PROMPT)

                    elif c in {"r", "R"}:
                        if self._obs_vL is not None and self._lf_enable is not None:
                            self._leftMotorGo.put(False)
                            self._rightMotorGo.put(False)
                            self._lf_enable.put(True)
                            self._ser.write("Line following ON + streaming... any key to stop\r\n")
                            self._stream_t = 0
                            self._state = S10_STREAM
                        else:
                            self._ser.write("Observer or line follow not configured.\r\n")
                            self._ser.write(UI_PROMPT)

                    elif c in {"w", "W"}:
                        if self._cal_cmd is not None:
                            self._cal_cmd.put(1)
                            self._ser.write("Calibrating WHITE...\r\n")
                        else:
                            self._ser.write("Calibration not configured.\r\n")
                        self._ser.write(UI_PROMPT)

                    elif c in {"b", "B"}:
                        if self._cal_cmd is not None:
                            self._cal_cmd.put(2)
                            self._ser.write("Calibrating BLACK...\r\n")
                        else:
                            self._ser.write("Calibration not configured.\r\n")
                        self._ser.write(UI_PROMPT)

                    elif c in {"a", "A"}:
                        if self._cal_cmd is not None:
                            self._cal_cmd.put(3)
                            self._ser.write("Auto-calibrating (move over white/black)...\r\n")
                        else:
                            self._ser.write("Calibration not configured.\r\n")
                        self._ser.write(UI_PROMPT)

                    elif c in {"f", "F"}:
                        if self._lf_enable is not None:
                            self._leftMotorGo.put(False)
                            self._rightMotorGo.put(False)
                            cur = bool(self._lf_enable.get())
                            self._lf_enable.put(not cur)
                            if cur:
                                self._ser.write("Line following OFF\r\n")
                            else:
                                self._ser.write("Line following ON\r\n")
                        else:
                            self._ser.write("Line following not configured.\r\n")
                        self._ser.write(UI_PROMPT)

                    elif c in {"d", "D"}:
                        if (self._cenData is not None) and (self._cenTime is not None):
                            self._ser.write("Time_ms,Centroid\r\n")
                            self._state = S8_LF_HDR
                        else:
                            self._ser.write("Centroid logging not configured.\r\n")
                            self._ser.write(UI_PROMPT)

                    else:
                        self._ser.write("Unknown command. Type 'h'.\r\n" + UI_PROMPT)

            elif self._state == S2_KP:
                done, val = self._input_process()
                if done:
                    if val is not None:
                        self._Kp.put(val)
                        self._ser.write("Kp set to {}\r\n".format(val))
                    else:
                        self._ser.write("Kp not changed\r\n")
                    self._input_start("Enter integral gain, Ki:")
                    self._state = S3_KI

            elif self._state == S3_KI:
                done, val = self._input_process()
                if done:
                    if val is not None:
                        self._Ki.put(val)
                        self._ser.write("Ki set to {}\r\n".format(val))
                    else:
                        self._ser.write("Ki not changed\r\n")
                    self._ser.write(UI_PROMPT)
                    self._state = S1_CMD

            elif self._state == S4_SP:
                done, val = self._input_process()
                if done:
                    if val is not None:
                        self._SP.put(val)
                        self._ser.write("Setpoint set to {}\r\n".format(val))
                    else:
                        self._ser.write("Setpoint not changed\r\n")
                    self._ser.write(UI_PROMPT)
                    self._state = S1_CMD

            elif self._state == S5_PICK:
                if self._ser.any():
                    c = self._ser.read(1).decode()
                    self._ser.write(c + "\r\n")
                    if c in {"l", "L"}:
                        self._which = "L"
                    elif c in {"r", "R"}:
                        self._which = "R"
                    else:
                        self._ser.write("Please type l or r.\r\n" + UI_PROMPT)
                        yield self._state
                        continue

                    self._flush_rx()
                    self._ser.write("Step response triggered.\r\n")

                    if self._which == "L":
                        self._rightMotorGo.put(False)
                        self._leftMotorGo.put(True)
                    else:
                        self._leftMotorGo.put(False)
                        self._rightMotorGo.put(True)

                    self._state = S6_WAIT

            elif self._state == S6_WAIT:
                self._flush_rx()
                if (not self._leftMotorGo.get()) and (not self._rightMotorGo.get()):
                    self._ser.write("Step response complete, printing data.\r\n\r\n")
                    self._ser.write("Time_us,Velocity\r\n")
                    self._state = S7_PRINT

            elif self._state == S7_PRINT:
                if self._which == "L":
                    if self._leftData.any():
                        self._ser.write("{},{}\r\n".format(self._leftTime.get(), self._leftData.get()))
                    else:
                        self._ser.write(UI_PROMPT)
                        self._state = S1_CMD
                else:
                    if self._rightData.any():
                        self._ser.write("{},{}\r\n".format(self._rightTime.get(), self._rightData.get()))
                    else:
                        self._ser.write(UI_PROMPT)
                        self._state = S1_CMD

            elif self._state == S8_LF_HDR:
                self._state = S9_LF_PRINT

            elif self._state == S9_LF_PRINT:
                if self._cenData.any():
                    self._ser.write("{},{}\r\n".format(self._cenTime.get(), self._cenData.get()))
                else:
                    self._ser.write(UI_PROMPT)
                    self._state = S1_CMD

            elif self._state == S10_STREAM:
                if self._ser.any():
                    self._ser.read(1)
                    if self._lf_enable is not None:
                        self._lf_enable.put(False)
                    self._ser.write("\r\nStopped. Line following OFF\r\n")
                    self._ser.write(UI_PROMPT)
                    self._state = S1_CMD
                else:
                    self._stream_t += 1
                    if self._stream_t >= 10:
                        self._stream_t = 0
                        # Get raw encoder velocities for comparison
                        if self._left_enc is not None and self._right_enc is not None:
                            raw_vL = self._left_enc.get_velocity() * _METERS_PER_COUNT
                            raw_vR = self._right_enc.get_velocity() * _METERS_PER_COUNT
                        else:
                            raw_vL = 0.0
                            raw_vR = 0.0
                        self._ser.write(
                            "obs_vL={:.3f} raw_vL={:.3f} | obs_vR={:.3f} raw_vR={:.3f} | spd={:.3f} yaw={:.3f} x={:.3f} y={:.3f}\r\n".format(
                                self._obs_vL.get(),
                                raw_vL,
                                self._obs_vR.get(),
                                raw_vR,
                                self._obs_speed.get(),
                                self._obs_yaw.get(),
                                self._obs_xpos.get(),
                                self._obs_ypos.get()
                            )
                        )

            yield self._state