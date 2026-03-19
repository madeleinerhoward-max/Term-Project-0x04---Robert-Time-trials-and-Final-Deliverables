from utime import ticks_us, ticks_diff
import micropython

S0_INIT = micropython.const(0)
S1_WAIT = micropython.const(1)
S2_RUN  = micropython.const(2)

class task_motor:
    def __init__(self, mot, enc, goFlag, dataValues, timeValues,
                 Kp_share, Ki_share, SP_share, vel_sign=1.0, effort_share=None):
        self._state = S0_INIT

        self._mot = mot
        self._enc = enc

        self._goFlag = goFlag
        self._dataValues = dataValues
        self._timeValues = timeValues

        self._Kp = Kp_share
        self._Ki = Ki_share
        self._SP = SP_share
        self._vel_sign = float(vel_sign)
        self._effort_share = effort_share  # Share to publish current effort (%)

        self._startTime = 0
        self._lastTime  = 0
        self._i_term = 0.0

        print("Motor Task object instantiated")

    def run(self):
        while True:

            if self._state == S0_INIT:
                self._mot.set_effort(0)
                self._state = S1_WAIT

            elif self._state == S1_WAIT:
                if self._goFlag.get():
                    self._dataValues.clear()
                    self._timeValues.clear()

                    self._enc.zero()
                    self._mot.enable()
                    self._mot.set_effort(0)

                    self._startTime = ticks_us()
                    self._lastTime  = self._startTime
                    self._i_term = 0.0

                    self._state = S2_RUN

            elif self._state == S2_RUN:
                now = ticks_us()
                dt_us = ticks_diff(now, self._lastTime)
                if dt_us <= 0:
                    dt_us = 1
                self._lastTime = now
                dt = dt_us * 1e-6

                self._enc.update()
                vel = self._vel_sign * float(self._enc.get_velocity())

                sp = float(self._SP.get())
                kp = float(self._Kp.get())
                ki = float(self._Ki.get())

                err = sp - vel
                self._i_term += err * dt

                effort = kp * err + ki * self._i_term

                if effort > 100.0:
                    effort = 100.0
                elif effort < -100.0:
                    effort = -100.0

                self._mot.set_effort(effort)

                # Publish effort so observer knows what was commanded
                if self._effort_share is not None:
                    self._effort_share.put(float(effort))

                self._dataValues.put(vel)
                self._timeValues.put(ticks_diff(now, self._startTime))

                if self._dataValues.full():
                    self._mot.set_effort(0)
                    self._goFlag.put(False)
                    self._state = S1_WAIT

            yield self._state