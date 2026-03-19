

import micropython
import math
from utime import ticks_us, ticks_diff, ticks_ms

S0_INIT = micropython.const(0)
S1_RUN  = micropython.const(1)

class task_imu:
    def __init__(self, imu, heading_share, yawrate_share, cal_share):
        self._state = S0_INIT
        self.imu = imu
        self.heading_share = heading_share
        self.yawrate_share = yawrate_share
        self.cal_share = cal_share

        self._theta = 0.0
        self._prev_theta_wrap = None
        self._heading_offset = 0.0  

    @staticmethod
    def _wrap_to_pi(x):
        while x <= -math.pi:
            x += 2.0 * math.pi
        while x > math.pi:
            x -= 2.0 * math.pi
        return x

    def run(self):
        while True:
            if self._state == S0_INIT:
                h_deg, _, _ = self.imu.read_euler()
                _, _, gz_dps = self.imu.read_gyro()

                theta_wrap = (h_deg * math.pi) / 180.0

               
                self._heading_offset = theta_wrap
                self._theta = 0.0
                self._prev_theta_wrap = theta_wrap

                self.heading_share.put(0.0)
                self.yawrate_share.put((gz_dps * math.pi) / 180.0)
                self.cal_share.put(self.imu.read_cal_status())

                self._state = S1_RUN

            elif self._state == S1_RUN:
                h_deg, _, _ = self.imu.read_euler()
                _, _, gz_dps = self.imu.read_gyro()
                cal = self.imu.read_cal_status()

                theta_wrap = (h_deg * math.pi) / 180.0

                # Unwrap continuous heading
                d = theta_wrap - self._prev_theta_wrap
                d = self._wrap_to_pi(d)
                self._theta += d
                self._prev_theta_wrap = theta_wrap

                # Apply offset so heading starts at zero
                heading_zeroed = self._theta - self._heading_offset

                self.heading_share.put(heading_zeroed)
                self.yawrate_share.put((gz_dps * math.pi) / 180.0)
                self.cal_share.put(cal)


            yield self._state
