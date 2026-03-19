

from pyb import ADC, Pin
from utime import sleep_ms

class LineSensorArray:

    def __init__(self, pins, oversample=6, weights=None):
        self.n = len(pins)
        if self.n < 2:
            raise ValueError("Need at least 2 sensors")

        self._oversample = max(1, int(oversample))
        self._adcs = []
        for p in pins:
            if isinstance(p, Pin):
                pinobj = p
            else:
                pinobj = Pin(p)
            self._adcs.append(ADC(pinobj))

        self._min = [4095] * self.n
        self._max = [0] * self.n

        if weights is None:
            mid = (self.n - 1) / 2.0
            self._weights = [float(i - mid) for i in range(self.n)]
        else:
            if len(weights) != self.n:
                raise ValueError("weights length must match number of sensors")
            self._weights = [float(w) for w in weights]

    def read_raw(self):
        vals = [0] * self.n
        for _ in range(self._oversample):
            for i, a in enumerate(self._adcs):
                vals[i] += a.read()
        for i in range(self.n):
            vals[i] //= self._oversample
        return vals

    def calibrate_min(self, samples=200, delay_ms=2):
        if samples < 1:
            samples = 1
        for _ in range(samples):
            r = self.read_raw()
            for i in range(self.n):
                if r[i] < self._min[i]:
                    self._min[i] = r[i]
            sleep_ms(delay_ms)

    def calibrate_max(self, samples=200, delay_ms=2):
        if samples < 1:
            samples = 1
        for _ in range(samples):
            r = self.read_raw()
            for i in range(self.n):
                if r[i] > self._max[i]:
                    self._max[i] = r[i]
            sleep_ms(delay_ms)

    def auto_calibrate(self, duration_ms=2000, delay_ms=5):
        import utime
        t0 = utime.ticks_ms()
        while utime.ticks_diff(utime.ticks_ms(), t0) < duration_ms:
            r = self.read_raw()
            for i in range(self.n):
                if r[i] < self._min[i]:
                    self._min[i] = r[i]
                if r[i] > self._max[i]:
                    self._max[i] = r[i]
            sleep_ms(delay_ms)

    def normalized(self):
        raw = self.read_raw()
        out = [0.0] * self.n
        for i in range(self.n):
            lo = self._min[i]
            hi = self._max[i]
            if hi <= lo:
                out[i] = 0.0
            else:
                v = (raw[i] - lo) / float(hi - lo)
                if v < 0.0:
                    v = 0.0
                if v > 1.0:
                    v = 1.0
                out[i] = v
        return out

    def centroid(self):
        vals = self.normalized()
        s = 0.0
        wsum = 0.0
        for i in range(self.n):
            s += vals[i] * self._weights[i]
            wsum += vals[i]
        if wsum <= 1e-4:
            return (None, 0.0)

        maxw = 0.0
        for w in self._weights:
            maxw += abs(w)
        if maxw <= 1e-6:
            maxw = 1.0

        pos = s / maxw
        return (pos, wsum)

    def get_calibration(self):

        return (list(self._min), list(self._max))
