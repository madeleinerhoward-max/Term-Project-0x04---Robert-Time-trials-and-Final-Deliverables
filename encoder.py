
from pyb import Timer, Pin
from utime import ticks_us, ticks_diff


class encoder:
    """Quadrature encoder using STM32 timer in ENC_AB mode (with input pull-ups)."""

    def __init__(self, tim, chA_pin, chB_pin):
        self.tim = tim
        self.pinA = Pin(chA_pin, mode=Pin.IN, pull=Pin.PULL_UP)
        self.pinB = Pin(chB_pin, mode=Pin.IN, pull=Pin.PULL_UP)
        self.ch1 = self.tim.channel(1, pin=self.pinA, mode=Timer.ENC_AB)
        self.ch2 = self.tim.channel(2, pin=self.pinB, mode=Timer.ENC_AB)
        self.tim.counter(0)

        try:
            self.AR = int(self.tim.period())
        except Exception:
            self.AR = 0xFFFF
        self.half_AR = (self.AR + 1) // 2

        self.position = 0
        self.delta = 0
        self.dt = 1
        self.prev_time = ticks_us()
        self.prev_count = int(self.tim.counter())

    def update(self):
        now = ticks_us()
        self.dt = ticks_diff(now, self.prev_time)
        self.prev_time = now
        if self.dt <= 0:
            self.dt = 1

        count = int(self.tim.counter())
        raw_delta = count - self.prev_count

        if raw_delta > self.half_AR:
            raw_delta -= (self.AR + 1)
        elif raw_delta < -self.half_AR:
            raw_delta += (self.AR + 1)

        self.delta = raw_delta
        self.position += self.delta
        self.prev_count = count

    def get_position(self):
        return self.position

    def get_velocity(self):
        return (self.delta * 1_000_000.0) / self.dt if self.dt > 0 else 0.0

    def zero(self):
        self.tim.counter(0)
        self.position = 0
        self.delta = 0
        self.dt = 1
        self.prev_time = ticks_us()
        self.prev_count = int(self.tim.counter())




