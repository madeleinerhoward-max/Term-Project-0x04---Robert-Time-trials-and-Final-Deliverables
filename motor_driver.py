

from pyb import Pin, Timer
import pyb

class motor_driver:

    def __init__(self, PWM_pin, DIR_pin, nSLP_pin, tim, chan):
        
        self.DIR_pin = Pin(DIR_pin, mode=Pin.OUT_PP)

        self.nSLP_pin = Pin(nSLP_pin, mode=Pin.OUT_PP)
        self.nSLP_pin.low() 

        
        self.PWM_chan = tim.channel(chan, pin=PWM_pin, mode=Timer.PWM)
        self.PWM_chan.pulse_width_percent(0)

    def enable(self):
        self.nSLP_pin.high()
        pyb.delay(10)
        self.PWM_chan.pulse_width_percent(0)

    def disable(self):

        self.PWM_chan.pulse_width_percent(0)
        self.nSLP_pin.low()

    def set_effort(self, effort):
        if effort > 100:
            effort = 100
        elif effort < -100:
            effort = -100

        if effort == 0:
            self.PWM_chan.pulse_width_percent(0)
            return

        if effort > 0:
            self.DIR_pin.low()
            self.PWM_chan.pulse_width_percent(effort)
        else:
            self.DIR_pin.high()
            self.PWM_chan.pulse_width_percent(-effort)

