
from pyb import Pin
import time

_CHIP_ID = 0x00
_OPR_MODE = 0x3D
_EULER_H_LSB = 0x1A
_GYRO_DATA_LSB = 0x14
_CALIB_STAT = 0x35
_CALIB_START = 0x55
_CALIB_LENGTH = 22


OPR_MODE_CONFIG = 0x00
OPR_MODE_IMU = 0x08
OPR_MODE_NDOF = 0x0C

class BNO055:
    def __init__(self, i2c, address=0x28, reset_pin=None, do_reset=True):
        self.i2c = i2c
        self.addr = address

        self.reset_pin = None
        if reset_pin:
            self.reset_pin = reset_pin if isinstance(reset_pin, Pin) else Pin(reset_pin, Pin.OUT_PP)
            self.reset_pin.high()
            time.sleep_ms(10)

            if do_reset:
                self.reset()
        else:
          
            time.sleep_ms(700)
            
    def _mem_read(self, nbytes, reg):
        return self.i2c.mem_read(nbytes, self.addr, reg)

    def _mem_write(self, value, reg):
        return self.i2c.mem_write(value, self.addr, reg)

    def reset(self):
        """Pulse hardware reset (RST is active-low on most BNO055 breakouts)."""
        if not self.reset_pin:
            return
        self.reset_pin.low()
        time.sleep_ms(50)
        self.reset_pin.high()
      
        time.sleep_ms(700)

   
    def read_chip_id(self):
        return self._mem_read(1, _CHIP_ID)[0]

    
    def set_mode(self, mode):
        try:
            self._mem_write(mode, _OPR_MODE)
        except OSError:
            time.sleep_ms(50)
            self._mem_write(mode, _OPR_MODE)
        time.sleep_ms(30)

    def get_mode(self):
        return self._mem_read(1, _OPR_MODE)[0]

    def read_cal_status(self):
        return self._mem_read(1, _CALIB_STAT)[0]

    def read_calibration(self):
        return self._mem_read(_CALIB_LENGTH, _CALIB_START)

    def write_calibration(self, data_bytes):
        if not (isinstance(data_bytes, (bytes, bytearray)) and len(data_bytes) == _CALIB_LENGTH):
            raise ValueError("calibration data must be bytes length %d" % _CALIB_LENGTH)

        cur_mode = self.get_mode()
        self._mem_write(OPR_MODE_CONFIG, _OPR_MODE)
        time.sleep_ms(20)

        for i in range(_CALIB_LENGTH):
            self._mem_write(data_bytes[i], _CALIB_START + i)
            time.sleep_ms(2)

        self._mem_write(cur_mode, _OPR_MODE)
        time.sleep_ms(50)

    def _s16(self, lo, hi):
        v = lo | (hi << 8)
        return v - 65536 if v & 0x8000 else v

    def read_euler(self):
        raw = self._mem_read(6, _EULER_H_LSB)
        h = self._s16(raw[0], raw[1]) / 16.0
        r = self._s16(raw[2], raw[3]) / 16.0
        p = self._s16(raw[4], raw[5]) / 16.0
        h = (h + 360.0) % 360.0
        return (h, r, p)

    def read_gyro(self):
        raw = self._mem_read(6, _GYRO_DATA_LSB)
        gx = self._s16(raw[0], raw[1]) / 16.0
        gy = self._s16(raw[2], raw[3]) / 16.0
        gz = self._s16(raw[4], raw[5]) / 16.0
        return (gx, gy, gz)

    def read_heading(self):
        return self.read_euler()[0]

    def read_yaw_rate(self):

        return self.read_gyro()[2]
