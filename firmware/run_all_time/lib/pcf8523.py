"""
PCF8523 Real-Time Clock Driver for MicroPython (RP2040)
Bit masks from Raspberry Pi Pico C SDK example (pcf8523_i2c.c).
I2C address 0x68
"""
import time

_PCF8523_ADDR  = 0x68
_REG_CONTROL_1 = 0x00
_REG_SECONDS   = 0x03

_MDAYS = [0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31]


def _bcd2dec_hi3(raw):   return (10 * ((raw & 0x70) >> 4)) + (raw & 0x0F)  # seconds, minutes
def _bcd2dec_hi2(raw):   return (10 * ((raw & 0x30) >> 4)) + (raw & 0x0F)  # hours, day
def _bcd2dec_month(raw): return (10 * ((raw & 0x10) >> 4)) + (raw & 0x0F)  # month
def _bcd2dec_year(raw):  return (10 * ((raw & 0xF0) >> 4)) + (raw & 0x0F)  # year 0-99
def _dec2bcd(dec):       return ((dec // 10) << 4) | (dec % 10)
def _is_leap(y):         return y % 4 == 0 and (y % 100 != 0 or y % 400 == 0)


def _to_unix(year, month, day, hour, minute, second):
    if not (1 <= month <= 12 and 1 <= day <= 31 and
            0 <= hour <= 23 and 0 <= minute <= 59 and 0 <= second <= 59):
        return 0
    days = 0
    for y in range(1970, year):
        days += 366 if _is_leap(y) else 365
    mdays = _MDAYS[:]
    if _is_leap(year):
        mdays[2] = 29
    for m in range(1, month):
        days += mdays[m]
    days += day - 1
    return days * 86400 + hour * 3600 + minute * 60 + second


class PCF8523:
    def __init__(self, i2c, address=_PCF8523_ADDR):
        self._i2c  = i2c
        self._addr = address
        self._enable_battery_switchover()


    def initialized(self):
        """Return True if oscillator-stop flag is clear (clock is running)."""
        data = self._i2c.readfrom_mem(self._addr, _REG_SECONDS, 1)
        return not bool(data[0] & 0x80)

    def now(self):
        """Return (year, month, day, weekday, hour, minute, second)."""
        data = self._i2c.readfrom_mem(self._addr, _REG_SECONDS, 7)
        second  = _bcd2dec_hi3(data[0])
        minute  = _bcd2dec_hi3(data[1])
        hour    = _bcd2dec_hi2(data[2])
        day     = _bcd2dec_hi2(data[3])
        weekday = data[4] & 0x07
        month   = _bcd2dec_month(data[5])
        year    = 2000 + _bcd2dec_year(data[6])
        return (year, month, day, weekday, hour, minute, second)

    def unixtime(self):
        """Return Unix timestamp, or 0 if time is invalid."""
        try:
            y, mo, d, _, h, mi, s = self.now()
            return _to_unix(y, mo, d, h, mi, s)
        except Exception:
            return 0

    def datetime_str(self):
        """Return formatted string YYYY/MM/DD HH:MM:SS, or 'N/A' if invalid."""
        try:
            y, mo, d, _, h, mi, s = self.now()
            if _to_unix(y, mo, d, h, mi, s) == 0:
                return "N/A"
            return "{}/{:02d}/{:02d} {:02d}:{:02d}:{:02d}".format(y, mo, d, h, mi, s)
        except Exception:
            return "N/A"

    def set_time(self, year, month, day, weekday, hour, minute, second):
        """Set the RTC. weekday: 0=Sunday … 6=Saturday."""
        # Read CONTROL_1 once, set STOP bit, write it back
        ctrl1 = self._i2c.readfrom_mem(self._addr, _REG_CONTROL_1, 1)[0]
        self._i2c.writeto_mem(self._addr, _REG_CONTROL_1, bytes([ctrl1 | 0x20]))

        buf = bytearray(7)
        buf[0] = _dec2bcd(second) & 0x7F  # clear OS flag
        buf[1] = _dec2bcd(minute)
        buf[2] = _dec2bcd(hour)
        buf[3] = _dec2bcd(day)
        buf[4] = weekday & 0x07
        buf[5] = _dec2bcd(month)
        buf[6] = _dec2bcd(year - 2000)
        self._i2c.writeto_mem(self._addr, _REG_SECONDS, buf)

        # Clear STOP bit using the same ctrl1 value we read earlier
        # avoids a second read which could return stale/wrong data
        self._i2c.writeto_mem(self._addr, _REG_CONTROL_1, bytes([ctrl1 & 0xDF]))
        time.sleep_ms(100)  # give oscillator extra time to restart

    def _enable_battery_switchover(self):
        """Set CTRL3 to 0x00 — standard battery switchover, battery low detection enabled."""
        self._i2c.writeto_mem(self._addr, 0x02, bytes([0x00]))
