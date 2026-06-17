"""
SHT4x Temperature & Humidity Sensor Driver for MicroPython
I2C interface, default address 0x44
"""
import time
import struct

_SHT4X_DEFAULT_ADDR = 0x44
_SHT4X_READSERIAL = 0x89
_SHT4X_SOFTRESET = 0x94
_SHT4X_MEASURE_HIGH = 0xFD
_SHT4X_MEASURE_MED = 0xF6
_SHT4X_MEASURE_LOW = 0xE0


class SHT4x:
    def __init__(self, i2c, address=_SHT4X_DEFAULT_ADDR):
        self._i2c = i2c
        self._addr = address
        self._buf = bytearray(6)
        # Verify sensor is present
        self.serial_number

    @property
    def serial_number(self):
        self._i2c.writeto(self._addr, bytes([_SHT4X_READSERIAL]))
        time.sleep_ms(10)
        data = self._i2c.readfrom(self._addr, 6)
        ser = (data[0] << 24) | (data[1] << 16) | (data[3] << 8) | data[4]
        return ser

    def reset(self):
        self._i2c.writeto(self._addr, bytes([_SHT4X_SOFTRESET]))
        time.sleep_ms(1)

    def read(self):
        """Returns (temperature_C, relative_humidity_percent)."""
        self._i2c.writeto(self._addr, bytes([_SHT4X_MEASURE_HIGH]))
        time.sleep_ms(10)
        data = self._i2c.readfrom(self._addr, 6)

        temp_raw = (data[0] << 8) | data[1]
        # CRC at data[2]
        hum_raw = (data[3] << 8) | data[4]
        # CRC at data[5]

        temperature = -45.0 + 175.0 * temp_raw / 65535.0
        humidity = -6.0 + 125.0 * hum_raw / 65535.0
        humidity = max(0.0, min(100.0, humidity))

        return temperature, humidity
