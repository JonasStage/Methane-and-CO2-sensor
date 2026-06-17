"""
K33 CO2/RH/Temperature Sensor Driver for MicroPython
I2C interface with GPIO-based wake pulse (replaces AVR register manipulation)
Default address: 0x7F (modified from 0x68 to avoid RTC collision)
"""
import time


class K33:
    def __init__(self, i2c, sda_pin_num, address=0x7F, scl_pin_num=None, freq=100000):
        """
        i2c:         machine.I2C instance
        sda_pin_num: GPIO number of the SDA pin (kept for API compatibility)
        address:     I2C address of the K33 sensor
        scl_pin_num: unused, kept for API compatibility
        freq:        unused, kept for API compatibility
        """
        self._i2c = i2c
        self._addr = address

    def init_poll(self):
        """Tell the sensor to take a measurement."""
        cmd = bytes([0x11, 0x00, 0x60, 0x35, 0xA6])
        try:
            self._i2c.writeto(self._addr, cmd)
        except OSError:
            return False
        time.sleep_ms(20)
        try:
            data = self._i2c.readfrom(self._addr, 2)
        except OSError:
            return False
        return True

    def _read_register(self, reg_hi, reg_lo, checksum_byte):
        """Read 2 bytes from sensor RAM using Command 0x22."""
        cmd = bytes([0x22, 0x00, reg_hi, checksum_byte])
        try:
            self._i2c.writeto(self._addr, cmd)
        except OSError:
            return None
        time.sleep_ms(50)
        try:
            data = self._i2c.readfrom(self._addr, 4)
        except OSError:
            return None

        # Verify checksum
        calc_sum = (data[0] + data[1] + data[2]) & 0xFF
        if calc_sum != data[3]:
            return None

        value = ((data[1] & 0xFF) << 8) | (data[2] & 0xFF)
        return value

    def read_co2(self):
        """Read CO2 value in ppm. Returns float or -1 on error."""
        val = self._read_register(0x08, 0x09, 0x2A)
        if val is None:
            return -1.0
        return float(val)

    def read_rh(self):
        """Read relative humidity in %. Returns float or -1 on error."""
        val = self._read_register(0x14, 0x15, 0x36)
        if val is None:
            return -1.0
        return val / 100.0
    
    def wake(self):
        """Wake the K33 sensor by issuing an I2C start condition.
        A bare writeto generates START + address on the bus; the NACK is expected
        and ignored. This avoids GPIO pin manipulation that would break the shared
        hardware I2C peripheral."""
        try:
            self._i2c.writeto(self._addr, b'')
        except OSError:
            pass  # NACK is expected
        time.sleep_ms(2)

    def read_temp(self):
        """Read temperature in °C. Returns float or -1 on error."""
        val = self._read_register(0x12, 0x13, 0x34)
        if val is None:
            return -1.0
        return val / 100.0

    def read_all(self):
        """Wake, poll, and read all values. Returns (co2, rh, temp)."""
        self.wake()
        time.sleep_ms(50)
        self.init_poll()
        time.sleep_ms(50)
        co2 = self.read_co2()
        time.sleep_ms(20)
        rh = self.read_rh()
        time.sleep_ms(20)
        temp = self.read_temp()
        return co2, rh, temp

