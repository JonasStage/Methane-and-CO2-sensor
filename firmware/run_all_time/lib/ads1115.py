"""
ADS1115 16-bit ADC Driver for MicroPython
I2C interface, configurable address (default 0x49)
"""
import time
import struct

# Register addresses
_ADS1115_REG_CONVERSION = 0x00
_ADS1115_REG_CONFIG = 0x01

# Config register bits
_ADS1115_OS_SINGLE = 0x8000
_ADS1115_MUX_SINGLE = {0: 0x4000, 1: 0x5000, 2: 0x6000, 3: 0x7000}

# Differential MUX settings: {(positive, negative): mux_bits}
_ADS1115_MUX_DIFF = {
    (0, 1): 0x0000,  # AIN0 - AIN1 (default differential pair)
    (0, 3): 0x1000,  # AIN0 - AIN3
    (1, 3): 0x2000,  # AIN1 - AIN3
    (2, 3): 0x3000,  # AIN2 - AIN3
}

# Gain settings: FSR in volts
_ADS1115_GAINS = {
    2/3: 0x0000,  # +/- 6.144V
    1:   0x0200,  # +/- 4.096V
    2:   0x0400,  # +/- 2.048V (default)
    4:   0x0600,  # +/- 1.024V
    8:   0x0800,  # +/- 0.512V
    16:  0x0A00,  # +/- 0.256V
}

_ADS1115_FSR = {
    2/3: 6.144,
    1:   4.096,
    2:   2.048,
    4:   1.024,
    8:   0.512,
    16:  0.256,
}

_ADS1115_DR_128SPS = 0x0080
_ADS1115_CMODE_TRAD = 0x0000
_ADS1115_CPOL_ACTVLOW = 0x0000
_ADS1115_CLAT_NONLAT = 0x0000
_ADS1115_CQUE_NONE = 0x0003


class ADS1115:
    def __init__(self, i2c, address=0x49, gain=1):
        self._i2c = i2c
        self._addr = address
        self.gain = gain

    def _write_register(self, reg, value):
        data = struct.pack('>BH', reg, value)
        self._i2c.writeto(self._addr, data)

    def _read_register(self, reg):
        self._i2c.writeto(self._addr, bytes([reg]))
        data = self._i2c.readfrom(self._addr, 2)
        return struct.unpack('>h', data)[0]

    def read_adc(self, channel):
        """Read single-ended ADC value from channel (0-3). Returns raw 16-bit signed value."""
        config = (
            _ADS1115_OS_SINGLE |
            _ADS1115_MUX_SINGLE[channel] |
            _ADS1115_GAINS[self.gain] |
            _ADS1115_DR_128SPS |
            _ADS1115_CMODE_TRAD |
            _ADS1115_CPOL_ACTVLOW |
            _ADS1115_CLAT_NONLAT |
            _ADS1115_CQUE_NONE |
            0x0100  # Single-shot mode
        )
        self._write_register(_ADS1115_REG_CONFIG, config)
        time.sleep_ms(10)
        return self._read_register(_ADS1115_REG_CONVERSION)

    def read_adc_differential(self, positive=0, negative=1):
        """Read differential ADC value between two input pins.
        Valid (positive, negative) pairs: (0,1), (0,3), (1,3), (2,3).
        Returns raw 16-bit signed value (can be negative)."""
        try:
            mux = _ADS1115_MUX_DIFF[(positive, negative)]
        except KeyError:
            raise ValueError(
                "Invalid differential pair ({}, {}). Valid pairs: {}".format(
                    positive, negative, list(_ADS1115_MUX_DIFF.keys())
                )
            )
        config = (
            _ADS1115_OS_SINGLE |
            mux |
            _ADS1115_GAINS[self.gain] |
            _ADS1115_DR_128SPS |
            _ADS1115_CMODE_TRAD |
            _ADS1115_CPOL_ACTVLOW |
            _ADS1115_CLAT_NONLAT |
            _ADS1115_CQUE_NONE |
            0x0100  # Single-shot mode
        )
        self._write_register(_ADS1115_REG_CONFIG, config)
        time.sleep_ms(10)
        return self._read_register(_ADS1115_REG_CONVERSION)

    def compute_volts(self, raw):
        """Convert raw ADC value to voltage."""
        return raw * _ADS1115_FSR[self.gain] / 32767.0
