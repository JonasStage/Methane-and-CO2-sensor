"""
Pico Methane Sensor — Main Script
Ported from Arduino (AVR) to MicroPython for Raspberry Pi Pico.

Reads CH4 (via ADS1115), CO2/RH/Temp (via K33), SHT4x temp/humidity,
battery voltage, and logs data to CSV on SD card. Includes pump cycling
with low-power lightsleep between measurement periods.

On boot, syncs the PCF8523 RTC from the host computer's time (via
machine.RTC, which Thonny sets automatically on connect).
Headless boots use the battery-backed PCF8523 time directly.

Authors:
  - David Bastviken & Nguyen Thanh Duc, Linköping University, Sweden
  - Jonas Stage Sø, University of Southern Denmark (Pico port & automation)
"""

import machine
import time
import os
import sys

import config
from lib.sht4x import SHT4x
from lib.ads1115 import ADS1115
from lib.pcf8523 import PCF8523
from lib.k33 import K33
from lib.sdcard import SDCard

# ── Hardware Initialization ──────────────────────────────────────────────────

i2c = machine.I2C(
    config.I2C_ID,
    sda=machine.Pin(config.I2C_SDA_PIN),
    scl=machine.Pin(config.I2C_SCL_PIN),
    freq=config.I2C_FREQ,
)

machine.Pin(config.SPI_MISO_PIN, machine.Pin.IN, machine.Pin.PULL_UP)
spi = machine.SPI(
    config.SPI_ID,
    baudrate=1_000_000,
    polarity=0,
    phase=0,
    sck=machine.Pin(config.SPI_SCK_PIN),
    mosi=machine.Pin(config.SPI_MOSI_PIN),
    miso=machine.Pin(config.SPI_MISO_PIN),
)
sd_cs = machine.Pin(config.SD_CS_PIN, machine.Pin.OUT, value=1)
sd_cd = machine.Pin(config.SD_CD_PIN, machine.Pin.IN, machine.Pin.PULL_UP)

pump_pin = machine.Pin(config.PUMP_PIN, machine.Pin.OUT, value=0)
led_pin  = machine.Pin(config.LED_PIN,  machine.Pin.OUT, value=0)


# ── Sensor Objects ───────────────────────────────────────────────────────────

def init_sensors():
    """Initialize all sensors except RTC. Returns (sht, ads, k33_sensor, sd_mounted)."""
    errors = []

    try:
        sht = SHT4x(i2c, config.SHT4X_ADDR)
        print("SHT4x initialized.")
    except Exception as e:
        sht = None
        errors.append("SHT4x: {}".format(e))

    try:
        ads = ADS1115(i2c, config.ADS1115_ADDR, gain=1)
        print("ADS1115 initialized.")
    except Exception as e:
        ads = None
        errors.append("ADS1115: {}".format(e))

    try:
        k33_sensor = K33(i2c, config.I2C_SDA_PIN, config.K33_ADDR,
                         scl_pin_num=config.I2C_SCL_PIN, freq=config.I2C_FREQ)
        print("K33 CO2 sensor initialized.")
    except Exception as e:
        k33_sensor = None
        errors.append("K33: {}".format(e))

    class _RP2SDCard(SDCard):
        def init_spi(self, baudrate):
            pass

    sd_mounted = False
    sd = None
    print("Initializing SD card... (CD pin = {})".format(sd_cd.value()))
    time.sleep_ms(250)
    try:
        sd = _RP2SDCard(spi, sd_cs)
        print("SD init OK, sectors={}".format(sd.sectors))
    except Exception as e:
        errors.append("SD init: {}".format(e))
    if sd is not None:
        try:
            buf = bytearray(512)
            sd.readblocks(0, buf)
            print("SD readblocks OK")
        except Exception as e:
            errors.append("SD readblocks: {}".format(e))
        try:
            os.mount(sd, "/sd")
            sd_mounted = True
            print("SD card mounted at /sd")
        except Exception as e:
            errors.append("SD mount: {}".format(e))

    if errors:
        for err in errors:
            print("WARNING: " + err)
        led_pin.value(1)

    return sht, ads, k33_sensor, sd_mounted


# ── RTC Setup ────────────────────────────────────────────────────────────────

def _rtc_time_valid(y, mo, d, h, mi, s):
    return (2024 <= y <= 2099 and 1 <= mo <= 12 and 1 <= d <= 31
            and 0 <= h <= 23 and 0 <= mi <= 59 and 0 <= s <= 59)


def setup_rtc(rtc):
    """Sync PCF8523 from Thonny-synced machine.RTC, or read stored time on headless boot."""
    if rtc is None:
        print("ERROR: RTC not available.")
        return

    # Path 1: Thonny synced machine.RTC — forward it to PCF8523
    internal = machine.RTC()
    y, mo, d, wd, h, mi, s, _ = internal.datetime()
    print("Internal RTC: {}/{}/{} {:02d}:{:02d}:{:02d}".format(y, mo, d, h, mi, s))

    if _rtc_time_valid(y, mo, d, h, mi, s):
        pcf_wd = (wd + 1) % 7  # MicroPython Mon=0 → PCF8523 Sun=0
        rtc.set_time(y, mo, d, pcf_wd, h, mi, s)
        print("RTC synced from host: " + rtc.datetime_str())
        return

    # Path 2: No host time — read PCF8523 directly (headless boot)
    for attempt in range(5):
        time.sleep_ms(200)
        try:
            if not rtc.initialized():
                print("PCF8523 oscillator not running (attempt {})".format(attempt + 1))
                continue
            ry, rmo, rd, _, rh, rmi, rs = rtc.now()
            if _rtc_time_valid(ry, rmo, rd, rh, rmi, rs):
                print("RTC is running: " + rtc.datetime_str())
                return
            print("PCF8523 invalid time: {}/{}/{} {:02d}:{:02d}:{:02d}".format(
                ry, rmo, rd, rh, rmi, rs))
        except Exception as e:
            print("PCF8523 error (attempt {}): {}".format(attempt + 1, e))

    print("No valid RTC time. Timestamps will be N/A.")


# ── Data Logging ─────────────────────────────────────────────────────────────

def ensure_csv(sd_mounted):
    print(config.CSV_HEADER)
    if not sd_mounted:
        return
    filepath = "/sd/" + config.DATA_FILENAME
    try:
        os.stat(filepath)
        print("{} exists.".format(config.DATA_FILENAME))
    except OSError:
        print("Creating {}...".format(config.DATA_FILENAME))
        with open(filepath, "w") as f:
            f.write(config.CSV_HEADER + "\n")


def build_log_line(m, rtc, sht, ads, K33_CO2, K33_RH, K33_Temp,
                    SampleNumber, PumpCycle):
    """Read sensors and return the formatted CSV line. Runs every sample;
    does not touch the SD card."""
    if rtc is not None:
        try:
            unix_ts = rtc.unixtime()
            dt_str  = rtc.datetime_str()
        except Exception:
            unix_ts = 0
            dt_str  = "N/A"
    else:
        unix_ts = 0
        dt_str  = "N/A"

    if sht:
        try:
            tempC, rh = sht.read()
        except Exception:
            tempC, rh = -1.0, -1.0
    else:
        tempC, rh = -1.0, -1.0

    if ads:
        try:
            time.sleep_ms(50)
            adc_diff = ads.read_adc_differential(0, 1)
            CH4mV = ads.compute_volts(adc_diff) * 1000
        except Exception:
            CH4mV = -1.0
    else:
        CH4mV = -1.0

    line = "{},{},{},{},{},{},{},{},{},{},{}".format(
        m, unix_ts, dt_str,
        rh, tempC,
        CH4mV,
        K33_RH, K33_Temp, K33_CO2,
        SampleNumber, PumpCycle,
    )

    print("[#{} | {} | pump cycle {}]".format(SampleNumber, dt_str, PumpCycle))
    print("  SHT4x : {:.1f} C  {:.1f} %RH".format(tempC, rh))
    print("  CH4   : CH4smV={:.1f} mV".format(CH4mV))
    print("  K33   : CO2={:.0f} ppm  {:.1f} C  {:.1f} %RH".format(K33_CO2, K33_Temp, K33_RH))

    return line


def flush_buffer(sd_mounted, buffer):
    """Write all buffered lines to the SD card in a single open/append/close.
    Returns True on success, False on write failure. Does not clear the
    buffer — caller clears it only after a confirmed successful write."""
    if not sd_mounted or not buffer:
        return True

    try:
        with open("/sd/" + config.DATA_FILENAME, "a") as f:
            for line in buffer:
                f.write(line + "\n")
    except Exception as e:
        print("SD write error: {}".format(e))
        return False

    print("Flushed {} samples to SD.".format(len(buffer)))
    return True


# ── Sleep with Pump ──────────────────────────────────────────────────────────

def sleep_with_pump(cycles):
    print("Pump ON, sleeping for (~{} seconds)...".format(config.SLEEP_CYCLE_MS // 1000))
    pump_pin.value(1)
    
    machine.lightsleep(config.SLEEP_CYCLE_MS)

    pump_pin.value(0)
    print("Pump OFF, waking up!")


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    print("\n=== Pico Methane Sensor ===\n")

    time.sleep_ms(500)  # allow I2C bus to settle after power-on

    # RTC must be initialised first, before any other sensor touches the I2C bus
    try:
        rtc = PCF8523(i2c, config.PCF8523_ADDR)
        print("PCF8523 RTC initialized.")
        print("raw after init:", [hex(b) for b in i2c.readfrom_mem(0x68, 0x03, 7)])
    except Exception as e:
        rtc = None
        print("WARNING: PCF8523: {}".format(e))

    setup_rtc(rtc)

    sht, ads, k33_sensor, sd_mounted = init_sensors()

    if not sd_mounted:
        print("FATAL: SD card not available. Halting.")
        led_pin.value(1)
        sys.exit()

    ensure_csv(sd_mounted)

    if rtc:
        try:
            print("Boot time: " + rtc.datetime_str())
        except Exception as e:
            print("RTC read error at boot: {}".format(e))

    print("Pump test...")
    pump_pin.value(1)
    time.sleep(3)
    pump_pin.value(0)

    SampleNumber     = 0
    PumpCycle        = 1
    time_to_read_k33 = True
    k33_delay_count  = 0
    K33_CO2          = -1.0
    K33_RH           = -1.0
    K33_Temp         = -1.0
    start_ticks      = time.ticks_ms()
    previous_millis  = time.ticks_ms()
    write_buffer	 = []

    while True:
        loop_start = time.ticks_ms()
        SampleNumber += 1
        m = time.ticks_diff(loop_start, start_ticks)

        if sd_cd.value() == 1:
            print("FATAL: SD card removed. Halting.")
            led_pin.value(1)
            sys.exit()

        if k33_sensor and (SampleNumber % 9) == 0:
            k33_sensor.init_poll()

        if k33_sensor and time_to_read_k33:
            co2, rh_k, temp_k = k33_sensor.read_all()
            K33_CO2  = co2
            K33_RH   = rh_k
            K33_Temp = temp_k
            time_to_read_k33 = False

        line = build_log_line(m, rtc, sht, ads, K33_CO2, K33_RH, K33_Temp,
                               SampleNumber, PumpCycle)
        write_buffer.append(line)

        if len(write_buffer) >= config.WRITE_BUFFER_SIZE:
            ok = flush_buffer(sd_mounted, write_buffer)
            if not ok:
                print("FATAL: SD write failed. Halting.")
                led_pin.value(1)
                sys.exit()
            write_buffer = []

        if (SampleNumber % 3) == 0:
            led_pin.value(1)
            time.sleep_ms(10)
            led_pin.value(0)

        if not time_to_read_k33:
            k33_delay_count += 1
            if k33_delay_count >= config.K33_READ_INTERVAL:
                time_to_read_k33 = True
                k33_delay_count  = 0

        current_millis = time.ticks_ms()
        if time.ticks_diff(current_millis, previous_millis) >= config.MEASUREMENT_LENGTH_MS:
            previous_millis = current_millis
            
            ok = flush_buffer(sd_mounted, write_buffer)
            if not ok:
                print("FATAL: SD write failed. Halting.")
                led_pin.value(1)
                sys.exit()
            write_buffer = []
                
            sleep_with_pump()
            PumpCycle    += 1
            SampleNumber  = 0

        elapsed   = time.ticks_diff(time.ticks_ms(), loop_start)
        remaining = config.LOG_INTERVAL_MS - elapsed
        if remaining > 0:
            time.sleep_ms(remaining)


# Run
main()