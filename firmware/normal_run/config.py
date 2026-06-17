"""
Configuration for Pico Methane Sensor
Pin assignments and timing constants for Raspberry Pi Pico.
"""

# === I2C Bus (shared by SHT4x, ADS1115, PCF8523, K33) ===
I2C_ID = 0
I2C_SDA_PIN = 0   # GP0
I2C_SCL_PIN = 1   # GP1
I2C_FREQ = 100_000

# === SPI Bus (SD Card) ===
SPI_ID = 0
SPI_SCK_PIN = 2   # GP2
SPI_MOSI_PIN = 3  # GP3
SPI_MISO_PIN = 4  # GP4
SD_CS_PIN = 5     # GP5
SD_CD_PIN = 12    # GP12 - SD card detect (input)

# === GPIO ===
PUMP_PIN = 25      # GP25 - Pump relay/MOSFET control
LED_PIN = 11       # GP11 - Status LED

# === I2C Addresses ===
SHT4X_ADDR = 0x44
ADS1115_ADDR = 0x49
PCF8523_ADDR = 0x68
K33_ADDR = 0x7F

# === Timing (milliseconds) ===
LOG_INTERVAL_MS = (2*1000)           # Time between data logging entries. Set to every 2 seconds.
MEASUREMENT_LENGTH_MS = (40*60*1000) # Length of measurements before going to sleep (40 min). Change 40 to the number of minutes you want the sensor to measure for
SLEEP_CYCLE_MS = (20*60*1000)  		 # Duration the sensor sleeps with pump turned on (20 min). Change 20 to the number of minutes you want the sensor to sleep for

# === Data Buffering ===
WRITE_BUFFER_SIZE = 10  # Number of samples to accumulate in RAM before writing to SD

# === K33 CO2 Sensor ===
K33_READ_INTERVAL = 10  # Read K33 every N logging cycles

# === ADC ===
ADC_VREF_MV = 3300.0   # Pico ADC reference voltage (3.3V)
ADC_STEPS = 65535.0     # Pico 16-bit ADC (read_u16 returns 0-65535)

# === Data File ===
DATA_FILENAME = "datalog.csv"
CSV_HEADER = "millis,stampunix,datetime,RH,tempC,CH4mV,K33_RH,K33_Temp,K33_CO2,SampleNumber,PumpCycle"
