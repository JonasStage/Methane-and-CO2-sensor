# Changelog

## Version 4 (current)
- Merged Arduino + datalogging shield into single PCB — fewer parts, lower cost
- Switched methane sensor from NGM2611-E13 → TGS2611-E00 with SR-6 socket for easy replacement
- Added three status LEDs: Power, SD write activity, and error/status indicator
- Requires [MiniCore](https://github.com/MCUdude/MiniCore) board package and USBASP ISP programmer
- Power-optimised firmware (sleep modes, disabled SPI/USART during sleep)

## Version 3
- Added SHT temperature and humidity sensor (high-bit resolution)
- Separate Arduino + datalogging shield; separate PCB for sensors
- See [`archive/v3/`](archive/v3/)

## Version 2
- Higher-bit ADC resolution
- See [`archive/v2/`](archive/v2/)

## Version 1
- Initial Arduino shield design
- See [`archive/v1/`](archive/v1/)

## Swimmer variant
- Experimental design with sensor mounted on a swimmer float
- See [`archive/swimmer/`](archive/swimmer/)
