# Changelog

## Version 3 (Current version)
- Switched to RP2040 microcontroller.
- Added USB-C connection.
- The ADC now utilizes differential input, rather than single-ended.
- Fewer soldering points.
- Added QR-code to this GitHub.
- Minor improvements to layout and design.

## Version 2
- Merged Arduino + datalogging shield into single PCB — fewer parts, lower cost
- Switched methane sensor from NGM2611-E13 → TGS2611-E00 with SR-6 socket for easy replacement
- Added three status LEDs: Power, SD write activity, and error/status indicator
- Requires [MiniCore](https://github.com/MCUdude/MiniCore) board package and USBASP ISP programmer
- Power-optimised firmware (sleep modes, disabled SPI/USART during sleep)

## Version 1
- Added SHT temperature and humidity sensor (high-bit resolution)
- Separate Arduino + datalogging shield; separate PCB for sensors
- Higher-bit ADC resolution
- Initial Arduino shield design
