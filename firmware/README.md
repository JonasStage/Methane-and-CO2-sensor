# Firmware

Micropython code for the CH₄ and CO₂ sensor (Version 3).

| Folder  | Use |
|--------|-----|
| `normal_run/`  | Standard deployment — measures for 40 min, vents chamber for 20 min, repeats |
| `run_all_time/` | Continuous logging without activating the pump |

## Board setup (MiniCore)

1. Install Thonny 
2. Connect the sensor via a USB-C cable.
3. Install MicroPython onto the sensor either through Thonny or by drag-and-dropping the .uf2 file found [here](https://micropython.org/download/rp2-pico/rp2-pico-latest.uf2) onto the sensor drive.
4. Transfer all the files from one of the code folders to the sensor through connecting in Thonny. This will also set the sensor clock according to the time of your computer.
