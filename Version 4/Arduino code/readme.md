In this folder, you will find three folders containing the different Arduino codes. 
The Calibration folder can be used to run the sensor continously without breaks, and without recording CO2 values to save power. The file in this folder is used to if calibrations are done using the sensor. 
The No_pump_code is used if you wish to have the sensor running without turning on the pump, which would otherwise stop the sensor from logging during that period. In this mode the sensor records continously.
The Normal_run_code is used if you want the sensor to turn on the pump at certain intervals. In this example the sensor measures for 40 minutes, before it turns the pump on for 20 minutes and then restarts the cycle. 
