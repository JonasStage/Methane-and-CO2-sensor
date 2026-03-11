# CH<sub>4</sub> and CO<sub>2</sub> DIY sensor with automated aeration

This repository is dedicated to CH<sub>4</sub> and CO<sub>2</sub> sensors created by Jonas Stage Sø [(Sø et al., 2024)](https://doi.org/10.1029/2024JG008035) and [(Sø et al., 2023)](https://doi.org/10.1016/j.scitotenv.2023.162895). 
Sensors are made following [Bastviken et al. (2020)](https://doi.org/10.5194/bg-17-3659-2020), modified for automated fluxes, higher resolution and lower power consumption by Jonas Stage Sø, University of Southern Denmark, Denmark. 

## Table of contents

* [Latest version](https://github.com/JonasStage/Methane-and-CO2-sensor#latest-version)
* [Step-by-step guide](https://github.com/JonasStage/Methane-and-CO2-sensor#a-step-by-step-guide-to-building-methane-and-co2-sensors-and-an-automated-floating-chamber)
  * [Sensor](https://github.com/JonasStage/Methane-and-CO2-sensor#building-the-sensor)
  * [Chamber](https://github.com/JonasStage/Methane-and-CO2-sensor#building-the-chamber)
* [Calibrating the CO<sub>2</sub> sensor](https://github.com/JonasStage/Methane-and-CO2-sensor#calibrating-the-co2-sensor)

## Latest version
I finally found the time to create the new version 4 of the sensor, which brings quite some upgrades!<br>
* The new version implements the arduino and the datalogging shield from the version 3 into the same PCB as all the sensors, which means that there are less items you need to buy and it is cheaper. 
* The new version implements the Figaro TGS2611-E00 methane sensor rather than the NGM2611-E13 which was used on the previous version. The TGS2611-E00 is actually the same sensor as used on the NGM2611-E13, but by utilizing the TGS2611-E00 the costs are lowered, and it allows you to use the socket (SR-6), such that the sensor can easily be replaced. Just make sure the orientation of the sensor is correct. The notch on the sensor should align with the footprint on the PCB. The PCB now allows the user to break off the small part, which holds the methane sensor. This part is then soldered onto the board, ensuring the pins A, B, C and D are correctly placed to reduce the height of the sensor
* One downside to this is that you have to use an other Arduino Core and this core should be installed in the Arduino IDE. In the version 4 we will use the [MiniCore](https://github.com/MCUdude/Minicore). You also need an USBASP ISP which is basically a programmer for the sensor, that allows you to upload the code to your sensor. The USBASP ISP can easily be bought online through many sellers.
* The new version of the sensor has also recieved some minor updates to the code. The sensor now has three LEDs, one that is always turned on when the sensor is powered, this LED is labeled 'Power LED'. Another LED is located closed to the SD card holder, it is labeled 'SD LED' and blinks whenever data is written to the SD card. The last LED is labeled 'Status LED', this LED provides feedback on whether the sensor is running as expected. In the event of a corrupt SD card, or in case the sensor cannot connect the CO2 sensor, temperature and humidity sensor or the analog-to-digital chip, that reads out the methane sensor, the LED will be continous, and action must be taken. If the LED lights up every 3 sampling (6 seconds) then no errors have been detected. 

## A step-by-step guide to building methane and CO<sub>2</sub> sensors and an automated floating chamber


### Building the sensor
* Buy all the equipment from the [Equipment list](Version%204/PCB/Equipment%20list%20V4.docx). Most items can be bought from your PCB manufacturer, however, some parts need to be bought from other companies. When ordering the printed circuit boards (PCB) from a manufacturer a bill of materials (bom) and component placement list (cpl) is available, which includes parts numbers for JLCPCB, thus many of the parts can be assemblied by the PCB manufacturer which is advised. Remember to indicate that PCB assembly on both sides should be done by the manufacturer (On JLCPCB this is indicated at the bottom of the site by a checkbox) and it's advised to check the box for "Confirm Parts Placement", just to be sure parts are correctly placed.
Additionally, a soldering iron, flux pen, solder wire and solder wick are needed.
* Solder four pin headers to the K33 ELG CO<sub>2</sub> sensor, so they and the screws match the lineups on the PCB.  
* Install the coin-cell battery to power the real-time-clock (RTC).
* Install the SD card. 
* Download the Arduino IDE, which can be downloaded from the [Arduino website](https://www.arduino.cc/en/software).
* Download the [Arduino sketch for the sensor](Version%204/Arduino%20code) and the [RTC](RTC/RTC_set/RTC_set.ino). Make sure to download and install the needed [libraries](Arduino%20libraries/) first.
* Install MiniCore by following the [instructions here](https://github.com/MCUdude/Minicore?tab=readme-ov-file#how-to-install).
* Connect the sensor to your computer using the USBASP ISP.
* Find the 'Tools' menubar. Under Board > MiniCore > ATmega328
* Changes the options so it looks like this
<br><img src="Images/Arduino%20tools.png" width="250"> <br>
* Start by uploading the RTC sketch using the Arduino IDE, to set the clock to the time of the computer. This is done by pressing the Upload bottom (Arrow).
* Then upload the desired sensor sketch which can be found [here](Version%204/Arduino%20code/).
* The sensor should now be running. You might not get values from the K33 ELG CO2 sensor if you only power the sensor through the USBASP ISP as it needs at least 9 V. 
* To view the data output turn off the sensor and readout the SD card to find the datalog.csv file.
* For connection from a 12 V battery to the sensor, a two-conductor wire needs to be connected to the battery. Make sure the polarity is correct.  

> [!WARNING]  
> All Figaro CH4 sensors should be calibrated to achieve the correct calibration coefficient. See [(Sø et al., 2024)](https://doi.org/10.1029/2024JG008035) for more information.   

I'm hoping to make a video on how to order and assemble the sensor soon. 

### Building the chamber
* A bucket has been used to build these sensors, it’s important however that tests and considerations are made if changing the dimensions of the bucket, as this might cause the sensor to be too close to the water surface and thereby be shorted due to water damage. Here a bucket with a volume of 13.5 l and surface area of 0.0615 m2 is used.
* Wrap the outside of the bucket in aluminum foil tape to increase the reflectance. 
* Drill two holes on each side of the bucket, approximately 3/4 of the distance from the top of the bucket to the bottom. 
* Get two storage boxes to hold the sensor and the pump. The boxes should be able to get inside the bucket. 
* Drill two holes in the storage boxes and in the bucket, which align in pairs.
* Mount one storage box on the inside of the bucket and one on the outside using screws and bolts in the holes drilled before. Add an O-ring to ensure no air is exchanged through the drilled holes. 
* Drill a small hole through the two storage boxes and the bucket to allow wires from the fan to reach the sensor inside the bucket. 
* Place the fan in the storage box which is located on the outside of the bucket. 
* Push the wires from the fan through the hole drilled before and seal the hole with some silicone. 
* Place the sensor in the storage box on the inside of the bucket and connect the fan through the push terminal. Be sure to consider the polarity.
* Now create a floatation device using a 3 cm thick styrofoam sheet. The sheet should be big enough to cut a hole in the middle for the bucket.
* Drill holes in the side of the bucket at the very top, to allow for cable ties to attach the floatation device to the bucket. 

<img src="Version%204/V4.png" width="500">

### Calibrating the CO<sub>2</sub> sensor
The CO<sub>2</sub> sensor can be calibrated to background concentration or 0 concentration. This is done by shorting two connections on the sensor. To initiate the background calibration, on the photo above, the two pins that should be connected have a pin header soldered to them. To find more information visit the [CO<sub>2</sub> sensors website](https://senseair.com/product/k33-elg/).
