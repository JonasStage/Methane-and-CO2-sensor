# CH4 and CO2 DIY sensor with automated aeration

This repository is dedicated to CH4 and CO2 sensors created by Jonas Stage Sø (Sø et al., 2023). 
Sensors are made following Bastviken et al. (2020), modified for automated fluxes, higher resolution and lower power consumption by Jonas Stage Sø, University of Southern Denmark, Denmark. 

------------------

# A step-by-step guide to building methane and CO2 sensors and an automated floating chamber


## Building the sensor
* Buy all the equipment necessary. A thorough equipment list can be found in the folder “/Version 3”. Most items can be bought from your PCB manufacturer, however, some parts need to be bought from other companies. Additionally, a soldering iron, flux pen, solder wire and solder wick are needed.
* Solder four pin headers to the K33 ELG CO2 sensor, so they and the screws match the lineups on the PCB. 
* Solder the remaining parts to the PCB. 
* Add a battery to the Adafruit Datalogger shield to power the real-time-clock (RTC).
* Download the Arduino IDE, which can be downloaded from the Arduino website.
* Download the Arduino sketch for the sensor and the RTC. Make sure to download and install the needed libraries first. 
* Start by uploading the RTC sketch using the Arduino IDE, to set the clock to the time of the computer. This is done by selecting the port that is connected to the Arduino and pressing the Upload bottom (Arrow).
* Then upload the sensor sketch similarly to the previous step.
* The sensor should now be running. You might not get values from the K33 ELG CO2 sensor if you only power the sensor through USB as it needs at least 9 V. 
* The view the data output either turn off the sensor and dismount the SD card to find the datalog.csv file, or open the serial monitor in the Arduino IDE by pressing the magnifying glass in the top right corner. 
* For connection from a 12 V battery to the sensor, a two-conductor wire needs to be crimped and the battery connector should be attached. Connect the wire to the battery.  Make sure the polarity is correct.



## Building the chamber
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

------------------

Files include PCB designs and code for setting up sensors and processing data.

Everything is licensed under the MIT License.


As of this moment, the newest version is version 3. 
Comparisons of this sensor against a Los Gatos Research Microportable Greenhouse Gas Analyzer (GLA131-GPC AAB, Switzerland), can be found in an upcoming paper.

![Skærmbillede 2023-08-14 kl  10 37 44 2](https://github.com/JonasStage/Methane-and-CO2-sensor/assets/57667863/85bc4e85-a9ac-4017-9cfc-d753d11304c8) ![Billede1](https://github.com/JonasStage/Methane-and-CO2-sensor/assets/57667863/3e51f4ee-0072-430b-99f7-15fcb3a9be2a)



------------------

Citations


Sø, J. S., Sand-Jensen, K., Martinsen, K. T., Polauke, E., Kjær, J. E., Reitzel, K., & Kragh, T. (2023). Methane and carbon dioxide fluxes at high spatiotemporal resolution from a small temperate lake. Science of the Total Environment, 878, 162895. doi:https://doi.org/10.1016/j.scitotenv.2023.162895

  Bastviken, D., Nygren, J., Schenk, J., Parellada Massana, R., and Duc, N. T.: Technical note: Facilitating the use          of low-cost methane (CH4) sensors in flux chambers – calibration, data processing, and an open-source make-it-yourself logger, Biogeosciences, 17, 3659–3667, https://doi.org/10.5194/bg-17-3659-2020, 2020.
