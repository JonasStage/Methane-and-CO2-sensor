// By David Bastviken and Nguyen Thanh Duc, Linkoping University, Sweden. 
// Modified for automated fluxes and lower power consumption by Jonas Stage Sø, University of Southern Denmark, Denmark.
// Thanks to cactus.io and Adafruit for code components. For setup of RTC, see separate logger shield documentation.
#include <SPI.h>
#include <Wire.h>
#include <SimpleDHT.h>
#include <RTClib.h>
#include "cactus_io_DHT22.h"
#include <SdFat.h>
#include "Arduino.h"
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MAX31865.h>
SdFat SD;

#define POWER_PIN 5                     // Used as a switch to turn sensors off when sleeping through D5 pin
int SampleNumber = 0 ;                  // Used for sample number in output file

#define LOG_INTERVAL 2000               // Millisecond between logging entries (reduce to take more/faster data). milliseconds before writing the logged data permanently to disk. LOG_INTERVAL write each time (safest)
#define SYNC_INTERVAL 5000              // Millisecond between calls to flush() - to write data to the card

uint32_t syncTime = 0;                  // Time of last sync()
#define ECHO_TO_SERIAL   1              // If you want data to show in serial port
#define WAIT_TO_START    0              // Wait for serial input in setup()

#define Vb A0                           // Battery voltage. A0 pin
int CH4s = 0;                           // Variables used to store data
int CH4r = 0;                           // Variables used to store data
int Vbat = 0;                           // Variables used to store data
float CH4smV = 0;                       // Variables used to store data
float CH4rmV = 0;                       // Variables used to store data
float VbatmV = 0;                       // Variables used to store data                              
float mV = 5000;                        // Variables used to store data
float steps = 1024;                     // Variables used to store data
float CH4worm_kort = 0;
float CH4worm_lang = 0;

RTC_PCF8523 RTC;                        // define the Real Time Clock object

File logfile;                           // the logging file

Adafruit_ADS1115 ads; 

Adafruit_MAX31865 pt1000_1 = Adafruit_MAX31865(8);
Adafruit_MAX31865 pt1000_2 = Adafruit_MAX31865(9);
#define RREF      4300.0
#define RNOMINAL  1000.0


void error(char *str)                   // Halt if error
{
  Serial.print("error: ");
  Serial.println(str);
  digitalWrite(13, HIGH); // red LED indicates error
  while (1); //halt command
}

void wakeSensor() { 
  // This command serves as a wakeup to the CO2 sensor, for K33?ELG/BLG Sensors Only 
   
  // You'll have the look up the registers for your specific device, but the idea here is simple: 
  // 1. Disabled the I2C engine on the AVR 
  // 2. Set the Data Direction register to output on the SDA line 
  // 3. Toggle the line low for ~1ms to wake the micro up. Enable I2C Engine 
  // 4. Wake a millisecond. 
   
  TWCR &= ~(1<<2); // Disable I2C Engine 
  DDRC |= (1<<4); // Set pin to output mode 
  PORTC &= ~(1<<4); // Pull pin low 
  delay(1); 
  PORTC |= (1<<4); // Pull pin high again 
  TWCR |= (1<<2); // I2C is now enabled 
  delay(1);     
}


void setup(void)
{
  pinMode(POWER_PIN, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);
  Serial.begin(9600);
  Serial.println();
  RTC.begin(); 
  digitalWrite(POWER_PIN, HIGH);
  pt1000_1.begin(MAX31865_4WIRE);  // set to 2WIRE or 4WIRE as necessary
  pt1000_2.begin(MAX31865_4WIRE);  // set to 2WIRE or 4WIRE as necessary

if (!ads.begin(0x49)) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }
    while (!Serial) {
    ;                              // wait for serial port to connect. Needed for native USB port only
  }

  Serial.print("Initializing SD card...");
  pinMode(10, OUTPUT);

  // see if the card is present and can be initialized:
  if (!SD.begin()) {
    error("Card failed, or not present");
    while(1);
  }
  Serial.println("card initialized.");


  // connect to RTC
  Wire.begin();
  if (!RTC.begin()) {
    logfile.println("RTC failed");
#if ECHO_TO_SERIAL
    Serial.println("RTC failed");
#endif  //ECHO_TO_SERIAL
  } 

  if (! RTC.initialized()) {
    Serial.println("RTC is NOT running!");
    RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // following line sets the RTC to the date & time this sketch was compiled
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

  if (SD.exists("datalog.csv")) {
    Serial.println("datalog.csv exists.");
    File logfile = SD.open("datalog.csv", FILE_WRITE);
    logfile.close();
    Serial.println("millis,stampunix,datetime,CH4smV, CH4rmV, CH4worm_kort, CH4worm_lang, pt1000_kort, pt1000_lang, VbatmV, SampleNumber");
} else {
    Serial.println("datalog.csv doesn't exist.");
    Serial.println("Creating datalog.csv...");
    logfile = SD.open("datalog.csv", FILE_WRITE);
    logfile.println("millis,stampunix,datetime,CH4smV, CH4rmV, CH4worm_kort, CH4worm_lang, pt1000_kort, pt1000_lang, VbatmV, SampleNumber");
    logfile.close();
    Serial.println("millis,stampunix,datetime,CH4smV, CH4rmV, CH4worm_kort, CH4worm_lang, pt1000_kort, pt1000_lang, VbatmV, SampleNumber");
}
}

void loop() {
  DateTime now;
  digitalWrite(POWER_PIN, HIGH);

  SampleNumber++;

  // delay for the amount of time we want between readings
  delay((LOG_INTERVAL - 1) - (millis() % LOG_INTERVAL));
 
  // log milliseconds since starting
  uint32_t m = millis();

#if ECHO_TO_SERIAL
  Serial.print(m);         // milliseconds since start
  Serial.print(", ");
#endif

  // fetch the time
  now = RTC.now();
  // log time
  logfile = SD.open("datalog.csv", FILE_WRITE);
  logfile.print(m);           // milliseconds since start
  logfile.print(", ");
  logfile.print(now.unixtime()); // seconds since 1/1/1970
  logfile.print(", ");
  //logfile.print('"');
  logfile.print(now.year(), DEC);
  logfile.print("/");
  logfile.print(now.month(), DEC);
  logfile.print("/");
  logfile.print(now.day(), DEC);
  logfile.print(" ");
  logfile.print(now.hour(), DEC);
  logfile.print(":");
  logfile.print(now.minute(), DEC);
  logfile.print(":");
  logfile.print(now.second(), DEC);
  logfile.close();
  //logfile.print('"');
#if ECHO_TO_SERIAL
  Serial.print(now.unixtime()); // seconds since 1/1/1970
  Serial.print(", ");
  //Serial.print('"');
  Serial.print(now.year(), DEC);
  Serial.print("/");
  Serial.print(now.month(), DEC);
  Serial.print("/");
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(":");
  Serial.print(now.minute(), DEC);
  Serial.print(":");
  Serial.print(now.second(), DEC);
  //Serial.print('"');
#endif //ECHO_TO_SERIAL

  int16_t adc0, adc1, adc2, adc3 = 0;
  float volts0, volts1, volts2, volts3 = 0;

  adc0 = ads.readADC_SingleEnded(0);
  delay(100);
  adc1 = ads.readADC_SingleEnded(1);
  delay(100);
  adc2 = ads.readADC_SingleEnded(2);
  delay(100);
  adc3 = ads.readADC_SingleEnded(3);
  delay(100);
  CH4smV = ads.computeVolts(adc0)*1000;
  CH4rmV = ads.computeVolts(adc1)*1000;
  CH4worm_kort = ads.computeVolts(adc2)*1000;
  CH4worm_lang = ads.computeVolts(adc3)*1000;
  delay(100);
  Vbat = analogRead(Vb); //read CH4 Vref
  VbatmV = Vbat * (mV / steps); //convert pin reading to mV, NOT YET correcting for the voltage divider.
  
float pt_kort, pt_lang = 0;
pt_kort = pt1000_1.temperature(RNOMINAL, RREF);
pt_lang = pt1000_2.temperature(RNOMINAL, RREF);
  

  logfile = SD.open("datalog.csv", FILE_WRITE);
  logfile.print(", ");
  logfile.print(CH4smV);
  logfile.print(", ");
  logfile.print(CH4rmV);
  logfile.print(", ");
  logfile.print(CH4worm_kort);
  logfile.print(", ");
  logfile.print(CH4worm_lang);
  logfile.print(", ");
  logfile.print(pt_kort);
  logfile.print(", ");
  logfile.print(pt_lang);
  logfile.print(", ");
  logfile.print(VbatmV);
  logfile.print(", ");
  logfile.print(SampleNumber);
  logfile.println();
  logfile.close();
#if ECHO_TO_SERIAL
  Serial.print(", ");
  Serial.print(CH4smV);
  Serial.print(", ");
  Serial.print(CH4rmV);
  Serial.print(", ");
  Serial.print(CH4worm_kort);
  Serial.print(", ");
  Serial.print(CH4worm_lang);
  Serial.print(", ");
  Serial.print(pt_kort);
  Serial.print(", ");
  Serial.print(pt_lang); 
  Serial.print(", ");
  Serial.print(VbatmV);
  Serial.print(", ");
  Serial.print(SampleNumber);
  Serial.println();

#endif //ECHO_TO_SERIAL
 
  // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
  // which uses power and takes time
  if ((millis() - syncTime) < SYNC_INTERVAL) return;
  syncTime = millis();
  logfile.flush();  

}
