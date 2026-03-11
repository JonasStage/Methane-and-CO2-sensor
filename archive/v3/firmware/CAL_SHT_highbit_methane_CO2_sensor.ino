// By David Bastviken and Nguyen Thanh Duc, Linkoping University, Sweden. 
// Modified for automated fluxes and lower power consumption by Jonas Stage Sø, University of Southern Denmark, Denmark.
// Thanks to cactus.io and Adafruit for code components. For setup of RTC, see separate logger shield documentation.
#include <SPI.h>
#include <Wire.h>
#include <RTClib.h>
#include "Adafruit_SHT4x.h"
#include <SdFat.h>
#include "Arduino.h"
#include <Adafruit_ADS1X15.h>
SdFat SD;

int SampleNumber = 0 ;                  // Used for sample number in output file

#define LOG_INTERVAL 2000               // Millisecond between logging entries (reduce to take more/faster data). milliseconds before writing the logged data permanently to disk. LOG_INTERVAL write each time (safest)
#define SYNC_INTERVAL 5000              // Millisecond between calls to flush() - to write data to the card

uint32_t syncTime = 0;                  // Time of last sync()
#define ECHO_TO_SERIAL   1              // If you want data to show in serial port
#define WAIT_TO_START    0              // Wait for serial input in setup()

Adafruit_SHT4x sht4 = Adafruit_SHT4x(); // Definename for sht (temperature and RH)

#define Vb A0                           // Battery voltage. A0 pin
int CH4s = 0;                           // Variables used to store data
int CH4r = 0;                           // Variables used to store data
int Vbat = 0;                           // Variables used to store data
float CH4smV = 0;                       // Variables used to store data
float CH4rmV = 0;                       // Variables used to store data
float VbatmV = 0;                       // Variables used to store data                              
float mV = 5000;                        // Variables used to store data
float steps = 1024;                     // Variables used to store data

RTC_PCF8523 RTC;                        // define the Real Time Clock object

Adafruit_ADS1115 ads;

#define light_pin 4
#define PUMP_PIN 6                // Pump turned on and off through D6 pin

#define SD_CS_PIN 10                    // for the data logging shield, we use digital pin 10 for the SD cs line
File logfile;                           // the logging file

void error(char *str)                   // Halt if error
{
  Serial.print("error: ");
  Serial.println(str);
  digitalWrite(light_pin, HIGH); // red LED indicates error
  while (1); //halt command
}

void setup(void)
{
  pinMode(light_pin, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT); //Start pump
  Serial.begin(9600);
  Serial.println();
  RTC.begin(); 

    while (!Serial) {
    ;                              // wait for serial port to connect. Needed for native USB port only
  }

  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(SD_CS_PIN)) {
    error("Card failed, or not present");
    while(1);
  }
  Serial.println("card initialized.");

if (!ads.begin(0x49)) {
    error("Failed to initialize ADS.");
    while (1);
  }

if (! sht4.begin()) {
    error("Couldn't find SHT4x");
    while (1);
  }
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
  Serial.println("<<<<< THIS IS THE CALIBRATION FILE, PUMP WILL NOT BE TURNED ON >>>>>");
  
  if (SD.exists("datalog.csv")) {
    Serial.println("datalog.csv exists.");
    File logfile = SD.open("datalog.csv", FILE_WRITE);
    logfile.close();
    Serial.println("millis,stampunix,datetime,RH,tempC,CH4smV, CH4rmV, VbatmV, SampleNumber");
} else {
    Serial.println("datalog.csv doesn't exist.");
    Serial.println("Creating datalog.csv...");
    logfile = SD.open("datalog.csv", FILE_WRITE);
    logfile.println("millis,stampunix,datetime,RH,tempC,CH4smV, CH4rmV, VbatmV, SampleNumber");
    logfile.close();
    Serial.println("millis,stampunix,datetime,RH,tempC,CH4smV, CH4rmV, VbatmV, SampleNumber");
}
}

char time_to_read_CO2 = 1;
char n_delay_wait = 0;


void loop() {
  digitalWrite(PUMP_PIN, HIGH); // Just to ensure the pump is functioning
  DateTime now;
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
sensors_event_t rh,temp;
  // Reading temperature or humidity takes about 250 milliseconds.
  // Sensor readings may also be up to 2 seconds 'old' (its a slow sensor)
sht4.getEvent(&rh, &temp);// populate temp and humidity objects with fresh data

int16_t adc0, adc1, adc2, adc3 = 0;
  float volts0, volts1, volts2, volts3 = 0;
  
  delay(100);
  adc0 = ads.readADC_SingleEnded(0);
  delay(100);
  adc1 = ads.readADC_SingleEnded(1);
  delay(100);
  CH4smV = ads.computeVolts(adc0)*1000;
  CH4rmV = ads.computeVolts(adc1)*1000;
  delay(100);
  Vbat = analogRead(Vb); //read CH4 Vref
  VbatmV = Vbat * (mV / steps); //convert pin reading to mV, NOT YET correcting for the voltage divider.

  logfile = SD.open("datalog.csv", FILE_WRITE);
  logfile.print(", ");
  logfile.print(rh.relative_humidity);
  logfile.print(", ");
  logfile.print(temp.temperature);
  logfile.print(", ");
  logfile.print(CH4smV);
  logfile.print(", ");
  logfile.print(CH4rmV);
  logfile.print(", ");
  logfile.print(VbatmV);
  logfile.print(", ");
  logfile.print(SampleNumber);
  logfile.println();
  logfile.close();
#if ECHO_TO_SERIAL
  Serial.print(", ");
  Serial.print(rh.relative_humidity);
  Serial.print(", ");
  Serial.print(temp.temperature);
  Serial.print(", ");
  Serial.print(CH4smV);
  Serial.print(", ");
  Serial.print(CH4rmV);
  Serial.print(", ");
  Serial.print(VbatmV);
  Serial.print(", ");
  Serial.print(SampleNumber);

#endif //ECHO_TO_SERIAL

#if ECHO_TO_SERIAL
  Serial.println();
#endif // ECHO_TO_SERIAL

if ((SampleNumber % 3) == 0) {
    digitalWrite(light_pin, HIGH);
    delay(10);
    digitalWrite(light_pin, LOW);
}
 
  // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
  // which uses power and takes time
  if ((millis() - syncTime) < SYNC_INTERVAL) return;
  syncTime = millis();
  logfile.flush();  

}