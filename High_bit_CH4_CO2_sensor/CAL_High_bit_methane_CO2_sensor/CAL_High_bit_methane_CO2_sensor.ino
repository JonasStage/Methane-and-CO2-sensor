// By David Bastviken and Nguyen Thanh Duc, Linkoping University, Sweden. 
// Modified for automated fluxes and lower power consumption by Jonas Stage Sø, University of Southern Denmark, Denmark.
// Thanks to cactus.io and Adafruit for code components. For setup of RTC, see separate logger shield documentation.
#include <SPI.h>
#include <Wire.h>
#include <RTClib.h>
#include "DHT.h"
#include <SdFat.h>
#include "Arduino.h"
#include <Adafruit_ADS1X15.h>
SdFat SD;

#define POWER_PIN 5                     // Used as a switch to turn sensors off when sleeping through D5 pin
int SampleNumber = 0 ;                  // Used for sample number in output file
int PumpCycle = 1 ;                     // Used for seperating pumpcycles 

int sleepCnt = 0;                       // Used for counting number of sleeping mills

/// Pump function
const byte PUMP_PIN = 6;                // Pump turned on and off through D6 pin

#define LOG_INTERVAL 2000               // Millisecond between logging entries (reduce to take more/faster data). milliseconds before writing the logged data permanently to disk. LOG_INTERVAL write each time (safest)
#define SYNC_INTERVAL 5000              // Millisecond between calls to flush() - to write data to the card

uint32_t syncTime = 0;                  // Time of last sync()
#define ECHO_TO_SERIAL   1              // If you want data to show in serial port
#define WAIT_TO_START    0              // Wait for serial input in setup()

#define dht_pin 7                       // DHT22 (RH_T) data pin. here D7 pin.
DHT dht;                                // Initialize DHT sensor for normal 16mhz Arduino.

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

#define error_pin 4
#define running_pin 3

#define SD_CS_PIN 10                    // for the data logging shield, we use digital pin 10 for the SD cs line
File logfile;                           // the logging file

void error(char *str)                   // Halt if error
{
  Serial.print("error: ");
  Serial.println(str);
  digitalWrite(error_pin, HIGH); // red LED indicates error
  while (1); //halt command
}

double RHValue = 0;                     // Variables used to store data
double TempValue = 0;                   // Variables used to store data
double CO2Value = 0;                    // Variables used to store data
int co2Addr = 0x7F;                     // This is the modified address of the CO2 sensor, 7bits shifted left (defaul 0x68, but collide with RTC chip)

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
///////////////////////////////////////////////////////////////////
// Function : void initPoll()
// Executes : Tells sensor to take a measurement.
// Notes    : A fuller implementation would read the register back and
//            ensure the flag was set, but in our case we ensure the poll
//            period is >25s and life is generally good.
///////////////////////////////////////////////////////////////////
void initPoll() { 
 Wire.beginTransmission(co2Addr); 
 Wire.write(0x11); 
 Wire.write(0x00); 
 Wire.write(0x60); 
 Wire.write(0x35); 
 Wire.write(0xA6); 
  
 Wire.endTransmission(); 
 delay(20);  
 Wire.requestFrom(co2Addr, 2); 
   
 byte i = 0; 
 byte buffer[2] = {0, 0}; 

 while(Wire.available()) { 
     buffer[i] = Wire.read(); 
     i++; 
 }   
}

/////////////////////////////////////////////////////////////////// 
// Function : double readRH() 
// Returns  : The current RH Value, ?1 if error has occured 
/////////////////////////////////////////////////////////////////// 

double readRH() {
 int RH_value = 0;   // We will store the RH value inside this variable.  
 digitalWrite(13, HIGH);                
 
 ////////////////////////// 
 /* Begin Write Sequence */ 
 ////////////////////////// 
  
 Wire.beginTransmission(co2Addr); 
 Wire.write(0x22); //Command number 2 (Read Ram, 2 bytes)
 Wire.write(0x00); //Sensor address in ?? EEPROM ??
 Wire.write(0x14); //Two bytes starting from 0x14 (high byte) and 0x15 (low byte)
 Wire.write(0x36); //Checksum
  
 Wire.endTransmission(); 
 
 delay(20); 
 
 /////////////////////////   
 /* Begin Read Sequence */ 
 /////////////////////////  
   
 Wire.requestFrom(co2Addr, 4); 
   
 byte i = 0; 
 byte buffer[4] = {0, 0, 0, 0}; 
 
while(Wire.available()) { 
     buffer[i] = Wire.read(); 
     i++; 
 }   
   
 RH_value = 0; 
 RH_value |= buffer[1] & 0xFF;
 RH_value = RH_value << 8; 
 RH_value |= buffer[2] & 0xFF;
 Serial.print("RH  Data ");
 Serial.print(buffer[0], HEX);
 Serial.print("|"); 
 Serial.print(buffer[1], HEX);
 Serial.print("|");  
 Serial.print(buffer[2], HEX);
 Serial.print("|"); 
 Serial.println(buffer[3], HEX); 
 
 byte sum = 0;                              //Checksum Byte 
 sum = buffer[0] + buffer[1] + buffer[2];   //Byte addition utilizes overflow 
  
 if(sum == buffer[3]) { 
     // Success! 
     digitalWrite(13, LOW); 
     delay(10); 
     return ((double)RH_value / (double) 100); 
 }   
 else { 
  // Failure!  
  digitalWrite(13, LOW);
  delay(10);
  return ((double) -1); 
 }   
} 

/////////////////////////////////////////////////////////////////// 
// Function : double readTemp() 
// Returns  : The current Temperature Value, ?1 if error has occured 
/////////////////////////////////////////////////////////////////// 

double readTemp() {
 int Temp_value = 0;   // We will store the temperature value inside this variable.  
 digitalWrite(13, HIGH);                
 
 ////////////////////////// 
 /* Begin Write Sequence */ 
 ////////////////////////// 
  
 Wire.beginTransmission(co2Addr); //int K33 == 0x68
 Wire.write(0x22); //Command number 2 (Read Ram, 2 bytes)
 Wire.write(0x00); //Sensor address in ?? EEPROM ??
 Wire.write(0x12); //Two bytes starting from 0x12 (high byte) and 0x13 (low byte)
 Wire.write(0x34); //Checksum
  
 Wire.endTransmission(); 
 
 delay(20); 
 
  
 /////////////////////////   
 /* Begin Read Sequence */ 
 /////////////////////////  
   
 Wire.requestFrom(co2Addr, 4); 
   
 byte i = 0; 
 byte buffer[4] = {0, 0, 0, 0}; 
 
while(Wire.available()) { 
     buffer[i] = Wire.read(); 
     i++; 
 }   
   
 Temp_value = 0; 
 Temp_value |= buffer[1] & 0xFF;
 Temp_value = Temp_value << 8; 
 Temp_value |= buffer[2] & 0xFF;
 Serial.print("T   Data ");
 Serial.print(buffer[0], HEX);
 Serial.print("|"); 
 Serial.print(buffer[1], HEX);
 Serial.print("|");  
 Serial.print(buffer[2], HEX);
 Serial.print("|"); 
 Serial.println(buffer[3], HEX); 
 
 byte sum = 0;                              //Checksum Byte 
 sum = buffer[0] + buffer[1] + buffer[2];   //Byte addition utilizes overflow 
  
 if(sum == buffer[3]) { 
     // Success! 
     digitalWrite(13, LOW); 
     delay(10); 
     return ((double)Temp_value / (double) 100); 
 }   
 else { 
  // Failure!  
  digitalWrite(13, LOW);
  delay(10);
  return ((double) -1); 
 }   
} 


/////////////////////////////////////////////////////////////////// 
// Function : double readCO2() 
// Returns  : The current CO2 Value, ?1 if error has occured 
/////////////////////////////////////////////////////////////////// 

double readCO2() {
 int CO2_value = 0;   // We will store the temperature value inside this variable.  
 digitalWrite(13, HIGH);                
 
 ////////////////////////// 
 /* Begin Write Sequence */ 
 ////////////////////////// 
  
 Wire.beginTransmission(co2Addr); //int K33 == 0x68
 Wire.write(0x22); //Command number 2 (Read Ram, 2 bytes)
 Wire.write(0x00); //Sensor address in ?? EEPROM ??
 Wire.write(0x08); //Two bytes starting from 0x08 (high byte) and 0x09 (low byte). They contain the CO2 data
 Wire.write(0x2A); //Checksum
  
 Wire.endTransmission(); 
 
 delay(50); 
 
 /////////////////////////   
 /* Begin Read Sequence */ 
 /////////////////////////  
   
 Wire.requestFrom(co2Addr, 4); 
   
 byte i = 0; 
 byte buffer[4] = {0, 0, 0, 0}; 
 
while(Wire.available()) { 
     buffer[i] = Wire.read(); 
     i++; 
 }   
   
 CO2_value = 0; 
 CO2_value |= buffer[1] & 0xFF;
 CO2_value = CO2_value << 8; 
 CO2_value |= buffer[2] & 0xFF;
 Serial.print("CO2 Data ");
 Serial.print(buffer[0], HEX);
 Serial.print("|"); 
 Serial.print(buffer[1], HEX);
 Serial.print("|");  
 Serial.print(buffer[2], HEX);
 Serial.print("|"); 
 Serial.println(buffer[3], HEX); 
 
 byte sum = 0;                              //Checksum Byte 
 sum = buffer[0] + buffer[1] + buffer[2];   //Byte addition utilizes overflow 
  
 if(sum == buffer[3]) { 
     // Success! 
     digitalWrite(13, LOW); 
     delay(10); 
     return ((double)CO2_value * (double) 1); 
 }   
 else { 
  // Failure!  
  digitalWrite(13, LOW);
  delay(10);
  return ((double) -1); 
 }   
} 
  
void setup(void)
{
  pinMode(error_pin, OUTPUT);
  pinMode(running_pin, OUTPUT);
  pinMode(POWER_PIN, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT); //Start pump
  Serial.begin(9600);
  Serial.println();
  dht.setup(dht_pin); //start RH_T_sensor
  RTC.begin(); 
  digitalWrite(POWER_PIN, HIGH);

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
    Serial.println("millis,stampunix,datetime,RH%,tempC,CH4smV, CH4rmV, VbatmV, K33_RH, K33_Temp, K33_CO2, SampleNumber, PumpCycle");
} else {
    Serial.println("datalog.csv doesn't exist.");
    Serial.println("Creating datalog.csv...");
    logfile = SD.open("datalog.csv", FILE_WRITE);
    logfile.println("millis,stampunix,datetime,RH%,tempC,CH4smV, CH4rmV, VbatmV, K33_RH, K33_Temp, K33_CO2, SampleNumber, PumpCycle");
    logfile.close();
    Serial.println("millis,stampunix,datetime,RH%,tempC,CH4smV, CH4rmV, VbatmV, K33_RH, K33_Temp, K33_CO2, SampleNumber, PumpCycle");
}

digitalWrite(PUMP_PIN, HIGH); // Just to ensure the pump is functioning
delay(3000);
digitalWrite(PUMP_PIN, LOW);
Serial.println("Running calibration code");
}

char time_to_read_CO2 = 1;
char n_delay_wait = 0;


void loop() {
  DateTime now;
  digitalWrite(POWER_PIN, HIGH);

  SampleNumber++;

if ((SampleNumber % 9) == 0) {
  initPoll();
}

 if (time_to_read_CO2 == 1) {
    wakeSensor();
    delay(50);
    initPoll();
    delay(50);
    CO2Value = readCO2();
    delay(20);
    RHValue = readRH();
    delay(20);
    TempValue = readTemp();

    // if(RHValue >= 0) {
    //       Serial.print("RH: ");
    //       Serial.print(RHValue);
    //       Serial.print("% | Temp: ");
    //       Serial.print(TempValue);
    //       Serial.print("C | CO2: ");
    //       Serial.print(CO2Value, 0);
    //       Serial.println("ppm");
    //       Serial.println();
    // }
    // else {
    //       Serial.println(" | Checksum failed / Communication failure");
    // }
    time_to_read_CO2 = 0;
  }

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
float rh,temp;
  // Reading temperature or humidity takes about 250 milliseconds.
  // Sensor readings may also be up to 2 seconds 'old' (its a slow sensor)
rh = dht.getHumidity();
temp = dht.getTemperature();
  // Check if any reads failed and exit early (to try again).
  if (!dht.getStatusString() == "OK") {
    error("DHT fejl");
    return;
  }
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
  logfile.print(rh);
  logfile.print(", ");
  logfile.print(temp);
  logfile.print(", ");
  logfile.print(CH4smV);
  logfile.print(", ");
  logfile.print(CH4rmV);
  logfile.print(", ");
  logfile.print(VbatmV);

  logfile.print(", ");
  logfile.print(RHValue);
  logfile.print(", ");
  logfile.print(TempValue);
  logfile.print(", ");
  logfile.print(CO2Value);
  logfile.print(", ");
  logfile.print(SampleNumber);
  logfile.print(", ");
  logfile.print(PumpCycle);
  logfile.println();
  logfile.close();
#if ECHO_TO_SERIAL
  Serial.print(", ");
  Serial.print(rh);
  Serial.print(", ");
  Serial.print(temp);
  Serial.print(", ");
  Serial.print(CH4smV);
  Serial.print(", ");
  Serial.print(CH4rmV);
  Serial.print(", ");
  Serial.print(VbatmV);
  Serial.print(", ");
  Serial.print(RHValue);
  Serial.print(", ");
  Serial.print(TempValue);
  Serial.print(", ");
  Serial.print(CO2Value);
  Serial.print(", ");
  Serial.print(SampleNumber);
  Serial.print(", ");
  Serial.print(PumpCycle);

#endif //ECHO_TO_SERIAL

#if ECHO_TO_SERIAL
  Serial.println();
#endif // ECHO_TO_SERIAL

if ((SampleNumber % 3) == 0) {
    digitalWrite(running_pin, HIGH);
    delay(10);
    digitalWrite(running_pin, LOW);
}

  if (time_to_read_CO2 == 0 ) {
    if (n_delay_wait < 9)
      n_delay_wait ++;
    else {
      time_to_read_CO2 = 1;
      Serial.print("Time to read K33 sensor: ");
      n_delay_wait = 0;
    }
  }


 
  // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
  // which uses power and takes time
  if ((millis() - syncTime) < SYNC_INTERVAL) return;
  syncTime = millis();
  logfile.flush();  

}
