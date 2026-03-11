// By David Bastviken and Nguyen Thanh Duc, Linkoping University, Sweden. 
// Modified for lower power consumption by Jonas Stage Sø, University of Southern Denmark, Denmark.
// Thanks to cactus.io and Adafruit for code components. For setup of RTC, see separate logger shield documentation.
#include <SPI.h>
#include <Wire.h>
#include <SimpleDHT.h>
#include <RTClib.h>
#include "cactus_io_DHT22.h"
#include <SdFat.h>
#include "Arduino.h"
SdFat SD;

#define POWER_PIN 5                     // Used as a switch to turn sensors off when sleeping through D5 pin
int SampleNumber = 0 ;                  // Used for sample number in output file

/// Pump function
#define PUMP_PIN 6                      // Pump turned on and off through D6 pin

#define LOG_INTERVAL 2000               // Millisecond between logging entries (reduce to take more/faster data). milliseconds before writing the logged data permanently to disk. LOG_INTERVAL write each time (safest)
#define SYNC_INTERVAL 5000              // Millisecond between calls to flush() - to write data to the card

uint32_t syncTime = 0;                  // Time of last sync()
#define ECHO_TO_SERIAL   1              // If you want data to show in serial port
#define WAIT_TO_START    0              // Wait for serial input in setup()

#define DHT22_PIN 7                     // DHT22 (RH_T) data pin. here D7 pin.
DHT22 dht(DHT22_PIN);                   // Initialize DHT sensor for normal 16mhz Arduino.
#define DHTTYPE DHT22                   // DHT 22 version (AM2302), AM2321

#define CH4sens A1                      // CH4 sensor Vout. A1 pin
#define CH4ref A2                       // CH4 sensor Vref. A2 pin   
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

#define SD_CS_PIN 10                    // for the data logging shield, we use digital pin 10 for the SD cs line
File logfile;                           // the logging file

void error(char *str)                   // Halt if error
{
  Serial.print("error: ");
  Serial.println(str);
  digitalWrite(13, HIGH); // red LED indicates error
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
  pinMode(POWER_PIN, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT); //Start pump
  Serial.begin(9600);
  Serial.println();
  dht.begin(); //start RH_T_sensor
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
    Serial.println("millis,stampunix,datetime,RH%,tempC,CH4smV, CH4rmV, VbatmV, K33_RH, K33_Temp, K33_CO2, SampleNumber");
} else {
    Serial.println("datalog.csv doesn't exist.");
    Serial.println("Creating datalog.csv...");
    logfile = SD.open("datalog.csv", FILE_WRITE);
    logfile.println("millis,stampunix,datetime,RH%,tempC,CH4smV, CH4rmV, VbatmV, K33_RH, K33_Temp, K33_CO2, SampleNumber");
    logfile.close();
    Serial.println("millis,stampunix,datetime,RH%,tempC,CH4smV, CH4rmV, VbatmV, K33_RH, K33_Temp, K33_CO2, SampleNumber");
}
}

char time_to_read_CO2 = 1;
char n_delay_wait = 0;


void loop() {
  DateTime now;
  digitalWrite(POWER_PIN, HIGH);
  digitalWrite(PUMP_PIN, HIGH);
  SampleNumber++;
  
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

  // Reading temperature or humidity takes about 250 milliseconds.
  // Sensor readings may also be up to 2 seconds 'old' (its a slow sensor)
  dht.begin(); //start RH_T_sensor
  dht.readHumidity();
  dht.readTemperature();
  // Check if any reads failed and exit early (to try again).
  if (isnan(dht.humidity) || isnan(dht.temperature_C)) {
    error("DHT fejl");
    while(1);
    return;
  }
  CH4s = analogRead(CH4sens); //read CH4 Vout
  CH4smV = CH4s * (mV / steps); //convert pin reading to mV
  delay(10); //delay between reading of different analogue pins adviced.
  CH4r = analogRead(CH4ref); //read CH4 Vref
  CH4rmV = CH4r * (mV / steps); //convert pin reading to mV
  delay(10); //delay between reading of different analogue pins adviced.
  Vbat = analogRead(Vb); //read CH4 Vref
  VbatmV = Vbat * (mV / steps); //convert pin reading to mV, NOT YET correcting for the voltage divider.
  delay(10); //delay between reading of different analogue pins adviced.

  logfile = SD.open("datalog.csv", FILE_WRITE);
  logfile.print(", ");
  logfile.print(dht.humidity);
  logfile.print(", ");
  logfile.print(dht.temperature_C);
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
  logfile.println();
  logfile.close();
#if ECHO_TO_SERIAL
  Serial.print(", ");
  Serial.print(dht.humidity);
  Serial.print(", ");
  Serial.print(dht.temperature_C);
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

#endif //ECHO_TO_SERIAL

#if ECHO_TO_SERIAL
  Serial.println();
#endif // ECHO_TO_SERIAL

  if (time_to_read_CO2 == 0 ) {
    if (n_delay_wait < 10)
      n_delay_wait += 1;
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
