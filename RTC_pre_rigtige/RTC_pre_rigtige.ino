/// RTC starting code, has to be uploaded to the arduino for the RTC_PCF8523 to work properly

#include <SPI.h>
#include <Wire.h>
#include <RTClib.h>

RTC_PCF8523 RTC;                        // define the Real Time Clock object


void setup()
{
  Serial.begin(9600);
  RTC.begin(); 
  while (!Serial) {
    ;                              // wait for serial port to connect. Needed for native USB port only
  }
  
  // connect to RTC
  Wire.begin();
  if (!RTC.begin()) {
    Serial.println("RTC failed");
  }  
RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));

}

void loop() {
  DateTime now = RTC.now();

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
  Serial.println();
  delay(1000);
}
