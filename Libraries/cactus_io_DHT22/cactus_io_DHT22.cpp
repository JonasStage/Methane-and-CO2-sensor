/* DHT library
 MIT license
 this library was based on the Adafruit Industries dhtxx library which supports SHT22
 
 modified by cactus.io
 this library only supports the SHT22. It does not support the SHT11/SHT21
 */

#include "cactus_io_DHT22.h"

DHT22::DHT22(uint8_t pin, uint8_t count) {
    _pin = pin;
    _count = count;
    firstreading = true;
}

void DHT22::begin(void) {
    // set up the pins!
    pinMode(_pin, INPUT);
    digitalWrite(_pin, HIGH);
    _lastreadtime = 0;
}

void DHT22::readHumidity(void) {
    if (read()) {
        
        humidity = data[0];
        humidity *= 256;
        humidity += data[1];
        humidity /= 10;
    }
    else
        humidity = NAN;
}

void DHT22::readTemperature() {
    
    if (read()) {
        temperature_C = data[2] & 0x7F;
        temperature_C *= 256;
        temperature_C += data[3];
        temperature_C /= 10;
        if (data[2] & 0x80)
            temperature_C *= -1;
        
        temperature_F = temperature_C * 1.8 + 32;
    }
    else
    {
        temperature_C = NAN;
        temperature_F = NAN;
    }
}

float DHT22::computeHeatIndex_C() {
    // Wikipedia: http://en.wikipedia.org/wiki/Heat_index
    return -8.784695 +
    1.61139411 * temperature_C +
    2.33854900 * humidity +
    -0.14611605 * temperature_C*humidity +
    -0.01230809 * pow(temperature_C, 2) +
    -0.01642482 * pow(humidity, 2) +
    0.00221173 * pow(temperature_C, 2) * humidity +
    0.00072546 * temperature_C * pow(humidity, 2) +
    -0.00000358 * pow(temperature_C, 2) * pow(humidity, 2);
}

float DHT22::computeHeatIndex_F() {
    // Adapted from equation at: https://github.com/adafruit/DHT-sensor-library/issues/9 and
    // Wikipedia: http://en.wikipedia.org/wiki/Heat_index
    return -42.379 +
    2.04901523 * temperature_F +
    10.14333127 * humidity +
    -0.22475541 * temperature_F*humidity +
    -0.00683783 * pow(temperature_F, 2) +
    -0.05481717 * pow(humidity, 2) +
    0.00122874 * pow(temperature_F, 2) * humidity +
    0.00085282 * temperature_F*pow(humidity, 2) +
    -0.00000199 * pow(temperature_F, 2) * pow(humidity, 2);
}

boolean DHT22::read(void) {
    uint8_t laststate = HIGH;
    uint8_t counter = 0;
    uint8_t j = 0, i;
    unsigned long currenttime;
    
    // Check if sensor was read less than two seconds ago and return early
    // to use last reading.
    currenttime = millis();
    if (currenttime < _lastreadtime) {
        // ie there was a rollover
        _lastreadtime = 0;
    }
    if (!firstreading && ((currenttime - _lastreadtime) < 2000)) {
        return true; // return last correct measurement
        //delay(2000 - (currenttime - _lastreadtime));
    }
    firstreading = false;
    _lastreadtime = millis();
    
    data[0] = data[1] = data[2] = data[3] = data[4] = 0;
    
    // pull the pin high and wait 250 milliseconds
    digitalWrite(_pin, HIGH);
    delay(250);
    
    // now pull it low for ~20 milliseconds
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);
    delay(20);
    // disable interrupts
    noInterrupts();
    digitalWrite(_pin, HIGH);
    delayMicroseconds(40);
    pinMode(_pin, INPUT);
    
    // read in timings
    for ( i=0; i< MAXTIMINGS; i++) {
        counter = 0;
        while (digitalRead(_pin) == laststate) {
            counter++;
            delayMicroseconds(1);
            if (counter == 255) {
                break;
            }
        }
        laststate = digitalRead(_pin);
        
        if (counter == 255) break;
        
        // ignore first 3 transitions
        if ((i >= 4) && (i%2 == 0)) {
            // shove each bit into the storage bytes
            data[j/8] <<= 1;
            if (counter > _count)
                data[j/8] |= 1;
            j++;
        }
        
    }
    // enable interrupts
    interrupts();
    
    // check we read 40 bits and that the checksum matches
    if ((j >= 40) && 
        (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) ) {
        return true;
    }
    
    
    return false;
    
}