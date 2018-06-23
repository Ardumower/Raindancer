/* DHT library

MIT license
written by Adafruit Industries

Adapted for Raindacer by Kai Wuertz
*/

#include "DHT.h"
#include "Thread.h"
#include "hardware.h"
#include "errorhandler.h"
#include "config.h"

#define MIN_INTERVAL 2000

TDHT::TDHT( uint8_t type) {
   _type = type;

  _maxcycles = MICROSECONDS_TO_CLOCK_CYCLES(1000);  // 1 millisecond timeout for
                                                 // reading pulses from DHT sensor.
  // Note that count is now ignored as the DHT reading algorithm adjusts itself
  // based on the speed of the processor.
}

void TDHT::setup(void) {
  // set up the pins!
  //pinMode(_pin, INPUT_PULLUP); -- normally bus is high:
	dioDHT.setPinMode(OUTPUT);
	dioDHT.write(HIGH);

	overTempCounter = 0;
  // Using this value makes sure that millis() - lastreadtime will be
  // >= MIN_INTERVAL right away. Note that this assignment wraps around,
  // but so will the subtraction.
  _lastreadtime = -MIN_INTERVAL;
  DEBUG_PRINT("FROM DEBUG DHT CLASS Max clock cycles: ");
  DEBUG_PRINTLN(_maxcycles, DEC);
  DEBUG_PRINT("FROM DEBUG DHT Type: ");
  DEBUG_PRINTLN(_type);
  DEBUG_PRINT("FROM DEBUG DHT on Pin: ");
  DEBUG_PRINTLN(_pin);

  dhtTempActual = 20.0f;
   
}


void TDHT::run() {

	// check if thread should run
	unsigned long time = millis();
	if (!shouldRun(time)) {
		return;
	}

	// called every 20013ms by TRunTempService - only if TRunTempService is executed!
	runned(time);

	if (CONF_DISABLE_DHT_SERVICE) {
		return;
	}
	
	// This is a blocking call. Returns first, if the data is read from the sensor. Needs round about 5ms and blocks interrupts
	dhtTempActual = readTemperature();

	// two times overtemp must be measured before CONF_OVERHEATING_TEMP is reached 
	if (dhtTempActual >= CONF_OVERHEATING_TEMP)
	{
		overTempCounter = (overTempCounter < OVERTEMPCOUNTLIMIT) ? overTempCounter + 1 : OVERTEMPCOUNTLIMIT;
	}
	else
	{
		overTempCounter = (overTempCounter > 0) ? overTempCounter - 1 : 0;
	}

	// shut down power if overtemp is reached in two measurements
	if (overTempCounter == OVERTEMPCOUNTLIMIT) {
		errorHandler.setError(F("!03,TDHT: CONF_OVERHEATING_TEMP reached: %f\r\n"), dhtTempActual);
		doChargeEnable = LOW;
		delay(20);
		doBatteryOffSwitch = LOW;
	}

}



float TDHT::getLastReadTemperature() {
	return dhtTempActual;
}

//boolean S == Scale.  True == Fahrenheit; False == Celcius
float TDHT::readTemperature(bool S, bool force) {
  float f = NAN;

  if (read(force)) {
    switch (_type) {
    case DHT11:
      f = data[2];
      if(S) {
        f = convertCtoF(f);
      }
      break;
    case DHT22:
    case DHT21:
      f = data[2] & 0x7F;
      f *= 256;
      f += data[3];
      f *= 0.1;
      if (data[2] & 0x80) {
        f *= -1;
      }
      if(S) {
        f = convertCtoF(f);
      }
      break;
    }
  }
  return f;
}

float TDHT::convertCtoF(float c) {
  return c * 1.8 + 32;
}

float TDHT::convertFtoC(float f) {
  return (f - 32) * 0.55555;
}

float TDHT::readHumidity(bool force) {
  float f = NAN;
  if (read()) {
    switch (_type) {
    case DHT11:
      f = data[0];
      break;
    case DHT22:
    case DHT21:
      f = data[0];
      f *= 256;
      f += data[1];
      f *= 0.1;
      break;
    }
  }
  return f;
}

//boolean isFahrenheit: True == Fahrenheit; False == Celcius
float TDHT::computeHeatIndex(float temperature, float percentHumidity, bool isFahrenheit) {
  // Using both Rothfusz and Steadman's equations
  // http://www.wpc.ncep.noaa.gov/html/heatindex_equation.shtml
  float hi;

  if (!isFahrenheit)
    temperature = convertCtoF(temperature);

  hi = 0.5 * (temperature + 61.0 + ((temperature - 68.0) * 1.2) + (percentHumidity * 0.094));

  if (hi > 79) {
    hi = -42.379 +
             2.04901523 * temperature +
            10.14333127 * percentHumidity +
            -0.22475541 * temperature*percentHumidity +
            -0.00683783 * pow(temperature, 2) +
            -0.05481717 * pow(percentHumidity, 2) +
             0.00122874 * pow(temperature, 2) * percentHumidity +
             0.00085282 * temperature*pow(percentHumidity, 2) +
            -0.00000199 * pow(temperature, 2) * pow(percentHumidity, 2);

    if((percentHumidity < 13) && (temperature >= 80.0) && (temperature <= 112.0))
      hi -= ((13.0 - percentHumidity) * 0.25) * sqrt((17.0 - abs(temperature - 95.0)) * 0.05882);

    else if((percentHumidity > 85.0) && (temperature >= 80.0) && (temperature <= 87.0))
      hi += ((percentHumidity - 85.0) * 0.1) * ((87.0 - temperature) * 0.2);
  }

  return isFahrenheit ? hi : convertFtoC(hi);
}

boolean TDHT::read(bool force) {
  // Check if sensor was read less than two seconds ago and return early
  // to use last reading.
  uint32_t currenttime = millis();
  if (!force && ((currenttime - _lastreadtime) < 2000)) {
    return _lastresult; // return last correct measurement
  }
  _lastreadtime = currenttime;

  // Reset 40 bits of received data to zero.
  data[0] = data[1] = data[2] = data[3] = data[4] = 0;

  // Send start signal.  See DHT datasheet for full signal diagram:
  //   http://www.adafruit.com/datasheets/Digital%20humidity%20and%20temperature%20sensor%20AM2302.pdf

  // Go into high impedence state to let pull-up raise data line level and
  // start the reading process.
  //dioDHT.write(HIGH);
  //delay(250);

  // First set data line low for 20 milliseconds.
  // Send start signal and  prepare sensor for reading
  //dioDHT.setPinMode(OUTPUT); // already set to output
  dioDHT.write(LOW);
  delay(20);

  uint32_t cycles[80];
  {
    // Turn off interrupts temporarily because the next sections are timing critical
    // and we don't want any interruptions.
    InterruptLock lock;

    // End the start signal by setting data line high for 40 microseconds.
	// Host pulls up 
	dioDHT.write(HIGH);
    delayMicroseconds(40);

	// and wait now for sensor's response.
    // Now start reading the data line to get the value from the DHT sensor.
	dioDHT.setPinMode(INPUT);
    delayMicroseconds(10);  // Delay a bit to let sensor pull data line low.

	// here now the sensor has already pulled down the line. If not, then soemthing got wrong.

    // First expect a low signal for ~80 microseconds followed by a high signal
    // for ~80 microseconds again.

	// Sensor pulled low already. Check for the low signal and wait until it gets high 
    if (expectPulse(LOW) == 0) {
	  lock.unlock(); // Turn on interrupts
      _lastresult = false;
	  errorHandler.setInfo(F("!03,TDHT Timeout waiting for start signal low pulse.\r\n"));
      return _lastresult;
    }
	// Sensor pulled high. Check for the high signal and wait until it gets low 
    if (expectPulse(HIGH) == 0) {
      lock.unlock(); // Turn on interrupts
      _lastresult = false;
	  errorHandler.setInfo(F("!03,TDHT Timeout waiting for start signal high pulse.\r\n"));
      return _lastresult;
    }

    // Now read the 40 bits sent by the sensor.  Each bit is sent as a 50
    // microsecond low pulse followed by a variable length high pulse.  If the
    // high pulse is ~28 microseconds then it's a 0 and if it's ~70 microseconds
    // then it's a 1.  We measure the cycle count of the initial 50us low pulse
    // and use that to compare to the cycle count of the high pulse to determine
    // if the bit is a 0 (high state cycle count < low state cycle count), or a
    // 1 (high state cycle count > low state cycle count). Note that for speed all
    // the pulses are read into a array and then examined in a later step.
    for (int i=0; i<80; i+=2) {
      cycles[i]   = expectPulse(LOW);  // Wait until start transmit signal ends. The input goes from low to high.
      cycles[i+1] = expectPulse(HIGH); // measure the high signal how long it stays high 
    }

	lock.unlock(); // Turn on interrupts
  } // Timing critical code is now complete.

  dioDHT.setPinMode(OUTPUT);
  dioDHT.write(HIGH);

  // Inspect pulses and determine which ones are 0 (high state cycle count < low
  // state cycle count), or 1 (high state cycle count > low state cycle count).
  for (int i=0; i<40; ++i) {
    uint32_t lowCycles  = cycles[2*i];
    uint32_t highCycles = cycles[2*i+1];
    if ((lowCycles == 0) || (highCycles == 0)) {
      errorHandler.setInfo(F("!03,TDHT Timeout waiting for pulse\r\n"));
      _lastresult = false;
      return _lastresult;
    }
    data[i/8] <<= 1;
    // Now compare the low and high cycle times to see if the bit is a 0 or 1. When AM2302 is sending data to MCU, every bit's transmission begin with low-voltage-level that last 50us
    if (highCycles > lowCycles) {
      // High cycles are greater than 50us low cycle count, must be a 1.
      data[i/8] |= 1;
    }
    // Else high cycles are less than (or equal to, a weird case) the 50us low
    // cycle count so this must be a zero.  Nothing needs to be changed in the
    // stored data.
  }

  DEBUG_PRINTLN(F("Received:"));
  DEBUG_PRINT(data[0], HEX); DEBUG_PRINT(F(", "));
  DEBUG_PRINT(data[1], HEX); DEBUG_PRINT(F(", "));
  DEBUG_PRINT(data[2], HEX); DEBUG_PRINT(F(", "));
  DEBUG_PRINT(data[3], HEX); DEBUG_PRINT(F(", "));
  DEBUG_PRINT(data[4], HEX); DEBUG_PRINT(F(" =? "));
  DEBUG_PRINTLN((data[0] + data[1] + data[2] + data[3]) & 0xFF, HEX);

  // Check we read 40 bits and that the checksum matches.
  if (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
    _lastresult = true;
    return _lastresult;
  }
  else {
	  errorHandler.setInfo(F("!03,TDHT Checksum failure!"));
    _lastresult = false;
    return _lastresult;
  }
}

// Expect the signal line to be at the specified level for a period of time and
// return a count of loop cycles spent at that level (this cycle count can be
// used to compare the relative time of two pulses).  If more than a millisecond
// ellapses without the level changing then the call fails with a 0 response.
// This is adapted from Arduino's pulseInLong function (which is only available
// in the very latest IDE versions):
//   https://github.com/arduino/Arduino/blob/master/hardware/arduino/avr/cores/arduino/wiring_pulse.c
uint32_t TDHT::expectPulse(bool level) {
  uint32_t count = 0;

    while (dioDHT.read() == level) {
      if (count++ >= _maxcycles) {
        return 0; // Exceeded timeout, fail.
      }
    }


  return count;
}
