/* mbed Microcontroller Library
* Copyright (c) 2006-2013 ARM Limited
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.

* Adapted to Arduion 2017 Kai Würtz
*
*/



#ifndef _INOUT_IF_h
#define _INOUT_IF_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>
#include "pinman.h"
#include "adcman.h"
#include "errorhandler.h"
#include "RunningMedian.h"


static const uint8_t I2C_TIMEOUT = 100; // Used to check for errors in I2C communication

inline void digitalWriteDirect(int pin, int val) {

	// Same as code below
	if (val) g_APinDescription[pin].pPort->PIO_SODR = g_APinDescription[pin].ulPin;
	else    g_APinDescription[pin].pPort->PIO_CODR = g_APinDescription[pin].ulPin;

	/*
	//First lets get the pin and bit mask - this can be done once at the start and then used later in the code (as long as the variables are in scope
	Pio* imThePort = g_APinDescription[pin].pPort;
	unsigned int imTheMask = g_APinDescription[pin].ulPin;

	if (val) {
	    //Lets set the pin high
	    imThePort->PIO_SODR = imTheMask;
	}
	else{
	    //And then low
	    imThePort->PIO_CODR = imTheMask;
	}
	*/
}


// works only for input ports
inline int digitalReadDirect(int pin) {

	// Same as code below
	return !!(g_APinDescription[pin].pPort->PIO_PDSR & g_APinDescription[pin].ulPin);

	/*
	Pio* imThePort = g_APinDescription[pin].pPort;
	unsigned int imTheMask = g_APinDescription[pin].ulPin & g_APinDescription[pin].ulPin;
	return !!(imThePort->PIO_PDSR & imTheMask);
	*/

}

//works only for output ports
inline int digitalReadDirectOutput(int pin) {

	//if (PIO_Get(g_APinDescription[pin].pPort, PIO_OUTPUT_0, g_APinDescription[pin].ulPin) == 1)
	if (PIO_Get(g_APinDescription[pin].pPort, ((g_pinStatus[pin] & 0xF0) >> 4 ? PIO_OUTPUT_1 : PIO_OUTPUT_0), g_APinDescription[pin].ulPin) == 1) {
		return HIGH;
	}
	else {
		return LOW;
	}


}



/** \addtogroup drivers */

/** A digital input, used for reading the state of a pin
*
* @note Synchronization level: Interrupt safe
*
* Example:
* @code
* // Flash an LED while a DigitalIn is true
*
* #include "mbed.h"
*
* DigitalIn enable(p5);
* DigitalOut led(LED1);
*
* int main() {
*     while(1) {
*         if(enable) {
*             led = !led;
*         }
*         wait(0.25);
*     }
* }
* @endcode
* @ingroup drivers
*/

class DigitalIn {

public:
	/** Create a DigitalIn connected to the specified pin
	*
	*  @param pin DigitalIn pin to connect to
	*/
	DigitalIn(const uint8_t pin, boolean pullup) : myPin(pin), m_pullup(pullup) {

	}

	void setup() {

		if (m_pullup) {
			pinMode(myPin, INPUT_PULLUP);
		}
		else {
			pinMode(myPin, INPUT);
		}
	}

	/** Read the input, represented as 0 or 1 (int)
	*
	*  @returns
	*    An integer representing the state of the input pin,
	*    0 for logical 0, 1 for logical 1
	*/
	int read() {
		return digitalRead(myPin);
	}


	int readDirect() {
		return digitalReadDirect(myPin);
	}


	/** An operator shorthand for read()
	* \sa DigitalIn::read()
	*/
	operator int() {
		return read();
	}

	uint8_t pin() {
		return myPin;
	}

protected:
	uint8_t myPin;
	boolean m_pullup;
};


/** A digital output, used for setting the state of a pin
*
* @note Synchronization level: Interrupt safe
*
* Example:
* @code
* // Toggle a LED
* #include "mbed.h"
*
* DigitalOut led(LED1);
*
* int main() {
*     while(1) {
*         led = !led;
*         wait(0.2);
*     }
* }
* @endcode
* @ingroup drivers
*/
class DigitalOut {

public:
	/** Create a DigitalOut connected to the specified pin
	*
	*  @param pin DigitalOut pin to connect to
	*/
	DigitalOut(const uint8_t pin) : myPin(pin) {
	}
	/** Create a DigitalOut connected to the specified pin
	*
	*  @param pin DigitalOut pin to connect to
	*  @param value the initial pin value
	*/

	/*
	DigitalOut(const uint8_t pin, int value) : myPin(pin) {

	}
	*/


	void setup(int value = LOW) {
		pinMode(myPin, OUTPUT);
		write(value);
	}

	/** Set the output, specified as 0 or 1 (int)
	*
	*  @param value An integer specifying the pin output value,
	*      0 for logical 0, 1 (or any other non-zero value) for logical 1
	*/
	void write(int value) {
		digitalWrite(myPin, value);
	}


	/** Return the output setting, represented as HIGH | LOW (int)
	*
	*  @returns
	*    an integer representing the output setting of the pin,
	*    0 for logical 0, 1 for logical 1
	*/
	int read() {
		return digitalRead(myPin);
	}


	void writeDirect(int value) {
		digitalWriteDirect(myPin, value);
	}


	int readDirect() {
		return digitalReadDirectOutput(myPin);
	}


	uint8_t pin() {
		return myPin;
	}

	/** A shorthand for write()
	* \sa DigitalOut::write()
	*/
	DigitalOut& operator= (int value) {
		write(value);
		return *this;
	}


	/** A shorthand for write()
	* \sa DigitalOut::write()
	*/
	DigitalOut& operator= (DigitalOut& rhs) {
		write(rhs.read());
		return *this;
	}

	/** A shorthand for read()
	* \sa DigitalOut::read()
	*/

	operator int() {
		// Underlying call is thread safe
		return read();
	}



protected:
	uint8_t myPin;
};





class DigitalInOut {

public:
	/** Create a DigitalIn connected to the specified pin
	*
	*  @param pin DigitalIn pin to connect to
	*/
	DigitalInOut(const uint8_t pin, uint32_t mode) : myPin(pin), myMode(mode) {

	}

	void setup() {
		setPinMode(myMode);
	}


	void setPinMode(uint32_t mode) {

		myMode = mode;

		if (myMode == INPUT) {
			pinMode(myPin, INPUT);
		}
		else if (myMode == INPUT_PULLUP) {
			pinMode(myPin, INPUT_PULLUP);

		}
		else {
			pinMode(myPin, OUTPUT);
		}
	}

	/** Read the input, represented as 0 or 1 (int)
	*
	*  @returns
	*    An integer representing the state of the input pin,
	*    0 for logical 0, 1 for logical 1
	*/
	int read() {
		return digitalRead(myPin);
	}

	int readDirect() {
		return digitalReadDirect(myPin);
	}

	void write(int value) {
		digitalWrite(myPin, value);
	}

	void writeDirect(int value) {
		digitalWriteDirect(myPin, value);
	}

	/** An operator shorthand for read()
	* \sa DigitalIn::read()
	*/
	operator int() {
		return read();
	}


	/** A shorthand for write()
	* \sa DigitalOut::write()
	*/
	DigitalInOut& operator= (int value) {
		write(value);
		return *this;
	}


	/** A shorthand for write()
	* \sa DigitalOut::write()
	*/
	DigitalInOut& operator= (DigitalInOut& rhs) {
		write(rhs.read());
		return *this;
	}

	uint8_t pin() {
		return myPin;
	}

protected:
	uint8_t myPin;
	uint32_t myMode;
};

/** \addtogroup drivers */

/** An analog input, used for reading the voltage on a pin
*
* @note Synchronization level: Thread safe
*
* Example:
* @code
* // Print messages when the AnalogIn is greater than 50%
*
* #include "mbed.h"
*
* AnalogIn temperature(p20);
*
* int main() {
*     while(1) {
*         if(temperature > 0.5) {
*             printf("Too hot! (%f)", temperature.read());
*         }
*     }
* }
* @endcode
* @ingroup drivers
*/
class AnalogIn {

public:

	/** Create an AnalogIn, connected to the specified pin
	*
	* @param pin AnalogIn pin to connect to
	*/
	AnalogIn(const uint8_t pin) : myPin(pin) {
	}

	void setup() {
		pinMode(myPin, INPUT);
		ADCMan.setupChannel(myPin, 1);
		value = 0;
	}

	void setup(int samplecount) {
		pinMode(myPin, INPUT);
		ADCMan.setupChannel(myPin, samplecount);
		value = 0;
	}
	/** Read the input voltage, represented as a float in the range [0.0, 1.0]
	*
	* @returns A floating-point value representing the current input voltage, measured as a percentage
	*/
	float read() {
		float ret = (float)read_int32() / 4095.0f; // 1023.0f;
		return ret;
	}

	/** Read the input voltage, represented as an unsigned short in the range [0x0, 0xFFFF]
	*
	* @returns
	*   16-bit unsigned short representing the current input voltage, normalised to a 16-bit value
	*/
	int32_t read_int32() {
		if (ADCMan.isConvComplete(myPin)) {
			value = ADCMan.getValue(myPin);
		}
		return value;
	}

	int32_t* getSamples() {
		return ADCMan.getSamples(myPin);
	}

	bool isConvComplete() {
		return ADCMan.isConvComplete(myPin);
	}

	void restartConv() {
		ADCMan.restartConv(myPin);
	}

	float getVoltage() {
		float adc;
		adc = read_int32();
		return (adc*3.3f) / 4095.0f;
	}

	/** An operator shorthand for read()
	*
	* The float() operator can be used as a shorthand for read() to simplify common code sequences
	*
	* Example:
	* @code
	* float x = volume.read();
	* float x = volume;
	*
	* if(volume.read() > 0.25) { ... }
	* if(volume > 0.25) { ... }
	* @endcode
	*/
	operator float() {
		return read();
	}

	virtual ~AnalogIn() {
		// Do nothing
	}

	uint8_t pin() {
		return myPin;
	}


	int32_t measureOffset() {
		float offset = 0;
		RunningMedian<float, 16> myMedian;

		for (unsigned int i = 0; i < myMedian.getSize(); i++) {
			int32_t m = ADCMan.measureOffset(myPin);
			myMedian.add((float)m);
			errorHandler.setInfoNoLog(F("%d "), m);
		}
		myMedian.getAverage(8, offset); // Get Median of 8 values in the middle of the sorted array. Spikes are on the outside of the sorted array.
		return (int32_t)offset;
	}

	float measureOffsetVoltage() {

		float offset = 0;

		offset = (float)measureOffset();
		return (offset * 3.3f / 4095.0f);

	}


protected:
	uint8_t myPin;
	int32_t value;
};



class PwmOut {

public:

	/** Create a PwmOut connected to the specified pin
	*
	*  @param pin PwmOut pin to connect to
	*/
	PwmOut(const uint8_t pin, PinManager& _PinMan) : m_myPin(pin), m_PinMan(_PinMan) {

	}

	void setup() {
		m_ulValue = 0;
		pinMode(m_myPin, OUTPUT);
	}
	/** Set the ouput duty-cycle, specified as  0 - 255,
	*
	*  @param value A uint32_t value representing the output duty-cycle 0 - 255,
	*/
	void write(uint32_t ulValue) {
		m_ulValue = ulValue;
		m_PinMan.analogWrite(m_myPin, ((byte)ulValue));
	}

	/** Return the current output duty-cycle setting, measured as a
	*
	*  @returns
	*    A uint32_t value representing the current duty-cycle being output on the pin,
	*    measured as  0 - 255
	*
	*/
	uint32_t read() {
		return m_ulValue;
	}


	/** A operator shorthand for write()
	*  \sa PwmOut::write()
	*/
	PwmOut& operator= (float value) {
		// Underlying call is thread safe
		write(value);
		return *this;
	}

	/** A operator shorthand for write()
	* \sa PwmOut::write()
	*/
	PwmOut& operator= (PwmOut& rhs) {
		// Underlying call is thread safe
		write(rhs.read());
		return *this;
	}

	/** An operator shorthand for read()
	* \sa PwmOut::read()
	*/
	operator uint32_t() {
		// Underlying call is thread safe
		return read();
	}

protected:
	uint8_t m_myPin;
	uint32_t m_ulValue;
	PinManager& m_PinMan;
};



class i2cInOut {
private:
	uint8_t highAddressByte(uint16_t address) {
		uint8_t BYTE_1;
		BYTE_1 = address >> 8;
		return BYTE_1;
	}

	uint8_t lowAddressByte(uint16_t address) {
		uint8_t BYTE_2;
		BYTE_2 = address & 0xFF;
		return BYTE_2;
	}

public:
	/** Create a i2c connected to the specified device
	*
	*  @param pin DigitalOut pin to connect to
	*/
	i2cInOut(const uint8_t _seven_bit_adress) : seven_bit_address(_seven_bit_adress) {
	}

	void setup() {

	}


	/**
	* I2C_ClearBus
	* (http://www.forward.com.au/pfod/ArduinoProgramming/I2C_ClearBus/index.html)
	* (c)2014 Forward Computing and Control Pty. Ltd.
	* NSW Australia, www.forward.com.au
	* This code may be freely used for both private and commerical use
	*
	* This routine turns off the I2C bus and clears it
	* on return SCA and SCL pins are tri-state inputs.
	* You need to call Wire.begin() after this to re-enable I2C
	* This routine does NOT use the Wire library at all.
	*
	* returns 0 if bus cleared
	*         1 if SCL held low.
	*         2 if SDA held low by slave clock stretch for > 2sec
	*         3 if SDA held low after 20 clocks.
	*/
	static int I2C_ClearBus() {

		int SDA = 20;
		int SCL = 21;
#if defined(TWCR) && defined(TWEN)
		TWCR &= ~(_BV(TWEN)); //Disable the Atmel 2-Wire interface so we can control the SDA and SCL pins directly
#endif

		pinMode(SDA, INPUT_PULLUP); // Make SDA (data) and SCL (clock) pins Inputs with pullup.
		pinMode(SCL, INPUT_PULLUP);

		delay(2500);  // Wait 2.5 secs. This is strictly only necessary on the first power
				  // up of the DS3231 module to allow it to initialize properly,
				  // but is also assists in reliable programming of FioV3 boards as it gives the
				  // IDE a chance to start uploaded the program
				  // before existing sketch confuses the IDE by sending Serial data.

		boolean SCL_LOW = (digitalRead(SCL) == LOW); // Check is SCL is Low.
		if (SCL_LOW) { //If it is held low Arduno cannot become the I2C master. 
			return 1; //I2C bus error. Could not clear SCL clock line held low
		}

		boolean SDA_LOW = (digitalRead(SDA) == LOW);  // vi. Check SDA input.
		int clockCount = 20; // > 2x9 clock

		while (SDA_LOW && (clockCount > 0)) { //  vii. If SDA is Low,
			clockCount--;
			// Note: I2C bus is open collector so do NOT drive SCL or SDA high.
			pinMode(SCL, INPUT); // release SCL pullup so that when made output it will be LOW
			pinMode(SCL, OUTPUT); // then clock SCL Low
			delayMicroseconds(10); //  for >5uS
			pinMode(SCL, INPUT); // release SCL LOW
			pinMode(SCL, INPUT_PULLUP); // turn on pullup resistors again
							    // do not force high as slave may be holding it low for clock stretching.
			delayMicroseconds(10); //  for >5uS
						     // The >5uS is so that even the slowest I2C devices are handled.
			SCL_LOW = (digitalRead(SCL) == LOW); // Check if SCL is Low.
			int counter = 20;
			while (SCL_LOW && (counter > 0)) {  //  loop waiting for SCL to become High only wait 2sec.
				counter--;
				delay(100);
				SCL_LOW = (digitalRead(SCL) == LOW);
			}
			if (SCL_LOW) { // still low after 2 sec error
				return 2; // I2C bus error. Could not clear. SCL clock line held low by slave clock stretch for >2sec
			}
			SDA_LOW = (digitalRead(SDA) == LOW); //   and check SDA input again and loop
		}
		if (SDA_LOW) { // still low
			return 3; // I2C bus error. Could not clear. SDA data line held low
		}

		// else pull SDA line low for Start or Repeated Start
		pinMode(SDA, INPUT); // remove pullup.
		pinMode(SDA, OUTPUT);  // and then make it LOW i.e. send an I2C Start or Repeated start control.
					     // When there is only one I2C master a Start or Repeat Start has the same function as a Stop and clears the bus.
					     /// A Repeat Start is a Start occurring after a Start with no intervening Stop.
		delayMicroseconds(10); // wait >5uS
		pinMode(SDA, INPUT); // remove output low
		pinMode(SDA, INPUT_PULLUP); // and make SDA high i.e. send I2C STOP control.
		delayMicroseconds(10); // x. wait >5uS
		pinMode(SDA, INPUT); // and reset pins as tri-state inputs which is the default state on reset
		pinMode(SCL, INPUT);
		return 0; // all ok
	}



	static void I2C_reset() {
		int rtn = I2C_ClearBus(); // clear the I2C bus first before calling Wire.begin()
		if (rtn == 0) {
			return;
		}

		if (rtn == 1) {
			errorHandler.setError(F("I2C SCL clock line held low"));
		}
		else if (rtn == 2) {
			errorHandler.setError(F("I2C SCL clock line held low by slave clock stretch"));
		}
		else if (rtn == 3) {
			errorHandler.setError(F("I2C SDA data line held low"));
		}
	}


	static void I2C_scan() {
		byte error, address;
		int nDevices;

		errorHandler.setInfoNoLog(F("I2C scanning...\r\n"));

		nDevices = 0;
		for (address = 1; address < 127; address++) {
			// The i2c_scanner uses the return value of
			// the Write.endTransmisstion to see if
			// a device did acknowledge to the address.
			Wire.beginTransmission(address);
			error = Wire.endTransmission();

			if (error == 0) {
				errorHandler.setInfoNoLog(F("I2C device found at address %d  0x%02x  "), address, address);
				nDevices++;
				switch (address) {
				case 0x1E: errorHandler.setInfoNoLog(F("probably HMC5883L\r\n")); break;
				case 0x33: errorHandler.setInfoNoLog(F("probably Advanced perimeter Receiver\r\n")); break;
				case 0x50: errorHandler.setInfoNoLog(F("probably AT24C32\r\n")); break;
				case 0x53: errorHandler.setInfoNoLog(F("probably ADXL345B\r\n")); break;
				case 0x60: errorHandler.setInfoNoLog(F("probably CMPS11\r\n")); break;
				case 0x68: errorHandler.setInfoNoLog(F("probably DS1307\r\n")); break;
				case 0x69: errorHandler.setInfoNoLog(F("probably MPU6050/9150 or L3G4200D\r\n")); break;
				case 0x77: errorHandler.setInfoNoLog(F("probably BMP180\r\n")); break;
				default: errorHandler.setInfoNoLog(F("unknown module\r\n"));
				}
			}
			else if (error == 4) {
				errorHandler.setInfoNoLog(F("Unknown error at address %d  0x%02x\r\n"), address, address);
			}
		}
		if (nDevices == 0) {
			errorHandler.setInfoNoLog(F("No I2C devices found\r\n"));
		}
		else {
			errorHandler.setInfoNoLog(F("I2C scan done\r\n"));
		}
	}


	bool ping(uint8_t _nAddrBytes = 1, bool showError = false) {
		Wire.beginTransmission(seven_bit_address);	// start transmission to device
		if (_nAddrBytes == 2) Wire.write(0);        // write high addr byte
		Wire.write(0);                              // write low addr byte
		byte retVal = Wire.endTransmission();       // Send the Tx buffer and stop
		if (retVal != 0) {
			if (showError) {
				errorHandler.setInfo(F("I2C ping failed seven_bit_address: %d error: %d\r\n"), seven_bit_address, retVal);
			}
			return false;
		}
		return true;
	}


	uint8_t write8(uint8_t reg, int len, uint8_t buff[]) {
		//if (ping()) {			//Is device reachable?
		Wire.beginTransmission(seven_bit_address); // Initialize the Tx buffer 
		Wire.write(reg);                           // Put slave register address in Tx buffer
		Wire.write(buff, len);                     // Put data in Tx buffer
		uint8_t rcode = Wire.endTransmission();    // Send the Tx buffer and stop
		if (rcode != 0) {
			errorHandler.setInfo(F("I2C write8 could not write to device: %d reg: %d code: %d\r\n"), seven_bit_address, rcode);
		}
		return rcode;
		//}
		//else {
		//	errorHandler.setInfo(F("I2C write8 ping error device: %d reg: %d\r\n"), seven_bit_address, reg);
		//}
	}

	/*
	//Write byte
	uint8_t write16(uint16_t reg, uint8_t val) {
	    //if (ping()) {			//Is device reachable?
	    Wire.beginTransmission(seven_bit_address); // Initialize the Tx buffer
	    Wire.write(highAddressByte(reg));          // First Word register address Address
	    Wire.write(lowAddressByte(reg));           // Second Word register address Address
	    Wire.write(val);						   // Put data in Tx buffer
	    uint8_t rcode = Wire.endTransmission();    // Send the Tx buffer and stop
	    if (rcode) {
		  errorHandler.setInfo(F("I2C write16 could not write to device: %d reg: %d code: %d\r\n"), seven_bit_address, rcode);
	    }
	    return rcode;
	    //}
	    //else {
	    //	errorHandler.setInfo(F("I2C write16 ping error device: %d reg: %d\r\n"), seven_bit_address, reg);
	    //}
	}*/


	//Write bytes
	uint8_t write16(uint16_t reg, int len, uint8_t buff[]) {
		//if (ping()) {			//Is device reachable?
		Wire.beginTransmission(seven_bit_address); // Initialize the Tx buffer 
		Wire.write(highAddressByte(reg));          // First Word register address Address
		Wire.write(lowAddressByte(reg));           // Second Word register address Address
		Wire.write(buff, len);                     // Put data in Tx buffer
		uint8_t rcode = Wire.endTransmission();    // Send the Tx buffer and stop
		if (rcode) {
			errorHandler.setInfo(F("I2C write16 could not write to device: %d reg: %d code: %d\r\n"), seven_bit_address, rcode);
		}
		return rcode;
		//}
		//else {
		//	errorHandler.setInfo(F("I2C write16 ping error device: %d reg: %d\r\n"), seven_bit_address, reg);
		//}
	}

	/*
	    void write16(uint16_t reg, int len, uint8_t buff[]) {
		  Wire.beginTransmission(seven_bit_address); //start transmission to device
		  if (Wire.endTransmission() == 0) {
			Wire.beginTransmission(seven_bit_address); //start transmission to device
			Wire.write(highAddressByte(reg));  //First Word Address
			Wire.write(lowAddressByte(reg));   //Second Word Address
			for (int i = 0; i < len; i++) {
			    Wire.write(buff[i]);        // send value to write
			}
			Wire.endTransmission(); // send a stop
		  }
	    }
	*/
	int read8Only(uint8_t len, uint8_t buff[], int retryCount) {
		int i = 0;
		//uint32_t I2C_Status;

		for (int j = 0; j < retryCount + 1; j++) {
			i = 0;

			//I2C_Status = TWI_GetStatus(WIRE1_INTERFACE);
			//errorHandler.setInfo(F("I2C_Status 1: %lu\r\n"), I2C_Status);

			Wire.requestFrom(seven_bit_address, len, (uint8_t)true); // read len bytes and send a stop bit. retVal has numer of received bytes
			while (Wire.available() && i < len)    //device may send less than requested (abnormal)
			{
				buff[i] = Wire.read(); // receive a byte
				i++;
				//errorHandler.setInfo(F("I2C read8Only read: %u\r\n"), (unsigned int)buff[0]);
			}
			//I2C_Status = TWI_GetStatus(WIRE1_INTERFACE);
			//errorHandler.setInfo(F("I2C_Status 2: %lu\r\n"), I2C_Status);

			if (len == i) {
				return i;
			}
			else {
				errorHandler.setInfo(F("I2C read8Only N retVal != len from device: %d len received: %i\r\n"), seven_bit_address, i);
			}

			if (j != retryCount) {
				delay(3);
				errorHandler.setInfo(F("I2C read8Only N j(%i) != retryCount(%i) device: %d\r\n"), seven_bit_address,i, retryCount);

			}
			errorHandler.setInfo(F("I2C read8Only N try to reading device again: %d\r\n"), seven_bit_address);
		}

		errorHandler.setInfo(F("I2C read8Only N could not read from device: %d\r\n"), seven_bit_address);
		return i;
	}

	// 8Bit register address. Read sequence of n bytes
	int read8(uint8_t reg, uint8_t len, uint8_t buff[], int retryCount) {
		uint8_t rcode;

		int i = 0;
		for (int j = 0; j < retryCount + 1; j++) {
			i = 0;

			Wire.beginTransmission(seven_bit_address);   // Initialize the Tx buffer
			Wire.write(reg);                             // Put slave register address in Tx buffer
			rcode = Wire.endTransmission();             // Send the Tx buffer and End transmission with stop.

			if (rcode == 0) {

				Wire.requestFrom(seven_bit_address, len, (uint8_t)true); // read len bytes and send a stop bit. retVal has numer of received bytes
				while (Wire.available() && i < len)    //device may send less than requested (abnormal)
				{
					buff[i] = Wire.read(); // receive a byte
					i++;
				}


			}//if (rcode == 0) {
			else {
				errorHandler.setInfo(F("I2C read8 N rcode from device: %d rcode: %d\r\n"), seven_bit_address, rcode);
			}

			if (len == i) {
				return i;
			}
			else {
				errorHandler.setInfo(F("I2C read8 N retVal != len from device: %d register: %d\r\n"), seven_bit_address, reg);
			}

			if (j != retryCount) {
				delay(3);
				errorHandler.setInfo(F("I2C read8 N j != retryCount device: %d register: %d\r\n"), seven_bit_address, reg);

			}
			errorHandler.setInfo(F("I2C read8 N try to reading device again: %d register: %d\r\n"), seven_bit_address, reg);
		}

		errorHandler.setInfo(F("I2C read8 N could not read from device: %d reg: %d\r\n"), seven_bit_address, reg);
		return i;
	}


	// 16Bit register address. Read sequence of n bytes
	int read16(uint16_t reg, uint8_t len, uint8_t buff[], int retryCount) {
		uint8_t rcode;

		int i = 0;
		for (int j = 0; j < retryCount + 1; j++) {
			i = 0;

			Wire.beginTransmission(seven_bit_address);   // Initialize the Tx buffer
			Wire.write(highAddressByte(reg));  //First Word Address
			Wire.write(lowAddressByte(reg));   //Second Word Address
			rcode = Wire.endTransmission();   // Send the Tx buffer and End transmission with stop.

			if (rcode == 0) {

				Wire.requestFrom(seven_bit_address, len, (uint8_t)true); // read len bytes and send a stop bit. retVal has numer of received bytes
				while (Wire.available() && i < len)    //device may send less than requested (abnormal)
				{
					buff[i] = Wire.read(); // receive a byte
					i++;
				}


			}//if (rcode == 0) {
			else {
				errorHandler.setInfo(F("I2C read16 N rcode from device: %d rcode: %d\r\n"), seven_bit_address, rcode);
			}

			if (len == i) {
				return i;
			}
			else {
				errorHandler.setInfo(F("I2C read16 N retVal != len from device: %d reg: %d\r\n"), seven_bit_address, reg);
			}

			if (j != retryCount) {
				delay(3);
				errorHandler.setInfo(F("I2C read16 N j != retryCount device: %d reg: %d\r\n"), seven_bit_address, reg);

			}
			errorHandler.setInfo(F("I2C read16 N retry reading device: %d reg: %d\r\n"), seven_bit_address, reg);
		}

		errorHandler.setInfo(F("I2C read16 N could not read from device: %d reg: %d\r\n"), seven_bit_address, reg);
		return i;
	}

	/*
	    int read16(uint16_t reg, uint8_t& val) {
		  int i = 0;

		  Wire.beginTransmission(seven_bit_address); //start transmission to device
		  Wire.write(highAddressByte(reg));  //First Word Address
		  Wire.write(lowAddressByte(reg));   //Second Word Address
		  if (Wire.endTransmission() == 0) {
			Wire.requestFrom(seven_bit_address, (uint8_t)1, (uint8_t)true); // read len bytes and send a stop at the end ot receiving

			while (Wire.available() > 0 && i < 1)    //device may send less than requested (abnormal)
			{
			    val = Wire.read(); // receive a byte
			    i++;
			}
		  }

		  if (i == 0) {
			errorHandler.setInfo(F("I2C read16 could not read from device: %d reg: %d\r\n"), seven_bit_address, reg);
		  }
		  return i;
	    }
	    */
	    /*
	    int read16(uint16_t reg, uint8_t len, uint8_t buff[], int retryCount) {
		  int i = 0;
		  for (int j = 0; j < retryCount + 1; j++) {
			i = 0;
			Wire.beginTransmission(seven_bit_address); //start transmission to device
			if (Wire.endTransmission() == 0) {         //Is device reachable?
			    Wire.beginTransmission(seven_bit_address); //start transmission to device
			    Wire.write(highAddressByte(reg));  //First Word Address
			    Wire.write(lowAddressByte(reg));   //Second Word Address
			    if (Wire.endTransmission() == 0) { //end transmission with stop. Request only if without error
				  Wire.requestFrom(seven_bit_address, len, (uint8_t)true); // read len bytes and send a stop bit at the end ot receiving
				  while (Wire.available() > 0 && i < len)    //device may send less than requested (abnormal)
				  {
					buff[i] = Wire.read(); // receive a byte
					i++;
				  }
			    }
			}
			if (len == i) return i;
			if (j != retryCount) delay(3);
		  }

		  errorHandler.setInfo(F("I2C read16 N could not read from device: %d reg: %d\r\n"), seven_bit_address, reg);
		  return i;

	    }

	*/

	void set_seven_bit_address(uint8_t _seven_bit_address) {
		seven_bit_address = _seven_bit_address;
	}

	uint8_t read_seven_bit_address() {
		return seven_bit_address;
	}

protected:
	uint8_t seven_bit_address;
};




//
//class AnalogIn {
//
//public:
//
///** Create an AnalogIn, connected to the specified pin
//*
//* @param pin AnalogIn pin to connect to
//*/
//AnalogIn(const uint8_t pin) : myPin(pin) {
//}
//
//void setup() {
//	pinMode(myPin, INPUT);
//}
///** Read the input voltage, represented as a float in the range [0.0, 1.0]
//*
//* @returns A floating-point value representing the current input voltage, measured as a percentage
//*/
//float read() {
//	float ret = (float)read_u16() / 4095.0f; // 1023.0f;
//	return ret;
//}
//
///** Read the input voltage, represented as an unsigned short in the range [0x0, 0xFFFF]
//*
//* @returns
//*   16-bit unsigned short representing the current input voltage, normalised to a 16-bit value
//*/
//unsigned short read_u16() {
//	unsigned short ret = analogRead(myPin);
//	return ret;
//}
//
///** An operator shorthand for read()
//*
//* The float() operator can be used as a shorthand for read() to simplify common code sequences
//*
//* Example:
//* @code
//* float x = volume.read();
//* float x = volume;
//*
//* if(volume.read() > 0.25) { ... }
//* if(volume > 0.25) { ... }
//* @endcode
//*/
//operator float() {
//	return read();
//}
//
//virtual ~AnalogIn() {
//	// Do nothing
//}
//
//uint8_t pin() {
//	return myPin;
//}
//
//
//protected:
//	uint8_t myPin;
//};
//



#endif


