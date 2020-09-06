// BufferSerial.h
/* mbed BufferSerial Library
* Copyright (c) 2013 KentaShimizu, MIT License
* Version 0.1 (May 18, 2013)
*
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
* and associated documentation files (the "Software"), to deal in the Software without restriction,
* including without limitation the rights to use, copy, modify, merge, publish, distribute,
* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all copies or
* substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
* BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
* DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*
* Adapted to Arduino and own needs 2017 Kai Würtz
*/

#ifndef _BUFFERSERIAL_h
#define _BUFFERSERIAL_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

//#define ARDBUFFER 16
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>  




class BufferSerial
{
private:
	bool isUSB;
	UARTClass &serial;
	Serial_        &usbserial;
protected:
	/*
	int _size;
	int _present;
	volatile int _last;
	volatile char* _buf;
	void _setup(const int& size);
	int _getShift(volatile const int& value);
	*/


public:
	BufferSerial(UARTClass& s, const int& bufferSize);
	BufferSerial(Serial_& s, const int& bufferSize);

	int available();
	int availableForWrite();
	void begin(unsigned long);
	void flush();
	char getChar(void);

	size_t print(const __FlashStringHelper *);
	size_t print(const String &);
	size_t print(const char[]);
	size_t print(char);
	size_t print(unsigned char, int = DEC);
	size_t print(int, int = DEC);
	size_t print(unsigned int, int = DEC);
	size_t print(long, int = DEC);
	size_t print(unsigned long, int = DEC);
	size_t print(double, int = 2);
	size_t print(const Printable&);

	size_t println(const __FlashStringHelper *);
	size_t println(const String &s);
	size_t println(const char[]);
	size_t println(char);
	size_t println(unsigned char, int = DEC);
	size_t println(int, int = DEC);
	size_t println(unsigned int, int = DEC);
	size_t println(long, int = DEC);
	size_t println(unsigned long, int = DEC);
	size_t println(double, int = 2);
	size_t println(const Printable&);
	size_t println(void);
    size_t write(const uint8_t x);

	virtual ~BufferSerial();
	//	void run(void);


};


#endif


