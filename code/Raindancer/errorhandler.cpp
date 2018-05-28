/*
Robotic Lawn Mower
Copyright (c) 2017 by Kai Würtz

Private-use only! (you need to ask for a commercial-use)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

Private-use only! (you need to ask for a commercial-use)
*/

#include <stdio.h>
#include <stdarg.h> 
#include "errorhandler.h"
#include "hardware.h"

const char errorTxt[] PROGMEM = { "ERROR " };
const char noErrorTxt[] PROGMEM = { "No Error\r\n" };


TErrorHandler::TErrorHandler()
{
	error = "";
	errorAvtive = false;
	msg[0] = '\0';
}


void TErrorHandler::setError(String e)
{
	if (errorAvtive)
		return;
	error = e;
	errorAvtive = true;
}



void TErrorHandler::setError()
{
	if (errorAvtive)
		return;
	error += (char*)msg;
	errorAvtive = true;
}

void TErrorHandler::resetError()
{
	error = "";
	errorAvtive = false;
}

void TErrorHandler::setInfo()
{
	//r.put('#'); // Erstmal einfuegen, damit ich feststellen kann, ob ich alle debug->printf Befehle erwisch habe bei dedr Umstellung
	r.putString(msg);
	debug->serial.print((char*)msg);
}

void TErrorHandler::writeToLogOnly()
{
	//r.put('#'); // Erstmal einfuegen, damit ich feststellen kann, ob ich alle debug->printf Befehle erwisch habe bei dedr Umstellung
	r.putString(msg);

}

void TErrorHandler::setInfoNoLog()
{
	debug->serial.print((char*)msg);
}

/*
void TErrorHandler::setInfo(const char* i) {
	//r.put('#'); // Erstmal einfuegen, damit ich feststellen kann, ob ich alle debug->printf Befehle erwisch habe bei dedr Umstellung
	r.putString(i);
	debug.serial.print((char*)i);
}

void TErrorHandler::setInfo(const __FlashStringHelper *ifshi) {
	//r.put('#'); // Erstmal einfuegen, damit ich feststellen kann, ob ich alle debug->printf Befehle erwisch habe bei dedr Umstellung
	r.putString(ifshi);
	debug.serial.print(ifshi);
}
*/

void TErrorHandler::print()
{
	r.print(); // print info log
	printError(); // print error

}

void TErrorHandler::printError()
{
	if (errorAvtive) {
		debug->serial.print(errorTxt);
		debug->serial.println(error.c_str());
	}
	else {
		debug->serial.println(noErrorTxt);
	}
}


bool TErrorHandler::isErrorActive()
{
	return errorAvtive;
}


/*
https://playground.arduino.cc/Main/Printf
p("%s", "Hello world");
p("%s\n", "Hello world"); // with line break
unsigned long a=0xFFFFFFFF;
p("Decimal a: %l\nDecimal unsigned a: %lu\n", a, a); 
p("Hex a: %x\n", a); 
p(F("Best to store long strings to flash to save %s"),"memory");
*/

/*
void TErrorHandler::setInfo(char *fmt, ...) {
	va_list args;
	va_start(args, fmt);
	vsnprintf(msg, EH_MEASSAGE_SIZE, fmt, args);
	va_end(args);
	setInfo();
}
*/

void TErrorHandler::setInfo(const char *fmt, ...) {
	va_list args;
	va_start(args, fmt);
	vsnprintf(msg, EH_MEASSAGE_SIZE, fmt, args);
	va_end(args);
	setInfo();
}

void TErrorHandler::setInfo(const __FlashStringHelper *fmt, ...) {
	va_list args;
	va_start(args, fmt);
#ifdef __AVR__
	vsnprintf_P(msg, EH_MEASSAGE_SIZE, (const char *)fmt, args); // progmem for AVR
#else
	vsnprintf(msg, EH_MEASSAGE_SIZE, (const char *)fmt, args); // for the rest of the world
#endif
	va_end(args);
	setInfo();
}

void TErrorHandler::setInfoNoLog(const __FlashStringHelper *fmt, ...) {
	va_list args;
	va_start(args, fmt);
#ifdef __AVR__
	vsnprintf_P(msg, EH_MEASSAGE_SIZE, (const char *)fmt, args); // progmem for AVR
#else
	vsnprintf(msg, EH_MEASSAGE_SIZE, (const char *)fmt, args); // for the rest of the world
#endif
	va_end(args);
	debug->serial.print((char*)msg);
}


void TErrorHandler::setError(const char *fmt, ...) {
	va_list args;
	va_start(args, fmt);
	vsnprintf(msg, EH_MEASSAGE_SIZE, fmt, args);
	va_end(args);
	setError();
}

/*void TErrorHandler::setError(char *fmt, ...) {
	va_list args;
	va_start(args, fmt);
	vsnprintf(msg, EH_MEASSAGE_SIZE, fmt, args);
	va_end(args);
	setError();
}
*/


void TErrorHandler::setError(const __FlashStringHelper *fmt, ...) {
	va_list args;
	va_start(args, fmt);
#ifdef __AVR__
	vsnprintf_P(msg, EH_MEASSAGE_SIZE, (const char *)fmt, args); // progmem for AVR
#else
	vsnprintf(msg, EH_MEASSAGE_SIZE, (const char *)fmt, args); // for the rest of the world
#endif
	va_end(args);
	setError();
}

