/*
Robotic Lawn Mower
Copyright (c) 2017 by Kai Würtz



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


*/

/*

errorHandler.setInfo(F("A %d B%f\r\n"), 888, 55.66f);
errorHandler.setInfo(F("123456789"));
char txt[] = "0";
errorHandler.setInfo(txt);

int y = 9999;
float x = 44.56789223;
sprintf(errorHandler.msg, "\r\nthe current value is %d float: %f\r\n",y, x);
errorHandler.setInfo();

errorHandler.setError(F("ASDFG %f"),x);

debug.serial.println(F("\r\nBUFFER START ==================================:"));
errorHandler.print();
debug.serial.println(F("\r\nBUFFER END ==================================:"));


*/

#ifndef _ERRORHANDLER_h
#define _ERRORHANDLER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <string>
#include <stdio.h>
#include "helpers.h"
#include "RingBufferChar.h"
#include <stdarg.h>

using namespace std;


#define EH_MEASSAGE_SIZE 200
class TErrorHandler
{
private:
	String error;
	bool errorAvtive;
	RingBufferChar r;
public:

	char msg[EH_MEASSAGE_SIZE];

	TErrorHandler();

	void setInfo();
	void setInfoNoLog();
	void writeToLogOnly();
	//void setInfo(const char* i);
	//void setInfo(const __FlashStringHelper *ifshi);
	//void setInfo(char *fmt, ...);  //printf like
	//void setInfo(const char *fmt, ...);  //printf like
	void setInfo(const __FlashStringHelper *fmt, ...); //printf like
	void setInfoNoLog(const __FlashStringHelper *fmt, ...); //printf like

	//void setError(String e);
	//void setError(char *fmt, ...);  //printf like
	//void setError(const char *fmt, ...);  //printf like
	void setError();
	void setError(const __FlashStringHelper *fmt, ...); //printf like
	void resetError();
	bool isErrorActive();

	void print();
	void printError();
};


extern TErrorHandler errorHandler; // in main.cpp deklariert

#endif


