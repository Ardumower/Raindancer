// wheelMotorSensor.h

#ifndef _MOTORSENSOR_h
#define _MOTORSENSOR_h

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


#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "Thread.h"
#include "helpers.h"
#include "hardware.h"
#include "batterySensor.h"
#include "errorhandler.h"
#include "RunningMedian.h"

extern TbatterieSensor batterieSensor;
extern TErrorHandler errorHandler;

// Liest Motorstrom aus und berechnet die Wattzahl
// Wenn diese zu hoch ist, wird der Mähmotor sofort ausgeschaltet.

class TmotorSensor : public Thread
{
private:
	AnalogIn& myAnalogIn;
	DigitalIn& myDiMotorFault;
	DigitalOut& myMotorEnable;
	char myName;
	uint8_t count;
	int motorMaxCurrent;
	int motorSenseCounter;

public:



	bool showValuesOnConsole;

	float sensorValue;
	float current;
	float watt;
	float offset;
	float scale;

	TmotorSensor(AnalogIn& _myAnalogIn, DigitalIn& _myDiMotorFault, DigitalOut& _myMotorEnable, char name) :myAnalogIn(_myAnalogIn), myDiMotorFault(_myDiMotorFault), myMotorEnable(_myMotorEnable), myName(name){
	}

	void setup() {
		count = 10;
		current = 0;
		showValuesOnConsole = false;
		//measureOffset();
		sensorValue = 0; // Converts and read the analog input value (value from 0.0 to 1.0)
		scale = 2.4f;  // found out by measurement 1.905f;  //0.525V/A  2.3f; //
		offset = 0;
		motorMaxCurrent = 3.0;
		motorSenseCounter = 0;;
	}


	virtual void run() {
		// Wird alle 77ms aufgerufen
		const float accel = 0.1f;
		float sensorCurrent;

		runned();

		sensorValue = myAnalogIn.getVoltage();

		if (sensorValue < 0.001f) {
			sensorValue = 0.0f;
			sensorCurrent = 0.0f;
		}
		else {
			sensorCurrent = (sensorValue - offset) * scale;
			if (sensorCurrent < 0.0f) sensorCurrent = 0;
		}

		current = (1.0f - accel) * current + accel * sensorCurrent;

		watt = batterieSensor.voltage * current;
		
		if (showValuesOnConsole && (++count > 10)) { //show message only every tenth call
			errorHandler.setInfo(F("!03,Motor%c  Watt: %f MotorCurrent: %f SensorValue: %f scale: %f\r\n"),myName, watt, current, sensorValue,scale);
			count = 0;
		}

		checkCurrent();
		checkMotorFault();
	}



	void checkMotorFault() {
		if (myDiMotorFault == LOW) {
			myMotorEnable = LOW;
			errorHandler.setError(F("Motor %c  fault\r\n"), myName);
		}
	}


	void checkCurrent() {
		// Wird alle 77ms aufgerufen
		if (current >= motorMaxCurrent) {
			motorSenseCounter++;
			if (motorSenseCounter >35) { // Überlauf verhindern
				motorSenseCounter = 35;
			}
		}
		else {
			motorSenseCounter = 0;
		}

		if (motorSenseCounter >= 25) { //ignore motorMowPower for 2 seconds
				errorHandler.setError(F("!03,Motor %c overcurrent: %f\r\n"), myName, current);
		}
	}



	void calculateScale(float measuredCurrent, float softwareCurrent) {
		scale = measuredCurrent * scale / softwareCurrent;
		errorHandler.setInfoNoLog(F("!03,Motor %c scale calculated: %f\r\n"), myName,scale);
	}

	void measureOffset() {
		errorHandler.setInfo(F("!03,Motor %c measure offset\r\n"), myName);
		errorHandler.setInfoNoLog(F("!03,"));
		offset = myAnalogIn.measureOffsetVoltage();
   	    errorHandler.setInfo(F("  offset: %f\r\n"), offset);
	}


	void showConfig()
	{
		errorHandler.setInfoNoLog(F("!03,motSensor Config Motor: %c\r\n"), myName);
		errorHandler.setInfoNoLog(F("!03,enabled: %lu\r\n"), enabled);
		errorHandler.setInfoNoLog(F("!03,interval: %lu\r\n"), interval);
		errorHandler.setInfoNoLog(F("!03,offset %d\r\n"), offset);
		errorHandler.setInfoNoLog(F("!03,scale %f\r\n"), scale);
	}
};

#endif


