/*
Robotic Lawn Mower
Copyright (c) 2017 by Kai WÃ¼rtz



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

#ifndef MOWMOROTRSENSOR_H
#define MOWMOROTRSENSOR_H

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
extern TMowClosedLoopControlThread clcM;

// Liest MowMotorstrom aus und berechnet die Wattzahl
// Wenn diese zu hoch ist, wird der MÃ¤hmotor sofort ausgeschaltet.

class TMowMotorSensor : public Thread
{
private:
	DigitalIn& myDiMotorFault;
	DigitalOut& myMotorEnable;
	int motorMowPowerMax;
	int motorMowSenseCounter;
	unsigned long lastTimeMotorMowStucked;
	//unsigned long timeUnderHeavyLoad;
	uint8_t count;

public:

	bool showValuesOnConsole;
	bool motorUnderHeavyLoad;

	float sensorValue;
	float current;
	float watt;
	float offset;
	float scale;

	TMowMotorSensor(DigitalIn& _myDiMotorFault, DigitalOut& _myMotorEnable) : myDiMotorFault(_myDiMotorFault), myMotorEnable(_myMotorEnable) {
	}

	void setup() {
		count = 10;
		current = 0;
		motorMowSenseCounter = 0;
		motorMowPowerMax = 75;
		scale = 2.4f; // found out by tests 1.905f;  //0.525V/A
		offset = 0;
		lastTimeMotorMowStucked = 0;
		showValuesOnConsole = false;
		motorUnderHeavyLoad = false;
		//timeUnderHeavyLoad = 0;
	   //measureOffset(); 
	   //sensorValue = aiMotorMowCurrent.read() - offset; // Converts and read the analog input value (value from 0.0 to 1.0)
		sensorValue = 0;

	}


	virtual void run() {
		// Wird alle 97ms aufgerufen
		//int adc;
		float sensorCurrent;
		const float accel = 0.1f;

		runned();
		//adc = aiMotorMowCurrent.read_int32();
		sensorValue = aiMotorMowCurrent.getVoltage();

		if (sensorValue < 0.001f) {
			sensorValue = 0.0f;
			sensorCurrent = 0.0f;
		}
		else {
			sensorCurrent = (sensorValue - offset) * scale;
			if (sensorCurrent < 0.0f) sensorCurrent = 0.0f;
		}

		current = (1.0f - accel) * current + accel * sensorCurrent;

		watt = batterieSensor.voltage * current;

		checkMowCurrent();
		checkIfUnderHeavyLoad();
		checkMotorFault();

		if (showValuesOnConsole && (++count > 10)) {
			errorHandler.setInfo(F("!03MotorM ,Watt: %f MotorCurrent: %f sensorValue: %f scale %f, motorDisabled %d\r\n"), watt, current, sensorValue, scale, clcM.motorDisabled);
			//errorHandler.setInfo(F("!03MotorM ,Watt: %f MotorCurrent: %f sensorValue: %f motorDisabled %d ADC %d\r\n"),watt, current, sensorValue, clcM.motorDisabled,adc);
			count = 0;
		}

	}


	void checkMotorFault() {

		if (clcM.GetState() != STMM_STOP) { // While motor ist stopping it coud be, that there is a motorfault. mowclosedloopcontrol.cpp handle this in statw STMM_STOP
			if (myDiMotorFault == LOW) {
				myMotorEnable = LOW;
				errorHandler.setError(F("Motor %c  fault\r\n"), 'M');
			}
		}
	}

	void checkMowCurrent() {
		// Wird alle 97ms aufgerufen
		if (watt >= motorMowPowerMax) {
			motorMowSenseCounter++;
			if (motorMowSenseCounter > 35) { // Überlauf verhindern
				motorMowSenseCounter = 35;
			}
		}
		else {
			motorMowSenseCounter = 0;
			if (millis() >= lastTimeMotorMowStucked + 15000) { // wait 30 seconds before switching on again
				clcM.motorDisabled = false;
			}
		}

		if (motorMowSenseCounter >= 30) { //ignore motorMowPower for 3 seconds
			if (clcM.motorDisabled == false) { //Show message only once
				errorHandler.setInfo(F("!03,Mow MotorDisabled: current high\r\n"));
			}
			clcM.motorDisabled = true;
			lastTimeMotorMowStucked = millis();
		}
	}


	void checkIfUnderHeavyLoad() {
		// Mit Hysterese schalten
		if (watt >= CONF_MOW_MOT_UNDER_HEAVY_LOAD_ON) {
			if (motorUnderHeavyLoad == false) {
				motorUnderHeavyLoad = true;
				//timeUnderHeavyLoad = millis();
				//errorHandler.setInfo ("!03,Mowmotor under heavy load");
			}
		}
		else {
			if (motorUnderHeavyLoad == true) { // && (millis()-timeUnderHeavyLoad) > 2000 ) {
				if (watt < CONF_MOW_MOT_UNDER_HEAVY_LOAD_OFF) {
					motorUnderHeavyLoad = false;
				}
			}
		}
	}

	bool checkIfUnderLoad() {
		if (watt >= CONF_MOW_MOT_UNDER_LOAD) {
			return true;
		}
		else {
			return false;
		}
	}

	void calculateScale(float measuredCurrent, float softwareCurrent) {
		scale = measuredCurrent * scale / softwareCurrent;
		errorHandler.setInfoNoLog(F("!03,MowMotor scale calculated: %f\r\n"), scale);
	}

	void measureOffset() {

		errorHandler.setInfo(F("!03,Mow Motor measure offset\r\n"));
		errorHandler.setInfoNoLog(F("!03,"));
		offset = aiMotorMowCurrent.measureOffsetVoltage();
		errorHandler.setInfo(F("  offset: %f\r\n"), offset);
	}

	void showConfig()
	{
		errorHandler.setInfoNoLog(F("!03,motSensor Config Motor M:\r\n"));
		errorHandler.setInfoNoLog(F("!03,enabled: %lu\r\n"), enabled);
		errorHandler.setInfoNoLog(F("!03,interval: %lu\r\n"), interval);
		errorHandler.setInfoNoLog(F("!03,offset %d\r\n"), offset);
		errorHandler.setInfoNoLog(F("!03,scale %f\r\n"), scale);
	}

};

#endif

