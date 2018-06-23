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

#ifndef _HARDWARE_h
#define _HARDWARE_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "InOutInterface.h"
#include "BufferSerial.h"
#include "CRotaryEncoder.h"
#include "MC33926.h"
#include "Sabertooth.h"
#include "errorhandler.h"
#include "DueTimer.h"

//#include "SRF08.h"


extern BufferSerial pc;
extern BufferSerial bt;

extern BufferSerial *debug;
extern BufferSerial &perRX;
//extern BufferSerial &sabertoothTX;

//extern SRF08 rangeMod1;

extern AnalogIn aiBATVOLT;

extern AnalogIn aiCHARGEVOLTAGE;
extern AnalogIn aiCHARGECURRENT;

extern DigitalOut doChargeEnable;
extern DigitalOut doBatteryOffSwitch;


extern AnalogIn aiRandomIn;

extern DigitalOut doBuzzer;
extern DigitalOut doMyLED;

extern DigitalInOut dioDHT;
extern DigitalIn diPinRain;

extern DigitalIn diBumperL;
extern DigitalIn diBumperR;
extern AnalogIn aiBumper;



// left wheel motor
extern DigitalOut doMotorEnable;
extern PwmOut     pwmMotorLeft;
extern DigitalOut doMotorLeftDir;
extern AnalogIn   aiMotorLeftCurrent;
extern DigitalIn  diMotorLeftFault;

// right wheel motor
extern PwmOut     pwmMotorRight;
extern DigitalOut doMotorRightDir;
extern AnalogIn   aiMotorRightCurrent;
extern DigitalIn  diMotorRightFault;

// mower motor
extern DigitalOut doMotorMowEnable;
extern PwmOut     pwmMotorMowPWM;
extern DigitalOut doMotorMowDir;
extern AnalogIn   aiMotorMowCurrent;
extern DigitalIn  diMotorMowFault;


extern DigitalIn  diNearObsacleSensor;
extern DigitalIn  diBumperSensor;

// odometry
extern DigitalIn diEncLA;
//DigitalIn diEncLB(ENCODERLEFT_B_Pin, true);
extern DigitalIn diEncRA;
//DigitalIn diEncRB(ENCODERRIGTH_B_Pin, true);

//Perimeter
extern AnalogIn aiCoilLeft;
extern AnalogIn aiCoilRight;

extern CRotaryEncoder encoderL;
extern CRotaryEncoder encoderR;

extern MC33926Wheels motorDriver;
extern MC33926Mow mowMotorDriver;

//extern Sabertooth motordriver;
//extern Sabertooth mowMotorDriver;

extern i2cInOut i2cRTC;
extern i2cInOut i2cEEPROM;


extern void hardwareRun();
extern void hardwareSetup();


#define  MICROSECONDS_TO_CLOCK_CYCLES(x)  (microsecondsToClockCycles(x))


class InterruptLock {
public:
	InterruptLock() {
		noInterrupts();
	}

	void unlock() {
		interrupts();
	}

	~InterruptLock() {
		
	}

};

#endif


