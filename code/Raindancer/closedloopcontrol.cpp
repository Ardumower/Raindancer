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

#include <math.h>
#include "closedloopcontrol.h"
#include "helpers.h"
#include "hardware.h"
#include "errorhandler.h"
#include "config.h"



void TClosedLoopControlThread::setup(uint8_t motorNumber, CRotaryEncoder  *enc)    // Motor 1 oder 2
{

	flagHardstopActive = false;
	lastTimeEncoderRead = 0;

	setpointRPM = 0.0f;
	current_speedRPM = 0.0f;
	pdffOutputPWM = 0.0f; //

	flagShowSpeed = false;
	flagShowEncoder = false;
	flagShowSetpointCurrSpeed = false;

	flagControldirect = false;

	// Whixh motor is controlled 1=Left oder 2=Rigth
	motorNo = motorNumber;
	myEncoder = enc;

	enableDefaultRamping();
	firsttimeCurSpeed = true;

	current_speed_mph = 0;
	lastEncoderTickCounter = myEncoder->getTickCounter();
	flagShowEnableRamping = false;

	lastTimeSpeedSet = 0;
	lastSetSpeedPerc = 0;

	resetPDFF();
}


// used for position control
void TClosedLoopControlThread::enableDefaultRamping()
{
	kfr = 1;
	kp = 7;
	ki = 8;
	setOutputToZeroAtRPm = 4;
	stopReachedThresholdAtRpm = 0.5f;
	stopNearReachedThresholdAtRpm = 5.0f;
	resetPDFF();
	if (flagShowEnableRamping) {
		errorHandler.setInfoNoLog(F(" sp: %f cur: %f pwm: %f eRPM: %f iTerm: %f D\r\n"),  setpointRPM, current_speedRPM, pdffOutputPWM, errorRPM, ITermRPM);
	}

}


//used for perimeter tracking. Smooth ramping is done by changing kfr and kp
void TClosedLoopControlThread::enablePerTrackRamping()
{
	kfr = 0.45f;
	kp = 6;
	resetPDFF();
	if (flagShowEnableRamping) {
		errorHandler.setInfoNoLog(F("sp: %f cur: %f pwm: %f eRPM: %f iTerm: %f T\r\n"),  setpointRPM, current_speedRPM, pdffOutputPWM, errorRPM, ITermRPM);
	}
}

void TClosedLoopControlThread::enableFastStopRamping()
{
	kfr = 0.4f;
	kp = 4;
	resetPDFF();
	if (flagShowEnableRamping) {
		errorHandler.setInfoNoLog(F(" sp: %f cur: %f pwm: %f eRPM: %f iTerm: %f FS\r\n"),setpointRPM, current_speedRPM, pdffOutputPWM, errorRPM, ITermRPM);
	}
}

/*********************************************************/
// Motor FSM ausführen
/*********************************************************/
void TClosedLoopControlThread::run()
{
	// Wird alle 33ms aufgerufen
	runned();

	calculateCurSpeed();

	if (flagControldirect) {
		resetPDFF();
		return;
	}

	calculatePDFF();
}

void TClosedLoopControlThread::calculatePDFF() {
	unsigned long now = micros();
	unsigned long delta = now - lastTimePDFF;
	float Ta = (float)(delta) / 1000000.0f;  // Ta in seconds
	lastTimePDFF = now;

	errorRPM = setpointRPM - current_speedRPM;
	ITermRPM += (ki * errorRPM * Ta);

	float reqSetpointRPM = setpointRPM; //requested setpoint. don't change setpointRPM while bumpless transfer

	//Bumpless transfer and also at first time call Ta is not valid
	if (firsttimePDFF) {
		ITermRPM = current_speedRPM;
		reqSetpointRPM = current_speedRPM / kfr;
		firsttimePDFF = false;
	}

	pdffOutputPWM = kp * (kfr * reqSetpointRPM + ITermRPM - current_speedRPM);

	// Clamp Output and Iterm
	if (pdffOutputPWM > 255.0f) {
		pdffOutputPWM = 255.0f;
		ITermRPM = (pdffOutputPWM / kp) + current_speedRPM - (kfr * reqSetpointRPM);
	}
	else if (pdffOutputPWM < -255.0f) {
		pdffOutputPWM = -255.0f;
		ITermRPM = (pdffOutputPWM / kp) + current_speedRPM - (kfr * reqSetpointRPM);
	}

	// Speed is set to 0
	if (fabsf(reqSetpointRPM) < 0.01f) {
		if (fabsf(current_speedRPM) < setOutputToZeroAtRPm) {
			ITermRPM = 0.0f;
			pdffOutputPWM = 0.0f;
		}
	}

	if (flagHardstopActive) {
		pdffOutputPWM = 0.0f;
		resetPDFF();
	}

	motorDriver.motor(motorNo, pdffOutputPWM);

	if (flagShowSpeed) {
		errorHandler.setInfoNoLog(F("reqSp: %f cur: %f pwm: %f eRPM: %f iTerm: %f\r\n"), reqSetpointRPM, current_speedRPM, pdffOutputPWM, errorRPM, ITermRPM);
	}

	//used for pdff tuning
	if (flagShowSetpointCurrSpeed) {
		errorHandler.setInfoNoLog(F("%f,%f,%f\r\n"), setpointRPM, current_speedRPM, pdffOutputPWM);
	}
}

void TClosedLoopControlThread::resetPDFF() {
	firsttimePDFF = true;
}


float TClosedLoopControlThread::calculateSpeedPercToPwm(float  speedPercentage) {
	//float rpm = calculateSpeedPercToRpm(speedPercentage);
	//return  calculateSpeedRpmToPwm(rpm);
	return speedPercentage * 255.0f / 100.0f;
}


float TClosedLoopControlThread::calculateSpeedRpmToPwm(float  speedRPM) {
	return  (speedRPM * 255.0f) / CONF_MAX_WHEEL_RPM;
}

float TClosedLoopControlThread::calculateSpeedPercToRpm(int  speedPercentage) {
	//clamp to[-100, 100]
	speedPercentage = speedPercentage > 100 ? 100 : speedPercentage;
	speedPercentage = speedPercentage < -100 ? -100 : speedPercentage;

	return (CONF_MAX_WHEEL_RPM * (float)speedPercentage) / 100.0f;

}

float TClosedLoopControlThread::getCurrentSpeedInPerc() {

	return   current_speedRPM * 100.0f / CONF_MAX_WHEEL_RPM;
}

float TClosedLoopControlThread::getSetpointSpeedInPerc() {
	return   setpointRPM * 100.0f / CONF_MAX_WHEEL_RPM;
}


// ---------------------------------------------------------
// speed is -100% to +100%. This function calculates rpm for the given percentage.
// ---------------------------------------------------------
void TClosedLoopControlThread::setSpeed(int  speedPercentage)
{
	// Don't execute if error is active
	if (errorHandler.isErrorActive()) {
		return;
	}

	/*
	if (motorNo == 1) {
		errorHandler.setInfoNoLog(F("CLCsetSpeed %d%% ist: %f useRamp: %d \r\n"), speedPercentage, getCurrentSpeedInPerc(), useRamp);
	}
	*/

	if(flagControldirect){
		flagControldirect = false;
	}

	// call only if speed changes from last setting
	if (lastSetSpeedPerc != speedPercentage) {
		lastTimeSpeedSet = millis();
		lastSetSpeedPerc = speedPercentage;
		setpointRPM = calculateSpeedPercToRpm(speedPercentage);
		flagHardstopActive = false;
		//if (motorNo == 1) {
		//	errorHandler.setInfo(F("setSpeed %d ist: %f\r\n"), speedPercentage, getCurrentSpeedInPerc());
		//}
	}
	//errorHandler.setInfoNoLog(F("motorNo %d sollSpeedRPM =  %f \r\n"), motorNo, sollSpeedRPM);
}

void TClosedLoopControlThread::hardStop()
{
		motorDriver.motor(motorNo, 0);
		lastTimeSpeedSet = millis();
		lastSetSpeedPerc = 0;
		setpointRPM = 0;
		flagHardstopActive = true;
		//errorHandler.setInfoNoLog(F("!03,hardstop motor %i \r\n"), motorNo);
}

void TClosedLoopControlThread::stop()
{
	// call only if speed changes from last setting
	if (lastSetSpeedPerc != 0) {
		lastTimeSpeedSet = millis();
		lastSetSpeedPerc = 0;
		setpointRPM = 0;
		flagHardstopActive = false;
		//errorHandler.setInfoNoLog(F("!03,stop motor %i \r\n"), motorNo);	
	}
	
}


bool TClosedLoopControlThread::isStopped()
{
	return (fabsf(current_speedRPM) < stopReachedThresholdAtRpm);
}

bool TClosedLoopControlThread::isNearStopped()
{
	return (fabsf(current_speedRPM) < stopNearReachedThresholdAtRpm);
}

// send pwm direct to motot. speed: -255 to 255
void TClosedLoopControlThread::controlDirect(int speed)
{
	// Will only work if run is not called 
	// Therfore, before calling this function flagControldirect = true; must be set
	if (!flagControldirect) {
		flagControldirect = true;
	}

	// lastSetSpeedPerc will here also be used for PWM values
	if (lastSetSpeedPerc != speed) {
		lastTimeSpeedSet = millis();
		lastSetSpeedPerc = speed;
		pdffOutputPWM = speed;
		resetPDFF();
		//errorHandler.setInfoNoLog(F("cont direct setSpeed %d\r\n"),speedPercentage);
	}

	motorDriver.motor(motorNo, speed);
}


// Calculate current speed
void TClosedLoopControlThread::calculateCurSpeed()
{
	long encTickCounter, buff;
	unsigned long nowTime;

	// Delta Encoder ermitteln
	buff = myEncoder->getTickCounter();
	encTickCounter = buff - lastEncoderTickCounter;
	lastEncoderTickCounter = buff;

	// Delta Time ermitteln 
	nowTime = micros();
	unsigned long delta = nowTime - lastTimeEncoderRead;
	lastTimeEncoderRead = nowTime;
	//float deltaTimeSec = (float)delta / 1000000.0f;//time between most recent encoder ticks

	if (delta != 0) {
		//float speed = 60.0f * ((float)encTickCounter / CONF_ENCTICKSPERREVOLUTION) / deltaTimeSec;
		//float speed = 60.0f * (float)encTickCounter * 1000000.0f / (CONF_ENCTICKSPERREVOLUTION *(float)delta);
		float speed = 60000000.0f  * (float)encTickCounter / (CONF_ENCTICKSPERREVOLUTION *(float)delta);
		current_speedRPM = 0.6f * current_speedRPM + 0.4f * speed;
		//calculate current_speed in m per h. only used for flagShowEncoder/flagShowSpeed/printSensordata.cpp
		const float x = 60.0f * CONF_RADUMFANG_CM / 100.0f;
		current_speed_mph = current_speedRPM * x;
	}

	if (firsttimeCurSpeed) {
		// first time call, lastTickCounter and lastTimeEncoderRead are not initialized correctly. 
		// therefore set speed to 0 on startup;
		current_speedRPM = 0;
		firsttimeCurSpeed = false;
	}



	
	if (flagShowEncoder) {
		errorHandler.setInfoNoLog(F("!03,motor %i enc: %ld absEnc: %lu rpm: %f  m/h: %f deltaTicks: %ld deltaTime: %luus\r\n"), motorNo, myEncoder->getTickCounter(), myEncoder->getAbsTicksCounter(), current_speedRPM, current_speed_mph, encTickCounter, delta);
	}

	// Check motorstall or if Encoder runs in wrong direction
	//--------------------------------------------------------
	if (!CONF_DISABLE_MOTOR_STALL_CHECK ) {
		//errorHandler.setInfoNoLog(F("0 motor %i clc speed pwm: %f curSpeed %f\r\n"), motorNo, pdffOutputPWM, current_speedRPM);

		if (millis() - lastTimeSpeedSet > 1000ul) {
			if (pdffOutputPWM < -50 && current_speedRPM > 3) {  // just check if odometry sensors may be turning in the wrong direction
				errorHandler.setInfo(F("1 motor %i clc speed error pwm: %f curSpeed %f\r\n"), motorNo, pdffOutputPWM, current_speedRPM);
				motorStallCounter++;
			}

			else if (pdffOutputPWM > 50 && current_speedRPM < -3) {  // just check if odometry sensors may be turning in the wrong direction
				errorHandler.setInfo(F("2 motor %i clc speed error pwm: %f curSpeed %f\r\n"), motorNo, pdffOutputPWM, current_speedRPM);
				motorStallCounter++;
			}
			else if (abs(pdffOutputPWM) > 50 && abs(current_speedRPM) < 1) {  //Check if Motor stall
				errorHandler.setInfo(F("3 motor %i clc speed error pwm: %f curSpeed %f\r\n"), motorNo, pdffOutputPWM, current_speedRPM);
				motorStallCounter++;
			}
			else if (abs(pdffOutputPWM) <= 2 && abs(current_speedRPM) > 8) {  //Check if Motor will not stop
				errorHandler.setInfo(F("4 motor %i clc speed error pwm: %f curSpeed %f\r\n"), motorNo, pdffOutputPWM, current_speedRPM);
				motorStallCounter++;
			}
			else {
				motorStallCounter = 0;
			}

			if (motorStallCounter > 5) {
				errorHandler.setError(F("!03,motor %i clc speed error set: %f cur: %f pwm: %f\r\n"), motorNo, setpointRPM, current_speedRPM, pdffOutputPWM);
				pdffOutputPWM = 0;
				motorStallCounter = 0;
				motorDriver.motor(motorNo, pdffOutputPWM);
				hardStop();
			}
		}
	}

	if (flagShowSpeed) {
		errorHandler.setInfoNoLog(F("!03,motor %i rpm: %f  m/h: %f deltaTicks: %ld deltaTime: %lu stall: %d "), motorNo, current_speedRPM, current_speed_mph, encTickCounter, delta, motorStallCounter);
	}

}

void TClosedLoopControlThread::resetEncoderCounter() {
	myEncoder->resetTickCounter();
	myEncoder->resetAbsTicksCounter();
	lastEncoderTickCounter = myEncoder->getTickCounter();
}

void TClosedLoopControlThread::showConfig()
{
	errorHandler.setInfoNoLog(F("!03,CLC Config MotorNo: %i\r\n"), motorNo);
	errorHandler.setInfoNoLog(F("!03,enabled: %lu\r\n"), enabled);
	errorHandler.setInfoNoLog(F("!03,interval: %lu\r\n"), interval);
	errorHandler.setInfoNoLog(F("!03,KP: %f KI: %f \r\n"), kp, ki);
	errorHandler.setInfoNoLog(F("!03,setOutputZeroAtRPm %f\r\n"), setOutputToZeroAtRPm);
	errorHandler.setInfoNoLog(F("!03,stopReachedThresholdAtRpm %f\r\n"), stopReachedThresholdAtRpm);
	errorHandler.setInfoNoLog(F("!03,stopNearReachedThresholdAtRpm %f\r\n"), stopNearReachedThresholdAtRpm);
	
}

/*
// FFarray is the array of expected motor velocities (units: sensorValue per second) that correspond with the duty cycle in increments of 20
static double FFarray[11] = { -118, -93, -70, -49, -26, 0, 26, 49, 70, 93, 118 };

// FFdutycycle is the array of duty cycles in increments of 20. This array is constant.
static int FFdutycycle[11] = { -100,-80, -60,-40,-20, 0,20, 40, 60, 80,100 };

// The function getFF() simply converts velocityTarget from SensorValue/second to the estimated matching duty cycle using an array of pre-defined values and linear interpolation between those values.
// output = (Kp * velocityError) + (Kf * getFF(velocityTarget)) + (Ki * integral);
double TClosedLoopControlThread::getFF(double feedforward)
{
for (int i = 0; i < 10; i++)
{
// If feedforward is between two points on the characteristic curve,
if ((feedforward >= FFarray[i]) && (feedforward <= FFarray[i + 1]))
{
//  return (int)interpolate(FFarray[i], FFdutycycle[i], FFarray[i + 1], FFdutycycle[i + 1], feedforward);
}
}

// If feedforward is greater than the maximum value of the characteristic curve,
if (feedforward > FFarray[10])
return 100; // return the maximum duty cycle
// If feedforward is less than the minimum value of the characteristic curve,
else if (feedforward < FFarray[0])
return -100; // return the minimum duty cycle
// Else, something is wrong with the characteristic curve and the motor should be stopped
else
return 0;
}
*/















