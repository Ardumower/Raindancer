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

#include "mowclosedloopcontrol.h"
#include "hardware.h"
#include "errorhandler.h"
#include "config.h"





void TMowClosedLoopControlThread::setup(uint8_t motorNumber)    // Motor 1 oder 2
{
	// Welcher motor wird am Sabertooth angesteuert 1 oder 2
	motorNo = motorNumber;
	SetState(STMM_STOP);
	motorDisabled = false;
	uiMotorDisabled = false;
	speedCurr = 0;
	motorMowAccel = 2000;
	speedLimit = 255; // 0 to 255
	flagShowSpeed = false;
	directionForward = true;
	resetCount = 0;
	//errorHandler.setInfo(F("!03,srvClcM SETUP \r\n"));
}


/*********************************************************/
// Motor FSM ausführen
/*********************************************************/
bool TMowClosedLoopControlThread::Run() {
	// Wird alle 200ms aufgerufen

	PT_BEGIN();
	while (1) {

		PT_YIELD_INTERVAL();
		LoopFSM();
	}
	PT_END();
}



void TMowClosedLoopControlThread::BeginState(EMowMotorState t) {
	switch (t) {
	case STMM_FORWARD:
		resetCount = 0;
		directionForward = true;
		break;
	case STMM_BACKWARD:
		resetCount = 0;
		directionForward = false;
		break;
	default:
		break;
	}
}
/*********************************************************/
// Motor Zustände Do State
/*********************************************************/
void TMowClosedLoopControlThread::UpdateState(EMowMotorState t) {


	switch (t) {
	case STMM_FORWARD:

		if (CONF_DISABLE_MOW_MOTOR) {
			mowMotorDriver.motor(motorNo, 0);
			speedCurr = 0;
		}
		else if (motorDisabled || uiMotorDisabled) {
			speedCurr -= (float)interval * 0.15f; //=29,7 bei 198 ms
			if (speedCurr < 50)
				speedCurr = 0;

			if (speedCurr < 0) {
				speedCurr = 0;
			}
			mowMotorDriver.motor(motorNo, speedCurr);
		}
		else {
			// Ramp up until speedLimit
			speedCurr += ((float)interval * (speedLimit - speedCurr)) / (float)motorMowAccel; // intertval comes from thread
			if (speedCurr < 95)
				speedCurr = 95;

			if (speedCurr > 250) {
				speedCurr = 255;
			}

			mowMotorDriver.motor(motorNo, speedCurr);
		}
		break;

	case STMM_BACKWARD:

		if (CONF_DISABLE_MOW_MOTOR) {
			mowMotorDriver.motor(motorNo, 0);
			speedCurr = 0;
		}
		else if (motorDisabled || uiMotorDisabled) {
			speedCurr -= (float)interval * 0.15f; //=29,7 bei 198 ms
			if (speedCurr < 50)
				speedCurr = 0;

			if (speedCurr < 0) {
				speedCurr = 0;
			}
			mowMotorDriver.motor(motorNo, -speedCurr);
		}
		else {
			// Ramp up until speedLimit
			speedCurr += ((float)interval * (speedLimit - speedCurr)) / (float)motorMowAccel; // intertval comes from thread
			if (speedCurr < 95)
				speedCurr = 95;

			if (speedCurr > 250) {
				speedCurr = 255;
			}

			mowMotorDriver.motor(motorNo, -speedCurr);
		}
		break;

	case STMM_STOP:

		// latch the situation, that it was not able to reset the motorfault
		if (resetCount == 111) {
			speedCurr = 0;
			mowMotorDriver.motor(motorNo, speedCurr);
			return;
		}

		//speedCurr -= (float)interval * 0.2f; //=40 bei 200 ms
		speedCurr -= (float)interval * 0.15f; //= reduce pwm by 29,7 at 198 ms interval
		if (speedCurr < 50)
			speedCurr = 0;

		if (speedCurr < 0) {
			speedCurr = 0;
		}
		if (directionForward == true) {
			mowMotorDriver.motor(motorNo, speedCurr);
		}
		else {
			mowMotorDriver.motor(motorNo, -speedCurr);
		}

		if (resetCount < 3) {
			if (diMotorMowFault == LOW) {
				mowMotorDriver.resetFault(true);
				resetCount++;
				errorHandler.setInfo(F("srvClcM Motor %c resetCount++;\r\n"), 'M');
			}
		}
		else {
			errorHandler.setError(F("srvClcM Motor %c  fault\r\n"), 'M');
			resetCount = 111;
		}

		//speedCurr = 0;
		break;

	default:
		//TODO invalid state - reset, perhaps?
		break;
	}

	if (flagShowSpeed) {
		errorHandler.setInfo(F("!03,srvClcM speed: %f \r\n"), speedCurr);
	}

};



void TMowClosedLoopControlThread::forward() {

	if (GetState() == STMM_BACKWARD) {
		errorHandler.setInfo(F("!03,srvClcM FORWARD not possible. STOP MOTOR FIRST!!!\r"));
		return;
	}
	SetState(STMM_FORWARD);
}


void TMowClosedLoopControlThread::backward() {

	if (GetState() == STMM_FORWARD) {
		errorHandler.setInfo(F("!03,srvClcM BACKWARD not possible. STOP MOTOR FIRST!!!\r"));
		return;
	}
	SetState(STMM_BACKWARD);
}


bool  TMowClosedLoopControlThread::isRunning() {

	return (GetState() == STMM_FORWARD || GetState() == STMM_BACKWARD);
}


void TMowClosedLoopControlThread::stop() {
	//SetState(STMM_STOP_REQUEST);
	SetState(STMM_STOP);
}


bool TMowClosedLoopControlThread::isStopped() {
	return (GetState() == STMM_STOP);
}

void TMowClosedLoopControlThread::controlDirect(int speed) {
	mowMotorDriver.motor(motorNo, speed);
}



void TMowClosedLoopControlThread::showConfig() {
	errorHandler.setInfo(F("!03,srvClcM Config MowMotorNo: %i\r\n"), motorNo);
	errorHandler.setInfo(F("!03,enabled: %d\r\n"), IsRunning());
	errorHandler.setInfo(F("!03,interval: %lu\r\n"), interval);
	errorHandler.setInfo(F("!03,motorMowAccel %d\r\n"), motorMowAccel);
	errorHandler.setInfo(F("!03,motorDisabled %d\r\n"), motorDisabled);
	errorHandler.setInfo(F("!03,uiMotorDisabled %d\r\n"), uiMotorDisabled);
	if (CONF_DISABLE_MOW_MOTOR) {
		errorHandler.setInfo(F("!03,mow motor disabled in config\r\n"));
	}
	else {
		errorHandler.setInfo(F("!03,mow motor enabled in config\r\n"));
	}
	errorHandler.setInfo(F("!03,speedLimit %f\r\n"), speedLimit);
}
