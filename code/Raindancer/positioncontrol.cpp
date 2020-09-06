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

Code used from LinuxCNC Project and adapted to own needs
https://github.com/LinuxCNC/linuxcnc/blob/master/src/emc/motion/simple_tp.h
https://github.com/LinuxCNC/linuxcnc/blob/master/src/emc/motion/simple_tp.c
*******************************************************************
* Description: simple_tp.c
*   A simple single axis trajectory planner.  See simple_tp.h for API.
*
* Author: jmkasunich
* License: GPL Version 2
* Created on:
* System: Linux
*3
* Copyright (c) 2004 All rights reserved.

The update() function does all the work.  If 'enable' is true, it
computes a new value of 'curr_pos', which moves toward 'pos_cmd'
while obeying the 'max_vel' and 'max_accel' limits.  It also sets
'active' if movement is in progress, and clears it when motion
stops at the commanded position.  The command or either of the
limits can be changed at any time.  If 'enable' is false, it
ramps the velocity to zero, then clears 'active' and sets
'pos_cmd' to match 'curr_pos', to avoid motion the next time it
is enabled.  'period' is the period between calls, in seconds.

*******************************************************************
*/

#include <math.h>
#include "positioncontrol.h"
#include "hardware.h"
#include "errorhandler.h"
#include "config.h"

#define TINY_DP(max_acc,period) (max_acc*period*period*0.1)

/**********************************************************************************/
/**********************************************************************************/
// Position control
/**********************************************************************************/
/**********************************************************************************/

void TPositionControl::setup(TClosedLoopControlThread* _motor, CRotaryEncoder* enc) {
	flagShowResults = false;

	ta = 0.1f; //only for initatisation. Will be set in run to the right value
	stopCmBeforeTarget = 0.5f; // Must be positive and >0 
	max_acc = 70.0f; //maximal 70% speed change per 1 sec
	motor = _motor;
	myEncoder = enc;
	state = 1;
}


bool TPositionControl::Run() {
	float max_dv, tiny_dp, pos_err, vel_req;

	PT_BEGIN();
	while (1) {

		PT_YIELD_INTERVAL();

		ta = static_cast<float>(interval) / 1000.0f;

		if (state == 0) {
			active = 0;
			/* compute max change in velocity per servo period */
			max_dv = max_acc * ta;
			/* compute a tiny position range, to be treated as zero */
			tiny_dp = stopCmBeforeTarget; // TINY_DP(max_acc, ta);
			curr_pos = getCMForCounts(myEncoder->getTickCounter());


			if (enable) {
				/* planner enabled, request a velocity that tends to drive
				pos_err to zero, but allows for stopping without position
				overshoot */
				pos_err = pos_cmd - curr_pos;
				/* positive and negative errors require some sign flipping to avoid sqrt(negative)
				forward and backward I added to prevent driving back in case of overshoot of tiny_dp*/
				if ((pos_err > tiny_dp) && forward) {
					vel_req = -max_dv +
						sqrt(2.0f * max_acc * pos_err + max_dv * max_dv);
					/* mark planner as active */
					active = 1;
				}
				else if ((pos_err < -tiny_dp) && backward) {
					vel_req = max_dv -
						sqrt(-2.0f * max_acc * pos_err + max_dv * max_dv);
					/* mark planner as active */
					active = 1;
				}
				else {
					/* within 'tiny_dp' of desired pos, no need to move */
					vel_req = 0.0f;
				}
			}
			else {
				/* planner disabled, request zero velocity */
				vel_req = 0.0f;
				/* and set command to present position to avoid movement when next enabled */
				pos_cmd = curr_pos;
			}

			/* limit velocity request */
			if (vel_req > max_vel) {
				vel_req = max_vel;
			}
			else if (vel_req < -max_vel) {
				vel_req = -max_vel;
			}


			/* ramp velocity toward request at accel limit */
			if (vel_req > curr_vel + max_dv) {
				curr_vel += max_dv;
			}
			else if (vel_req < curr_vel - max_dv) {
				curr_vel -= max_dv;
			}
			else {
				curr_vel = vel_req;
			}

			motor->setSpeed(curr_vel);

			/* check for still moving */
			if (curr_vel != 0.0f) {
				/* yes, mark planner active */
				active = 1;
			}

			/* It could be that closed loop is used direct and then the positioning should not call setSpeed.*/
			/* Therfore change state to 1 to call this planer not again if position is reached*/
			if (active == 0) {
				enable = 0;
				motor->stop();
				state = 1;
				if (flagShowResults) {
					errorHandler.setInfo(F("motor %i stop state=1\r\n"), motor->motorNo);
				}
			}

			if (flagShowResults) {
				sprintf(errorHandler.msg, "!03,motor %i speed: %f%% pos: %f cm posError %fcm e: %d a: %d\r\n", motor->motorNo, curr_vel, curr_pos, pos_cmd - curr_pos, enable, active);
				errorHandler.setInfo();
			}
		}
	}
	PT_END();
}


bool  TPositionControl::isPositionReached() {
	if (state == 1 && motor->isStopped()) {
		return true;
	}
	return false;

}

void TPositionControl::changeSpeed(long _speedPercentage) {

	max_vel = abs(_speedPercentage);

	if (max_vel > 70) { //if accelerate to 90% do it slower
		max_acc = 30.0f;
	}
	else { // here we rotate or decelarate from 90%
		max_acc = 60.0f;
	}
}

void TPositionControl::stop() {
	state = 0;
	max_acc = 80.0f;
	enable = 0; /* if zero, motion stops ASAP. Depending only on max_acc => max_dv */
	//active = 1;
	curr_vel = motor->getCurrentSpeedInPerc();
	max_vel = fabsf(curr_vel);

	if (max_vel < 4) {
		motor->stop();
		enable = 0;
		active = 0;
		state = 1;
		return;
	}

}

void TPositionControl::stopAtPerimeter() {
	// should only called while driving forward 
	curr_vel = motor->getCurrentSpeedInPerc();

	if (curr_vel < 0.0f) {  //This can happen if outside while rotating
		curr_vel = 0.0f;
		errorHandler.setInfo(F("stopAtPerimeter() called whith negative velocity\r\n"));
	}

	max_vel = fabsf(curr_vel);

	state = 0;
	if (max_vel > 80.0f) {
		// Worst case. Running with 90% speed over the perimeter. We have to slow down more. 
		max_acc = 90.0f;
	}
	else {
		// If near perimeter mower normaly slows down to 70% speed. 
		max_acc = 80.0f;
	}


	enable = 1;
	//active = 1;
	float pos_start = getCMForCounts(myEncoder->getTickCounter());

	forward = true;
	backward = false;
	pos_cmd = pos_start + CONF_DRIVE_OVER_PERIMETER_CM;

	if (max_vel < 40.0f) {
		max_vel = 40.0f;
	}

	/*
	if (motor->motorNo == 1) {
		errorHandler.setInfo(F("TPositionControl::stopAtPerimeter\r\n"));
	}
	*/

	if (flagShowResults) {
		errorHandler.setInfo(F("\r\n!03,rotateCM motor %i speed: %f%%\r\n"), motor->motorNo, max_vel);
		errorHandler.setInfo(F("!03,start: %fcm end: %fcm dist: %fcm\r\n"), pos_start, pos_cmd, CONF_DRIVE_OVER_PERIMETER_CM);
		//		errorHandler.setInfo(F("!03,original endpos: %fcm\r\n"), pos_cmd-addCmToTargetPosition);
	}




}


void TPositionControl::rotateAngle(float _angle, long _speedPercentage) {
	long counts = getCountsForDegree(_angle);
	float distanceCM = getCMForCounts(counts);
	rotateCM(distanceCM, _speedPercentage);
}

void TPositionControl::rotateCM(float _distanceCm, long _speedPercentage) {

	state = 0;
	max_acc = 60.0f;  //=> max_dv = max_acc * ta; max_dv = 60 * 0.1 = 6; This means maximum 6% acceleration per every loop.
	enable = 1;
	//active = 1;
	float pos_start = getCMForCounts(myEncoder->getTickCounter());
	//max_vel = abs(_speedPercentage);
	pos_cmd = pos_start + _distanceCm;
	curr_vel = motor->getCurrentSpeedInPerc();

	// if the  mower stops after rotating and then starts again, curr_vel could be differ 
	// for both wheels. if one wheel rotated back and the other rotated forward
	// current speed has different sign because the wheel is spinning very low
	// therefore set speed to 3% to put same speed to both motors to start running synchrone

	if (_distanceCm < 0.0f) {
		backward = true;
		forward = false;
		if (fabsf(curr_vel) < 4.0f) {
			curr_vel = -3.0f;
		}
	}
	else {
		forward = true;
		backward = false;
		if (fabsf(curr_vel) < 4.0f) {
			curr_vel = 3.0f;
		}
	}


	changeSpeed(_speedPercentage);

	if (max_vel < 0.1f) {  //If this happend we have a software bug
		errorHandler.setError(F("rotateCM() motor %i velocity <0.1\r\n"), motor->motorNo);
	}

	if (flagShowResults) {
		errorHandler.setInfo(F("\r\n!03,rotateCM motor %i speed: %f%%\r\n"), motor->motorNo, max_vel);
		errorHandler.setInfo(F("!03,start: %fcm end: %fcm dist: %fcm\r\n"), pos_start, pos_cmd, _distanceCm);
		//		errorHandler.setInfo(F("!03,original endpos: %fcm\r\n"), pos_cmd-addCmToTargetPosition);
	}


}

void TPositionControl::reset()  // Motors must be stopped before calling!!!
{
	enable = 0;
	active = 0;
	state = 1;
}

void TPositionControl::showConfig() {
	errorHandler.setInfo(F("!03,PC Config MotorNo: %i\r\n"), motor->motorNo);
	errorHandler.setInfo(F("!03,enabled: %d\r\n"), IsRunning());
	errorHandler.setInfo(F("!03,interval: %lu\r\n"), interval);
	errorHandler.setInfo(F("!03,stopCmBeforeTarget %f\r\n"), stopCmBeforeTarget);
	//errorHandler.setInfo(F("!03,addCmToTargetPosition %f\r\n"), addCmToTargetPosition);
}


// ---------------------------------------------------------
// Umrechnungs Routinen
// ---------------------------------------------------------

long TPositionControl::getCountsForCM(float x) {
	float y;
	y = (x * CONF_ENCTICKSPERREVOLUTION) / CONF_RADUMFANG_CM;
	return y;
}

float TPositionControl::getCMForCounts(float x) {
	float y;
	y = (x * CONF_RADUMFANG_CM) / CONF_ENCTICKSPERREVOLUTION;
	return y;
}

long TPositionControl::getCountsForDegree(float x) {
	float y;
	y = x * CONF_ENCTICKSPERREVOLUTION / 360.0f;
	return y;
}

float TPositionControl::getDegreeForCounts(float x) {
	float y;
	y = x * 360.0f / CONF_ENCTICKSPERREVOLUTION;
	return y;
}







