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

#ifndef BH_PERIMETEROUTSIDE_H
#define BH_PERIMETEROUTSIDE_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "BehaviourTree.h"
#include "UseServices.h"

//#define DEBUG_ROTATE_RANDOM_NUMBERS 1

#ifdef DEBUG_ROTATE_RANDOM_NUMBERS
#  define DRRN(x) x
#else
#  define DRRN(x)
#endif

class TDriveBumperFree : public Action    // Each task will be a class (derived from Node of course).
{
private:
	bool bumperFree;
	unsigned long time;
public:

	TDriveBumperFree() {}

	virtual void onInitialize(Blackboard& bb) {

		bumperFree = false;

		float distance = bb.history0.distanceSoll - bb.history0.distanceIst;

		switch (bb.history0.driveDirection) {
		case DD_FORWARD:
			srvMotor.rotateCM(fabs(distance), bb.history0.cruiseSpeed);
			break;
		case DD_REVERSE:
			srvMotor.rotateCM(-fabs(distance), bb.history0.cruiseSpeed);
			break;
		case DD_ROTATECC:
			srvMotor.turnTo(-fabs(distance), -bb.history0.cruiseSpeed);
			bb.numberOfRotations++;
			break;
		case DD_ROTATECW:
			srvMotor.turnTo(fabs(distance), bb.history0.cruiseSpeed);
			bb.numberOfRotations++;
			break;
		default:
			sprintf(errorHandler.msg, "!03,TDriveBumperFree driveDirection not found: %s", enuDriveDirectionString[bb.history0.driveDirection]);
			errorHandler.setError();
			break;
		}


		errorHandler.setInfo(F("!05,TDriveBumperFree DD: %s Dist: %f\r\n"), enuDriveDirectionString[bb.history0.driveDirection], distance);

	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (getTimeInNode() > 10000) {
			errorHandler.setError(F("!03,TDriveBumperFree  too long in state\r\n"));
		}

		if (srvMotor.isPositionReached()) {
			return BH_SUCCESS;
		}

		switch (bumperFree) {
		case false:
			if (!srvBumperSensor.isBumperActivated()) {
				time = getTimeInNode();
				bumperFree = true;
			}
			break;
		case true:
			if (getTimeInNode() - time > 500) {
				return BH_SUCCESS;
			}
			break;
		}

		return BH_RUNNING;
	}

	virtual void onTerminate(NodeStatus status, Blackboard& bb) {
		/*
		if(status != BH_ABORTED) {

		}
		*/

	}
};

class TDriveHist0 : public Action    // Each task will be a class (derived from Node of course).
{
private:
public:

	TDriveHist0() {}

	virtual void onInitialize(Blackboard& bb) {

		float distance = bb.history0.distanceSoll - bb.history0.distanceIst;

		if (bb.history0.cruiseSpeed == 0) {
			errorHandler.setError(F("!03,TDiveHist0 cruisespeed not set\r\n"));
		}

		switch (bb.history0.driveDirection) {
		case DD_FORWARD:
			srvMotor.rotateCM(fabs(distance), bb.history0.cruiseSpeed);
			break;
		case DD_REVERSE:
			srvMotor.rotateCM(-fabs(distance), bb.history0.cruiseSpeed);
			break;
		case DD_ROTATECC:
			srvMotor.turnTo(-fabs(distance), bb.history0.cruiseSpeed);
			bb.numberOfRotations++;
			break;
		case DD_ROTATECW:
			srvMotor.turnTo(fabs(distance), bb.history0.cruiseSpeed);
			bb.numberOfRotations++;
			break;
		default:
			sprintf(errorHandler.msg, "!03,TDiveHist0 driveDirection not found: %s", enuDriveDirectionString[bb.history0.driveDirection]);
			errorHandler.setError();
			break;
		}


		errorHandler.setInfo(F("!05,TDiveHist0 DD: %s Dist: %f\r\n"), enuDriveDirectionString[bb.history0.driveDirection], distance);

	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (getTimeInNode() > 10000) {
			errorHandler.setError(F("!03,TDiveHist0  too long in state\r\n"));
		}

		if (srvMotor.isPositionReached()) {
			return BH_SUCCESS;
		}
		return BH_RUNNING;
	}

	virtual void onTerminate(NodeStatus status, Blackboard& bb) {
		/*
		if(status != BH_ABORTED) {

		}
		*/

	}
};


class TCalcAngle : public Action    // Each task will be a class (derived from Node of course).
{
private:
	bool isArcNotInitialised;
	int state0Count;
	int state1Count;
	int state2Count;
	int state0CountMax;
	int state1CountMax;
	float state2CountMax;

	int state;

	int angleCounter;

	enuDriveDirection lastRotateDirection;

	THistory hist;


public:


	TCalcAngle() {
		isArcNotInitialised = true;
		angleCounter = 0;
		state = 0;
		lastRotateDirection = DD_ROTATECW;
	}

	virtual void onInitialize(Blackboard& bb) {}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		lastRotateDirection = bb.historyGetLastRotateDirection();

		//float distance1, distance2, distance3;
		//int8_t result;

		hist = bb.getInitialisedHistoryEntry();
		hist.cruiseSpeed = bb.CRUISE_SPEED_MEDIUM;

		// Dertermine Drive Direction
		if (bb.history0.coilFirstOutside == CO_BOTH) { // Beide Coils waren gleichzeitig draussen
			if (lastRotateDirection == DD_ROTATECW) { 
				hist.driveDirection = DD_ROTATECW;
				hist.distanceSoll = 1.0f;
				if (bb.flagShowRotateX) {
					errorHandler.setInfo(F("!05,CO_BOTH => DD_ROTATECW;\r\n"));
				}
			}
			else {
				hist.driveDirection = DD_ROTATECC;
				hist.distanceSoll = -1.0f;
				if (bb.flagShowRotateX) {
					errorHandler.setInfo(F("!05,CO_BOTH => DD_ROTATECC;\r\n"));
				}
			}
		}
		else if (bb.history0.coilFirstOutside == CO_LEFT) {
			hist.driveDirection = DD_ROTATECW;
			hist.distanceSoll = 1.0f;
			if (bb.flagShowRotateX) {
				errorHandler.setInfo(F("!05,CO_LEFT => DD_ROTATECW;\r\n"));
			}
		}
		else { // if ( bb.history0.coilFirstOutside == CO_RIGHT) {
			hist.driveDirection = DD_ROTATECC;
			hist.distanceSoll = -1.0f;
			if (bb.flagShowRotateX) {
				errorHandler.setInfo(F("!05,CO_RIGHT => DD_ROTATECC;\r\n"));
			}
		}

		// Dertermine Drive Direction

		if (isArcNotInitialised) {
			state0CountMax = myRandom(9, 23);
			state1CountMax = myRandom(5, 12);
			state2CountMax = myRandom(2, 5);
			state0Count = 0;
			state1Count = 0;
			state2Count = 0;
			angleCounter = 0;
			isArcNotInitialised = false;
		}


		/**********************
		* Drehwinkel berechnen
		**********************/

		switch (state) {

			//transfer functions

		case 0: // Default mow routine

			state0Count++;

			// calcualte default angle
			if (bb.history0.coilOutsideAfterOverrun == CO_BOTH) {
				hist.distanceSoll *= myRandom(35, 115) + CONF_PER_CORRECTION_ANGLE; //108
				errorHandler.setInfo(F("!05,TCalcAngle CO_BOTH 35-115: %f\r\n"), hist.distanceSoll);
			}
			else {
				hist.distanceSoll *= myRandom(30, 80) + CONF_PER_CORRECTION_ANGLE; //70
				errorHandler.setInfo(F("!05,TCalcAngle CO_LR 30-80: %f\r\n"), hist.distanceSoll);
			}


			if (bb.flagShowRotateX) {
				errorHandler.setInfo(F("!05,TCalcAngle CO_LR 30-70: %f\r\n"), hist.distanceSoll);
			}

			/*
			// Check for short way and override the above angel if two times short way or no two forawards found
			result = bb.histGetThreeLastForwardDistances(distance1, distance2, distance3);

			if (result==3) {
				if (distance1 < 100 && distance2 < 100 && distance3 < 100) {
					bb.arcRotateXArc = myRandom(90, 130) + CONF_PER_CORRECTION_ANGLE;
					if (bb.flagShowRotateX) {
						errorHandler.setInfo(F("!5,override result=true S0:20-50: %ld\r\n"), bb.arcRotateXArc);
					}
				}
			}
			*/
			/*
			else { // also if result == false use small angel
				bb.arcRotateXArc = myRandom(20, 50) + CONF_PER_CORRECTION_ANGLE;
				if (bb.flagShowRotateX) {
					errorHandler.setInfo(F("!5,override result=false S0:20-50: %ld\r\n"), bb.arcRotateXArc);
				}
			}*/



			// Transition
			if (state0Count >= state0CountMax) {
				state = 1;
			}

			break; //case 0

		case 1:
			state1Count++;


			if (bb.history0.coilOutsideAfterOverrun == CO_BOTH) {
				hist.distanceSoll *= myRandom(100, 155) + CONF_PER_CORRECTION_ANGLE;
				errorHandler.setInfo(F("!05,TCalcAngle CO_BOTH 100-155: %ld\r\n"), hist.distanceSoll);
			}
			else {
				hist.distanceSoll *= myRandom(80, 100) + CONF_PER_CORRECTION_ANGLE;
				errorHandler.setInfo(F("!05,TCalcAngle CO_LR 80-100: %ld\r\n"), hist.distanceSoll);
			}

			if (bb.flagShowRotateX) {
				errorHandler.setInfo(F("!05,s1:60-110: %f\r\n"), hist.distanceSoll);
			}


			/*
			// Check for short way and override the above angel
			result = bb.histGetThreeLastForwardDistances(distance1, distance2);

			if (result) {
				if (distance1 < 100 && distance2 < 100 && distance3 < 100) {
					bb.arcRotateXArc = myRandom(90, 130) + CONF_PER_CORRECTION_ANGLE;
					if (bb.flagShowRotateX) {
						errorHandler.setInfo(F("!5,override result = true S1:20-50: %ld\r\n"), bb.arcRotateXArc);
					}
				}
			}
			*/
			/*
			else {
				{ // also if result == false use small angel
					bb.arcRotateXArc = myRandom(20, 50) + CONF_PER_CORRECTION_ANGLE;
					if (bb.flagShowRotateX) {
						errorHandler.setInfo(F("!5,override result=false S1:20-50: %ld\r\n"), bb.arcRotateXArc);
					}
			}*/




			// Transition
			if (state1Count >= state1CountMax) {
				isArcNotInitialised = true;
				state = 0;
			}

			break;
		case 2:
			/*
			state2Count++;
			alpha = random(5, 15);
			print("\t!05,5-30");

			// Transition
			if (state2Count >= state2CountMax) {
			isArcNotInitialised = true;
			state = 0;
			}
			*/
			break;
		}

		//  Check if oszilating at corner
		if (bb.history0.driveDirection == DD_FORWARD && bb.history[2].driveDirection == DD_FORWARD
			&& bb.history0.distanceIst < 60 && bb.history[2].distanceIst < 60
			&& ((bb.history[1].driveDirection == DD_ROTATECW && bb.history[3].driveDirection == DD_ROTATECC)
				|| (bb.history[1].driveDirection == DD_ROTATECC && bb.history[3].driveDirection == DD_ROTATECW))) {

			hist.distanceSoll = myRandom(110, 130);

			if (bb.history[1].driveDirection == DD_ROTATECW) {
				hist.driveDirection = DD_ROTATECW;
			}
			else {
				hist.driveDirection = DD_ROTATECC;
				hist.distanceSoll = -hist.distanceSoll;
			}
			errorHandler.setInfo(F("!05,TCalcAngle FORCE 110-130: %f\r\n"), hist.distanceSoll);
		}

		bb.addHistoryEntry(hist);
		return BH_SUCCESS;

	}

};






#endif
