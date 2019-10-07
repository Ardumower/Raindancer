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

//#define DEBUG_ROTATE_RANDOM_NUMBERS 1

#ifdef DEBUG_ROTATE_RANDOM_NUMBERS
#  define DRRN(x) x
#else
#  define DRRN(x)
#endif



class TSetArc20CW : public Action    // Each task will be a class (derived from Node of course).
{
private:
public:

	TSetArc20CW() {}

	virtual void onInitialize(Blackboard& bb) {
		bb.flagForceRotateDirection = FRD_CW;
		bb.driveDirection = DD_FEOROTATECW;
		bb.arcRotateXArc = 10;
		bb.flagForceSmallRotAngle = 2;
		bb.flagDeactivateRotInside = false;
		bb.flagCoilFirstOutside = CO_LEFT;
		if (bb.flagShowRotateX) {
			errorHandler.setInfo(F("!05,TSetArc20CW set bb.flagForceSmallRotAngle = 2;\r\n"));
		}
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		return BH_SUCCESS;
	}

};

class TSetArc20CC : public Action    // Each task will be a class (derived from Node of course).
{
private:
public:

	TSetArc20CC() {}

	virtual void onInitialize(Blackboard& bb) {
		bb.flagForceRotateDirection = FRD_CC;
		bb.driveDirection = DD_FEOROTATECC;
		bb.arcRotateXArc = 10;
		bb.flagForceSmallRotAngle = 2;
		bb.flagDeactivateRotInside = false;
		bb.flagCoilFirstOutside = CO_RIGHT;
		if (bb.flagShowRotateX) {
			errorHandler.setInfo(F("!05,TSetArc20CW set bb.flagForceSmallRotAngle = 2;\r\n"));
		}
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		return BH_SUCCESS;
	}

};

class TSetArcFEO_ROTCC1 : public Action    // Each task will be a class (derived from Node of course).
{
private:
public:

	TSetArcFEO_ROTCC1() {}

	virtual void onInitialize(Blackboard& bb) {
		bb.flagForceRotateDirection = FRD_CC;
		bb.driveDirection = DD_FEOROTATECC1;
		bb.arcRotateXArc = myRandom(60, 135);
		if (bb.flagShowRotateX) {
			errorHandler.setInfo(F("!05,TSetArcFEO_ROTCC:60-135: %ld\r\n"), bb.arcRotateXArc);
		}
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		return BH_SUCCESS;
	}

};

class TSetArcFEO_ROTCW1 : public Action    // Each task will be a class (derived from Node of course).
{
private:
public:

	TSetArcFEO_ROTCW1() {}

	virtual void onInitialize(Blackboard& bb) {
		bb.flagForceRotateDirection = FRD_CW;
		bb.driveDirection = DD_FEOROTATECW1;
		bb.arcRotateXArc = myRandom(60, 135);
		if (bb.flagShowRotateX) {
			errorHandler.setInfo(F("!05,TSetArcFEO_ROTCW:60-135: %ld\r\n"), bb.arcRotateXArc);
		}
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		return BH_SUCCESS;
	}

};

class TSetArcFEO_ROTCC2 : public Action    // Each task will be a class (derived from Node of course).
{
private:
public:

	TSetArcFEO_ROTCC2() {}

	virtual void onInitialize(Blackboard& bb) {
		bb.flagForceRotateDirection = FRD_CC;
		bb.driveDirection = DD_FEOROTATECC2;
		bb.arcRotateXArc = myRandom(60, 135);
		if (bb.flagShowRotateX) {
			errorHandler.setInfo(F("!05,TSetArcFEO_ROTCC:60-135: %ld\r\n"), bb.arcRotateXArc);
		}
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		return BH_SUCCESS;
	}

};

class TSetArcFEO_ROTCW2 : public Action    // Each task will be a class (derived from Node of course).
{
private:
public:

	TSetArcFEO_ROTCW2() {}

	virtual void onInitialize(Blackboard& bb) {
		bb.flagForceRotateDirection = FRD_CW;
		bb.driveDirection = DD_FEOROTATECW2;
		bb.arcRotateXArc = myRandom(60, 135);
		if (bb.flagShowRotateX) {
			errorHandler.setInfo(F("!05,TSetArcFEO_ROTCW:60-135: %ld\r\n"), bb.arcRotateXArc);
		}
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		return BH_SUCCESS;
	}

};

class TSetArcFEO_ROT : public Action    // Each task will be a class (derived from Node of course).
{
private:
public:

	TSetArcFEO_ROT() {}

	virtual void onInitialize(Blackboard& bb) {

		// select random angle first
		int i = myRandom(0, 10000);
		if (i < 5000) {
			bb.flagForceRotateDirection = FRD_CC;
			bb.driveDirection = DD_FEOROTATECC;
		}
		else {
			bb.flagForceRotateDirection = FRD_CW;
			bb.driveDirection = DD_FEOROTATECW;
		}

		// Overwrite random angle when usinng left and right bumper. If Bumperduino on userswitch is activated, left and right bumper may not be activated.
		// Therefore the random direction is set first.
		if (CONF_ESCAPEDIR_DEPENDING_ON_BUMPER) {
			if (bb.flagBumperActivatedRight) {
				bb.flagForceRotateDirection = FRD_CC;   //record the next rotate dir
				bb.driveDirection = DD_FEOROTATECC;
				if (bb.flagShowRotateX) {
					errorHandler.setInfo(F("!05,TSetArcFEO_ROT:Right Bumper: %ld\r\n"), bb.arcRotateXArc);
				}
			}
			if (bb.flagBumperActivatedLeft) {
				bb.flagForceRotateDirection = FRD_CW;  //record the next rotate dir
				bb.driveDirection = DD_FEOROTATECW;
				if (bb.flagShowRotateX) {
					errorHandler.setInfo(F("!05,TSetArcFEO_ROT:Left Bumper: %ld\r\n"), bb.arcRotateXArc);
				}
			}
		}


		bb.arcRotateXArc = myRandom(60, 135);
		if (bb.flagShowRotateX) {
			errorHandler.setInfo(F("!05,TSetArcFEO_ROT:60-135: %ld\r\n"), bb.arcRotateXArc);
		}
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		return BH_SUCCESS;
	}

};



class TRotateX : public Action    // Each task will be a class (derived from Node of course).
{
private:
public:

	TRotateX() {}

	virtual void onInitialize(Blackboard& bb) {

		//bb.motor.startMeasurementAngleRotated();

		bb.cruiseSpeed = bb.CRUISE_ROTATE_HIGH;
		if (bb.flagForceRotateDirection == FRD_CC) {
			bb.motor.turnTo(-bb.arcRotateXArc, bb.cruiseSpeed);
		}
		else {
			bb.motor.turnTo(bb.arcRotateXArc, bb.cruiseSpeed);
		}

		bb.flagForceRotateDirection = FRD_NONE;

		bb.numberOfRotations++;

		errorHandler.setInfo(F("!05,TRotateX: %s DD %s\r\n"), enuFlagForceRotateDirectionString[bb.flagForceRotateDirection], enuDriveDirectionString[bb.driveDirection]);

	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (getTimeInNode() > 10000) {
			errorHandler.setError(F("!03,TRotateX  too long in state\r\n"));
		}

		bb.history[0].rotAngleIst = bb.motor.getAngleRotatedAngleDeg();

		if (bb.motor.isPositionReached()) {
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







/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
class TPreUpdateHistoryBump : public Action    // Each task will be a class (derived from Node of course).
{
public:

	TPreUpdateHistoryBump() {}

	virtual void onInitialize(Blackboard& bb) {

	} //onInitialize

	virtual NodeStatus onUpdate(Blackboard& bb) {


		if (bb.flagForceSmallRotAngle > 0) {
			bb.arcRotateXArc = myRandom(10, 30);
			errorHandler.setInfo(F("!05,HistBump flagForceSmallRotAngle to %d\r\n"), bb.flagForceSmallRotAngle);
			errorHandler.setInfo(F("!05,HistBump set ForceSmallRotAngle to %d\r\n"), bb.arcRotateXArc);
			bb.flagForceSmallRotAngle--;
		}

		bb.addHistoryEntry(bb.driveDirection, 0.0f, bb.arcRotateXArc, 0.0f, bb.flagForceRotateDirection, CO_NONE);

		return BH_SUCCESS;
	}
};



class TPreUpdateHistory : public Action    // Each task will be a class (derived from Node of course).
{
public:

	TPreUpdateHistory() {}

	virtual void onInitialize(Blackboard& bb) {

	} //onInitialize

	virtual NodeStatus onUpdate(Blackboard& bb) {


		if (bb.flagForceSmallRotAngle > 0) {
			bb.arcRotateXArc = myRandom(10, 30);
			errorHandler.setInfo(F("!05,HistPer flagForceSmallRotAngle to %d\r\n"), bb.flagForceSmallRotAngle);
			errorHandler.setInfo(F("!05,HistPer ForceSmallRotAngle to %d\r\n"), bb.arcRotateXArc);
			bb.flagForceSmallRotAngle--;
		}

		bb.addHistoryEntry(bb.driveDirection, 0.0f, bb.arcRotateXArc, 0.0f, bb.flagForceRotateDirection, bb.flagCoilFirstOutside);

		return BH_SUCCESS;
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

public:


	TCalcAngle() {
		isArcNotInitialised = true;
		angleCounter = 0;
		state = 0;
	}

	virtual void onInitialize(Blackboard& bb) {}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		float distance1, distance2;
		bool result;

		bb.flagDeactivateRotInside = false;

		// Dertermine Drive Direction
		if (bb.flagCoilFirstOutside == CO_BOTH) { // Beide Coils waren gleichzeitig draussen
			if (bb.history[0].driveDirection == DD_ROTATECW) { // NOCH KORRIGIEREN KAnn niemals auftreten
				bb.driveDirection = DD_ROTATECC;
				bb.flagForceRotateDirection = FRD_CC;
				if (bb.flagShowRotateX) {
					errorHandler.setInfo(F("!05,CO_BOTH => FRD_CC;\r\n"));
				}
			}
			else {
				bb.driveDirection = DD_ROTATECW;
				bb.flagForceRotateDirection = FRD_CW;
				if (bb.flagShowRotateX) {
					errorHandler.setInfo(F("!05,CO_BOTH => FRD_CW;\r\n"));
				}
			}
		}
		else if (bb.flagCoilFirstOutside == CO_LEFT) {
			bb.flagForceRotateDirection = FRD_CW;
			bb.driveDirection = DD_ROTATECW;
			if (bb.flagShowRotateX) {
				errorHandler.setInfo(F("!05,CO_LEFT => FRD_CW;\r\n"));
			}
		}
		else { // if ( bb.flagCoilFirstOutside == CO_RIGHT) {
			bb.flagForceRotateDirection = FRD_CC;
			bb.driveDirection = DD_ROTATECC;
			if (bb.flagShowRotateX) {
				errorHandler.setInfo(F("!05,CO_RIGHT => FRD_CC;\r\n"));
			}
		}

		// Dertermine Drive Direction

		if (isArcNotInitialised) {
			state0CountMax = myRandom(9, 23);
			state1CountMax = myRandom(2, 5);
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
			bb.arcRotateXArc = myRandom(5, 80) + CONF_PER_CORRECTION_ANGLE;
			if (bb.flagShowRotateX) {
				errorHandler.setInfo(F("!05,S0: 5-80: %ld\r\n"), bb.arcRotateXArc);
			}


			// Check for short way and override the above angel if two times short way or no two forawards found 
			result = bb.histGetTwoLastForwardDistances(distance1, distance2);

			if (result) {
				if (distance1 < 150 && distance2 < 150) {
					bb.arcRotateXArc = myRandom(20, 50) + CONF_PER_CORRECTION_ANGLE;
					if (bb.flagShowRotateX) {
						errorHandler.setInfo(F("!5,override result=true S0:20-50: %ld\r\n"), bb.arcRotateXArc);
					}
				}
			}/*
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


			bb.arcRotateXArc = myRandom(60, 110) + CONF_PER_CORRECTION_ANGLE;
			if (bb.flagShowRotateX) {
				errorHandler.setInfo(F("!05,s1:60-110: %ld\r\n"), bb.arcRotateXArc);
			}

			// Check for short way and override the above angel
			result = bb.histGetTwoLastForwardDistances(distance1, distance2);

			if (result) {
				if (distance1 < 150 && distance2 < 150) {
					bb.arcRotateXArc = myRandom(20, 50) + CONF_PER_CORRECTION_ANGLE;
					if (bb.flagShowRotateX) {
						errorHandler.setInfo(F("!5,override result = true S1:20-50: %ld\r\n"), bb.arcRotateXArc);
					}
				}
			}/*
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


		return BH_SUCCESS;

	}

};


class TRotateBothCoilsInside
	: public Action
{
private:

public:


	TRotateBothCoilsInside() {}


	virtual void onInitialize(Blackboard& bb) {

		if (bb.flagDeactivateRotInside) {
			if (bb.flagShowRotateX) {
				errorHandler.setInfo(F("RBCI flagDeactivateRotInside == true\r\n"));
			}
			return;
		}

		bb.cruiseSpeed = bb.CRUISE_SPEED_OBSTACLE;

		if (bb.flagForceRotateDirection == FRD_CC) {
			bb.motor.turnTo(-380, bb.cruiseSpeed);
			if (bb.flagShowRotateX) {
				errorHandler.setInfo(F("RBCI FRD_CC rotateInside\r\n"));
			}
		}
		else {
			bb.motor.turnTo(380, bb.cruiseSpeed);
			if (bb.flagShowRotateX) {
				errorHandler.setInfo(F("RBCI FRD_CW rotateInside\r\n"));
			}
		}
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (getTimeInNode() > 15000) {
			errorHandler.setError(F("!03,TRotateBothCoilsInside  too long in state\r\n"));
		}

		bb.history[0].rotAngleIst = bb.motor.getAngleRotatedAngleDeg();

		if (bb.flagDeactivateRotInside) {
			bb.flagDeactivateRotInside = false;
			if (bb.flagShowRotateX) {
				errorHandler.setInfo(F("RBCI set flagDeactivateRotInside = false\r\n"));
			}
			return BH_SUCCESS;
		}

		if (bb.perimeterSensoren.isLeftInside() && bb.perimeterSensoren.isRightInside()) { // wenn beide coils innen dann weitermachen
			return BH_SUCCESS;
		}

		if (bb.motor.isPositionReached()) {
			return BH_FAILURE;
			//errorHandler.setError(F("!03,TRotateBothCoilsInside not able to rotate both coils to inside\r\n"));
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

class TRotateDriveBackInside : public Action {
private:
	long weg;
public:

	TRotateDriveBackInside() {}

	virtual void onInitialize(Blackboard& bb) {

		bb.cruiseSpeed = bb.CRUISE_SPEED_HIGH;
		bb.motor.rotateCM(-(3* CONF_DRIVE_OVER_PERIMETER_CM), bb.cruiseSpeed); // x cm zurueckfahren
		bb.driveDirection = DD_REVERSE_INSIDE; // DD_REVERSE_INSIDE;
		//bb.addHistoryEntry(bb.driveDirection, 0.0f, 0.0f, 0.0f, FRD_NONE, bb.flagCoilFirstOutside);

	}

	virtual NodeStatus onUpdate(Blackboard& bb) {
		if (getTimeInNode() > 10000) {
			errorHandler.setError(F("!03,TRotateDriveBackInside too long in state\r\n"));
		}

		//bb.history[0].distanceDriven = bb.motor.getDistanceInCM();

		if (bb.motor.isPositionReached()) {
			if (bb.perimeterSensoren.isLeftInside() && bb.perimeterSensoren.isRightInside()) {
				return BH_SUCCESS;
			}
			else {
				errorHandler.setError(F("!03,TRotateDriveBackInside not able to drive back inside\r\n"));
				return BH_FAILURE;
			}
			
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

/*
class TPreUpdateHistory : public Action    // Each task will be a class (derived from Node of course).
{
public:
	bool showValuesOnConsole;

	TPreUpdateHistory() :showValuesOnConsole(false) {}

	virtual void onInitialize(Blackboard& bb) {

	} //onInitialize

	virtual NodeStatus onUpdate(Blackboard& bb) {

		// Insert data to history captured from last rotation to now perimeter reached
		bb.history[0].distanceDriven = bb.motor.getDistanceInCMForTraveled(); // Driven distance from end of last rotating until now
		bb.history[0].coilFirstOutside = bb.flagCoilFirstOutside;    // which coil was first outside jet

		//Wird in TPostUpdateHistory eingetragen nachdem history geshiftet wurde:
		//bb.history[0].lastRotAngle = bb.randAngle;
		//bb.history[0].lastRotDirection = bb.driveDirection;

		// Im Array steht nun an erster Position [0], die letzte gefahrene Distanz, der letzte gedrehte Winkel und die letzte gedrehte Richtung
		// Sowie die aktuellen Werte fuer: flagCoilFirstOutside, getDistanceDiffInCMForCoilOut, getDistanceAngleCoilOut
		// Auf Grundlage dieser Wert muessen nun Entscheidungen getroffen werden. Unten werden dann der neue Winkel und Winkelrichtung eingetragen, beim nÃ¤chsten Aufruf dieser
		// Funktion die dazugehorrige distanz in [0] oben eingetragen


		if (showValuesOnConsole == true) {

			sprintf(errorHandler.msg, "!05,distanceDriven: %ld %ld %ld\r\n", bb.history[0].distanceDriven, bb.history[1].distanceDriven, bb.history[2].distanceDriven);
			errorHandler.setInfo();

			sprintf(errorHandler.msg, "!05,coilFirstOutside %d %d %d\r\n", bb.history[0].coilFirstOutside, bb.history[1].coilFirstOutside, bb.history[2].coilFirstOutside);
			errorHandler.setInfo();


			sprintf(errorHandler.msg, "!05,lastRotDirection ");
			errorHandler.setInfo();
			for (int i = 0; i < HISTROY_BUFSIZE; i++) {
				if (bb.history[i].lastRotDirection == DD_ROTATECW) {
					sprintf(errorHandler.msg, "CW ");
				}
				else if (bb.history[i].lastRotDirection == DD_ROTATECC) {
					sprintf(errorHandler.msg, "CC ");
				}
				else {
					sprintf(errorHandler.msg, "NA! ");
				}

				errorHandler.setInfo();
			}
			errorHandler.setInfo("\r\n");

			sprintf(errorHandler.msg, "!05,lastRotAngle: %ld %ld %ld\r\n", bb.history[0].lastRotAngle, bb.history[1].lastRotAngle, bb.history[2].lastRotAngle);
			errorHandler.setInfo();
		}


		bb.motor.startMeasurementAngleRotated();

		return BH_SUCCESS;
	}
};
*/

/*
class TPostUpdateHistory : public Action    // Each task will be a class (derived from Node of course).
{
public:
	bool showValuesOnConsole;
	TPostUpdateHistory() :showValuesOnConsole(false) {}

	virtual void onInitialize(Blackboard& bb) {
	} //onInitialize

	virtual NodeStatus onUpdate(Blackboard& bb) {

		for (int i = HISTROY_BUFSIZE - 1; i > 0; i--) {
			bb.history[i] = bb.history[i - 1];
		}
		bb.history[0].lastRotAngle = bb.randAngle;
		bb.history[0].lastRotDirection = bb.driveDirection;


		if (showValuesOnConsole == true) {
			sprintf(errorHandler.msg, "!05,new rotDirection: ");
			errorHandler.setInfo();

			if (bb.history[0].lastRotDirection == DD_ROTATECW) {
				sprintf(errorHandler.msg, "CW ");
			}
			else if (bb.history[0].lastRotDirection == DD_ROTATECC) {
				sprintf(errorHandler.msg, "CC ");
			}
			else {
				sprintf(errorHandler.msg, "NA! ");
			}

			errorHandler.setInfo();
			errorHandler.setInfo("\r\n");


			sprintf(errorHandler.msg, "!05,new angle: %ld\r\n", bb.history[0].lastRotAngle);
			errorHandler.setInfo();
		}

		return BH_SUCCESS;
	}
};
*/

/*
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

public:

	bool showValuesOnConsole;

	TCalcAngle() :showValuesOnConsole(false) {
		isArcNotInitialised = true;
		angleCounter = 0;
		state = 0;
	}

	virtual void onInitialize(Blackboard& bb) {}

	virtual NodeStatus onUpdate(Blackboard& bb) {


		if (bb.flagForceAngle) { // use bb.randAngle  to rotate
			if (showValuesOnConsole) {
				errorHandler.setInfo(F("!05,flagForceAngle: %ld\r\n"), bb.randAngle);
			}
			bb.flagForceAngle = false;
			return BH_SUCCESS;
		}

		//If bumper was activated choose angle
		if (bb.flagBumperInsidePerActivated || bb.flagBumperOutsidePerActivated) {
			bb.randAngle = myRandom(60, 135);
			if (showValuesOnConsole) {
				errorHandler.setInfo(F("!05,bump:60-135: %ld\r\n"), bb.randAngle);
			}
			return BH_SUCCESS;
		}


		if (isArcNotInitialised) {
			state0CountMax = myRandom(9, 23);
			state1CountMax = myRandom(2, 5);
			state2CountMax = myRandom(2, 5);
			state0Count = 0;
			state1Count = 0;
			state2Count = 0;
			angleCounter = 0;
			isArcNotInitialised = false;
		}



		switch (state) {

			//transfer functions

		case 0: // Default mow routine

			state0Count++;

			bb.randAngle = myRandom(5, 80);
			if (showValuesOnConsole) {
				errorHandler.setInfo(F("!05,s0: 5-80: %ld\r\n"), bb.randAngle);
			}

			// Transition
			if (state0Count >= state0CountMax) {
				state = 1;
			}

			break; //case 0

		case 1:
			state1Count++;

			if (bb.history[0].distanceDriven <100) {
				bb.randAngle = myRandom(20, 50);
				if (showValuesOnConsole) {
					errorHandler.setInfo(F("!05,s1:20-50: %ld\r\n"), bb.randAngle);
				}
			}
			else {
				bb.randAngle = myRandom(60, 110);
				if (showValuesOnConsole) {
					errorHandler.setInfo(F("!05,s1:60-110: %ld\r\n"), bb.randAngle);
				}
			}

			// Transition
			if (state1Count >= state1CountMax) {
				isArcNotInitialised = true;
				state = 0;
			}

			break;
		case 2:

			break;
		}


		return BH_SUCCESS;

	}

};
*/



/*
class TRotateBothCoilsInside
	: public Action
{
private:

public:

	bool showValuesOnConsole;

	TRotateBothCoilsInside() :showValuesOnConsole(false) {}


	virtual void onInitialize(Blackboard& bb) {

		if (bb.flagDeactivateRotInside) {
			if (showValuesOnConsole) {
				errorHandler.setInfo(F("RBCI flagDeactivateRotInside == true\r\n"));
			}
			return;
		}

		bb.cruiseSpeed = bb.CRUISE_SPEED_OBSTACLE;

		if (bb.flagForceRotateDirection == FRD_CC) {
			bb.motor.turnTo(-380, bb.cruiseSpeed);
			bb.driveDirection = DD_ROTATECC;
			if (showValuesOnConsole) {
				errorHandler.setInfo(F("RBCI FRD_CC rotateInside\r\n"));
			}
		}
		else if (bb.flagForceRotateDirection == FRD_CW) {
			bb.motor.turnTo(380, bb.cruiseSpeed);
			bb.driveDirection = DD_ROTATECW;
			if (showValuesOnConsole) {
				errorHandler.setInfo(F("RBCI FRD_CC rotateInside\r\n"));
			}
		}
		else if (bb.perimeterSensoren.isRightOutside() && bb.perimeterSensoren.isLeftInside()) {
			bb.motor.turnTo(-380, bb.cruiseSpeed);
			bb.flagForceRotateDirection = FRD_CC;
			bb.driveDirection = DD_ROTATECC;
			if (showValuesOnConsole) {
				errorHandler.setInfo(F("RBCI RightOutside\r\n"));
			}
		}
		else if (bb.perimeterSensoren.isLeftOutside() && bb.perimeterSensoren.isRightInside()) {
			bb.motor.turnTo(380, bb.cruiseSpeed);
			bb.flagForceRotateDirection = FRD_CW;
			bb.driveDirection = DD_ROTATECW;
			if (showValuesOnConsole) {
				errorHandler.setInfo(F("RBCI LeftOutside\r\n"));
			}
		}
		else if (bb.flagCoilFirstOutside == CO_RIGHT) {
			bb.motor.turnTo(-380, bb.cruiseSpeed);
			bb.flagForceRotateDirection = FRD_CC;
			bb.driveDirection = DD_ROTATECC;
			if (showValuesOnConsole) {
				errorHandler.setInfo(F("RBCI CO_RIGHT\r\n"));
			}
		}
		else if (bb.flagCoilFirstOutside == CO_LEFT) {
			bb.motor.turnTo(380, bb.cruiseSpeed);
			bb.flagForceRotateDirection = FRD_CW;
			bb.driveDirection = DD_ROTATECW;
			if (showValuesOnConsole) {
				errorHandler.setInfo(F("RBCI CO_LEFT\r\n"));
			}
		}
		else { // if (bb.flagCoilFirstOutside == CO_BOTH) {
			bb.motor.turnTo(380, bb.cruiseSpeed);
			bb.flagForceRotateDirection = FRD_CW;
			bb.driveDirection = DD_ROTATECW;
			if (showValuesOnConsole) {
				errorHandler.setInfo(F("RBCI CO_BOTH\r\n"));
			}
		}
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (getTimeInNode() > 15000) {
			errorHandler.setError(F("!03,TRotateBothCoilsInside  too long in state\r\n"));
		}

		if (bb.flagDeactivateRotInside) {
			bb.flagDeactivateRotInside = false;
			errorHandler.setInfo(F("RBCI flagDeactivateRotInside = false\r\n"));
			return BH_SUCCESS;
		}

		if (bb.perimeterSensoren.isLeftInside() && bb.perimeterSensoren.isRightInside()) { // wenn beide coils innen dann weitermÃ¤hen
			return BH_SUCCESS;
		}

		if (bb.motor.isPositionReached()) {
			errorHandler.setError("!03,TRotateBothCoilsInside not able to rotate both coils to inside\r\n");
		}

		return BH_RUNNING;
	}

	virtual void onTerminate(NodeStatus status, Blackboard& bb) {

		//if(status != BH_ABORTED) {

		//}

	}

};

*/
/*
class TRotatePer : public Action    // Each task will be a class (derived from Node of course).
{
public:
	bool showValuesOnConsole;

	TRotatePer() :showValuesOnConsole(false) {}

	virtual void onInitialize(Blackboard& bb) {
		//
		// Drehung entprechend random Winkel starten
		//
		bb.cruiseSpeed = bb.CRUISE_SPEED_LOW;

		// flagForceRotateDirection beruecksichtigen
		if (bb.flagForceRotateDirection == FRD_CW) {
			if (showValuesOnConsole) { errorHandler.setInfo(F("!05,Force Rotation to FRD_CW\r\n")); }
			bb.flagCoilFirstOutside = CO_LEFT;
			bb.flagForceRotateDirection = FRD_NONE;
		}
		else if (bb.flagForceRotateDirection == FRD_CC) {
			if (showValuesOnConsole) { errorHandler.setInfo(F("!05,Force Rotation to FRD_CC\r\n")); }
			bb.flagCoilFirstOutside = CO_RIGHT;
			bb.flagForceRotateDirection = FRD_NONE;
		}

		// flagForceSmallRotAngle berücksichtigen
		if (bb.flagForceSmallRotAngle > 0) {
			bb.randAngle = myRandom(10, 30);
			bb.flagForceSmallRotAngle--;
			errorHandler.setInfo("!05,ForceSmallRotAngle\r\n");
		}


		if (bb.flagCoilFirstOutside == CO_BOTH) { // Beide Coils waren gleichzeitig drauÃŸen
			if (bb.history[0].lastRotDirection == DD_ROTATECW) {
				bb.motor.turnTo(-1 * bb.randAngle, bb.cruiseSpeed);
				bb.driveDirection = DD_ROTATECC;
			}
			else {
				bb.motor.turnTo(bb.randAngle, bb.cruiseSpeed);
				bb.driveDirection = DD_ROTATECW;
			}
		}
		else if (bb.flagCoilFirstOutside == CO_LEFT) {
			bb.motor.turnTo(bb.randAngle, bb.cruiseSpeed);
			bb.driveDirection = DD_ROTATECW;
		}
		else { // if ( bb.flagCoilFirstOutside == CO_RIGHT) {
			bb.motor.turnTo(-1 * bb.randAngle, bb.cruiseSpeed);
			bb.driveDirection = DD_ROTATECC;
		}
	}


	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (getTimeInNode() > 10000) {
			errorHandler.setError("TRotatePer  too long in state\r\n");
		}
		else if (bb.motor.isPositionReached()) {
			return BH_SUCCESS;
		}

		return BH_RUNNING;
	}

	virtual void onTerminate(NodeStatus status, Blackboard& bb) {

		if(status != BH_ABORTED) {

		}

		bb.motor.startDistanceMeasurementTraveled();

	}
};

*/




#endif
