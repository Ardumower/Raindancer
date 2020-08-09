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

#ifndef BH_BUMPER_H
#define BH_BUMPER_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "BehaviourTree.h"
#include "config.h"
#include "UseServices.h"


class TMotorStopFast : public Action {
private:

public:

	TMotorStopFast() {}

	virtual void onInitialize(Blackboard& bb) {
		srvMotor.enableFastStopRamping();
		srvMotor.stopCLC();
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (getTimeInNode() > 10000) {
			errorHandler.setError(F("!03,motorStop too long in state\r\n"));
		}

		if (srvMotor.isCLCStopped()) {
			//debug->printf("onUpdate enableFastStopRamping()\r\n");
			//bb.motor.enableFastStopRamping();
			return BH_SUCCESS;
		}

		return BH_RUNNING;
	}

	virtual void onTerminate(NodeStatus status, Blackboard& bb) {
		if (status != BH_ABORTED){
		  srvMotor.enableDefaultRamping();
		}
	}
};



class THardstop : public Action {
private:

public:

	THardstop() {}

	virtual void onInitialize(Blackboard& bb) {
		srvMotor.hardStop();
	}


	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (srvMotor.isCLCStopped()) { // Warten bis motor gestoppt
			return BH_SUCCESS;
		}

		return BH_RUNNING;
	}
};

class TSetflagBumperActivatedLR : public Action    // Each task will be a class (derived from Node of course).
{
private:

public:

	TSetflagBumperActivatedLR() {}

	virtual void onInitialize(Blackboard& bb) {
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (srvBumperSensor.isBumperActivatedLeft() && srvBumperSensor.isBumperActivatedRight()) {
			bb.history0.bumperActivated = BUM_BOTH;
			if (srvBumperSensor.flagShowBumper) {
				errorHandler.setInfo(F("!03,TSetflagBumperActivatedLR bb.history0.bumperActivated = BUM_BOTH;\r\n"));
			}
		}
		else if (srvBumperSensor.isBumperActivatedLeft()) {
			bb.history0.bumperActivated = BUM_LEFT;
			if (srvBumperSensor.flagShowBumper) {
				errorHandler.setInfo(F("!03,TSetflagBumperActivatedLR bb.history0.bumperActivated = BUM_LEFT;\r\n"));
			}
		}
		else if (srvBumperSensor.isBumperActivatedRight()) {
			bb.history0.bumperActivated = BUM_RIGHT;
			if (srvBumperSensor.flagShowBumper) {
				errorHandler.setInfo(F("!03,TSetflagBumperActivatedLR bb.history0.bumperActivated = BUM_RIGHT;\r\n"));
			}
		}
		else if (srvBumperSensor.isBumperDuinoActivated()) {
			bb.history0.bumperActivated = BUM_DUINO;
			if (srvBumperSensor.flagShowBumper) {
				errorHandler.setInfo(F("!03,TSetflagBumperActivatedLR bb.history0.bumperActivated = BUM_DUINO;\r\n"));
			}
		}
		return BH_SUCCESS;
	}


};

class TBumperDriveBack : public Action {
private:
	long weg;
	
public:

	TBumperDriveBack() {}

	virtual void onInitialize(Blackboard& bb) {
		THistory hist;

		float  distance,d2,d3;
		distance = CONF_BUMPER_REVERSE_CM;
		bb.histGetThreeLastForwardDistances(distance, d2, d3);

		if (distance < CONF_BUMPER_REVERSE_CM) {
			if (distance < 10.0f) { // drive 5cm if distance is near 0
				distance = 10.0f;
			}
			else {
				distance = distance;
			}
		}
		else {
			distance = CONF_BUMPER_REVERSE_CM;
		}

		hist = bb.getInitialisedHistoryEntry();
		hist.cruiseSpeed = bb.CRUISE_SPEED_LOW;
		hist.driveDirection = DD_REVERSE; //That diables the bumper branch
		hist.distanceSoll = -distance;

		//errorHandler.setInfo(F("!03,TBumperDriveBack ADD TO HISTORY###########\r\n"));
		bb.addHistoryEntry(hist);
		srvMotor.rotateCM(-distance, bb.CRUISE_SPEED_LOW); // x cm zurueckfahren
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {



		if (srvMotor.isPositionReached()) {
			bb.history0.restored = true;
			return BH_SUCCESS;
		}

		if (getTimeInNode() > 8000) {
			errorHandler.setError(F("!03,TBumperDriveBack too long in state\r\n"));
		}

		return BH_RUNNING;


	}
};

/*
class TRestoreBumperRotation : public Action {
private:

public:

	TRestoreBumperRotation() {}

	virtual void onInitialize(Blackboard& bb) {


		THistory hist = bb.getInitialisedHistoryEntry();
		hist.cruiseSpeed = bb.CRUISE_SPEED_OBSTACLE;;
		float distance;

		distance = fabs(bb.history0.distanceIst) - fabs(bb.history[1].distanceIst);

		switch (bb.history0.driveDirection) { 
		case DD_ROTATECW:
			hist.driveDirection = DD_ROTATECC;
			hist.distanceSoll = bb.history0.distanceIst - bb.history[1].distanceIst;
			errorHandler.setInfo(F("!03,TRestoreBumperRotation escape from DD_ROTATECW [0]%f [1]%f\r\n"), bb.history0.distanceIst, bb.history[1].distanceIst);
		
			srvMotor.turnTo(-fabs(hist.distanceSoll), bb.history0.cruiseSpeed);
			bb.numberOfRotations++;
			break;

		case DD_ROTATECC:
			hist.driveDirection = DD_ROTATECW;
			hist.distanceSoll = bb.history0.distanceIst - bb.history[1].distanceIst;
			srvMotor.turnTo(fabs(hist.distanceSoll), bb.history0.cruiseSpeed);
			bb.numberOfRotations++;
			errorHandler.setInfo(F("!03,TRestoreBumperRotation escape from DD_ROTATECC [0]%f [1]%f\r\n"), bb.history0.distanceIst, bb.history[1].distanceIst);
			break;

		default:
			sprintf(errorHandler.msg, "!03,TRestoreBumperRotation driveDirection not found: %s", enuDriveDirectionString[bb.history0.driveDirection]);
			errorHandler.setError();
			break;
		}

		bb.history0.restored = true;
		bb.addHistoryEntry(hist);
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (srvMotor.isPositionReached()) {
			return BH_SUCCESS;
		}

		if (getTimeInNode() > 10000) {
			errorHandler.setError(F("!03,TRestoreBumperRotation too long in state\r\n"));
		}

		return BH_RUNNING;
	}
};
*/
class TCalcBumpAngleOpposDir : public Action {
private:

public:

	TCalcBumpAngleOpposDir() {}

	virtual void onInitialize(Blackboard& bb) {
	

		THistory hist = bb.getInitialisedHistoryEntry();
		hist.cruiseSpeed = bb.CRUISE_SPEED_OBSTACLE;;

		switch (bb.history0.driveDirection) {

            // if bumped forward, the robot must driven back before calling TCalcBumpAngle
		case DD_FORWARD:
		case DD_REVERSE:
			hist.driveDirection = bb.historyGetLastRotateDirection();
			if (hist.driveDirection == DD_ROTATECW) {
				hist.distanceSoll = myRandom(90, 110);
			}
			else {
				hist.distanceSoll = -myRandom(90, 110);
			}
			errorHandler.setInfo(F("!03,TCalcBumpAngleOpposDir escape from DD_FORWARD\r\n"));
			break;

		case DD_ROTATECW:
			hist.driveDirection = DD_ROTATECC;
			//hist.distanceSoll = -(fabs(bb.history0.distanceIst) + myRandom(90, 110));
			hist.distanceSoll = -myRandom(90, 110);
			errorHandler.setInfo(F("!03,TCalcBumpAngleOpposDir escape from DD_ROTATECW\r\n"));
			break;

		case DD_ROTATECC:
			hist.driveDirection = DD_ROTATECW;
			//hist.distanceSoll = (fabs(bb.history0.distanceIst) + myRandom(90, 110));
			hist.distanceSoll = myRandom(90, 110);
			errorHandler.setInfo(F("!03,TCalcBumpAngleOpposDir escape from DD_ROTATECC\r\n"));
			break;

		default:
			sprintf(errorHandler.msg, "!03,TCalcBumpAngleOpposDir driveDirection not found: %s", enuDriveDirectionString[bb.history0.driveDirection]);
			errorHandler.setError();
			break;
		}

		bb.addHistoryEntry(hist);
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		return BH_SUCCESS;
	}
};

class TCalcBumpAngleSameDirection : public Action {
private:

public:

	TCalcBumpAngleSameDirection() {}

	virtual void onInitialize(Blackboard& bb) {


		THistory hist = bb.getInitialisedHistoryEntry();
		hist.cruiseSpeed = bb.CRUISE_SPEED_OBSTACLE;;

		switch (bb.history0.driveDirection) {
		case DD_FORWARD:
		case DD_REVERSE:
			hist.driveDirection = bb.historyGetLastRotateDirection();
			if (hist.driveDirection == DD_ROTATECW) {
				hist.distanceSoll = myRandom(90, 110);
			}
			else {
				hist.distanceSoll = -myRandom(90, 110);
			}
			errorHandler.setInfo(F("!03,TCalcBumpAngleSameDirection escape from DD_FORWARD\r\n"));
			break;

		case DD_ROTATECC:
			hist.driveDirection = DD_ROTATECC;
			//hist.distanceSoll = -(fabs(bb.history0.distanceIst) + myRandom(90, 110));
			hist.distanceSoll = -myRandom(90, 110);
			errorHandler.setInfo(F("!03,TCalcBumpAngleSameDirection escape from DD_ROTATECW\r\n"));
			break;

		case DD_ROTATECW:
			hist.driveDirection = DD_ROTATECW;
			//hist.distanceSoll = (fabs(bb.history0.distanceIst) + myRandom(90, 110));
			hist.distanceSoll = myRandom(90, 110);
			errorHandler.setInfo(F("!03,TCalcBumpAngleSameDirection escape from DD_ROTATECC\r\n"));
			break;

		default:
			sprintf(errorHandler.msg, "!03,TCalcBumpAngleSameDirection driveDirection not found: %s", enuDriveDirectionString[bb.history0.driveDirection]);
			errorHandler.setError();
			break;
		}

		bb.addHistoryEntry(hist);
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		return BH_SUCCESS;
	}
};


class TFreeBumper : public Action {
private:
	int counter, counter1;
	unsigned long lastTimeCalled;
	int16_t cruiseSpeed;
public:

	TFreeBumper() :counter(0), counter1(0), lastTimeCalled(0) {}

	virtual void onInitialize(Blackboard& bb) {
		long distance;
		float  distanceDriven;
		distance = 5;
		counter = 0;
		cruiseSpeed = bb.CRUISE_SPEED_OBSTACLE;
		// If bumped after freeing in 10 sec again, then increase counter.
	  // If this happens 10 times, give out error. This means if robot is not drive 10sec without a bump 10 times, 
	  // then give out an error because the mower stucks maybe.
		// Abfangen, wenn bumperereignis wie bei AreaX und Perimetertracking nicht weiter bearbeitet wird. Dann faehrt robbi immer wieder gegen Hindernis. 
		if (millis() - lastTimeCalled < 10000) {
			counter1++;
			errorHandler.setInfo(F("freeBumper counter1++ at: %lu\r\n"), millis());
		}
		else {
			counter1 = 0;
		}

		if (counter1 > 10) {
			errorHandler.setError(F("!03,freeBumper counter1 > 10\r\n"));
		}



		switch (bb.history0.driveDirection) {
		case DD_FORWARD:

			distanceDriven = abs(bb.history0.distanceIst);
			if (distanceDriven < CONF_BUMPER_REVERSE_CM) {
				if (distanceDriven < 10.0f) { // drive 5cm if distance is near 0
					distanceDriven = -10.0f;
				}
				else {
					distanceDriven = -distanceDriven;
				}
				srvMotor.rotateCM(distanceDriven, cruiseSpeed);
			}
			else {
				srvMotor.rotateCM(-CONF_BUMPER_REVERSE_CM, cruiseSpeed);
				distanceDriven = -CONF_BUMPER_REVERSE_CM;
			}
			errorHandler.setInfo(F("!03,freeBumper escape from DD_FORWARD/OVERRUN\r\n"));
			break;

		case DD_ROTATECW:
			distance = bb.history0.distanceIst;
			if (distance < 1.0f) { // Bumper wurde aktiviert, ohne dass gefahren wurde=> Bumper steckt fest. 
				distance = 5.0f;
				errorHandler.setInfo(F("!03,freeBumper escape from DD_ROTATECW set distance=5.0\r\n"));
			}
			srvMotor.rotateCM(-distance, distance, cruiseSpeed, cruiseSpeed);

			errorHandler.setInfo(F("!03,freeBumper escape from DD_ROTATECW\r\n"));

			break;
		case DD_ROTATECC:
			distance = bb.history0.distanceIst;
			if (distance < 1.0f) { // Bumper wurde aktiviert, ohne dass gefahren wurde=> Bumper steckt fest. 
				distance = 5.0f;
				errorHandler.setInfo(F("!03,freeBumper escape from DD_ROTATECC set distance=5.0\r\n"));
			}
			srvMotor.rotateCM(distance, -distance, cruiseSpeed, cruiseSpeed);
			errorHandler.setInfo(F("!03,freeBumper escape from DD_ROTATECC\r\n"));
			break;

		default:
			sprintf(errorHandler.msg, "!03,TFreeBumper driveDirection not found: %s", enuDriveDirectionString[bb.history0.driveDirection]);
			errorHandler.setError();
			break;
		}
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		int buffer;

		if (getTimeInNode() > 10000) {
			errorHandler.setError(F("!03,freeBumper too long in state\r\n"));
		}

		if (getTimeInNode() > 3000) { // Nach 3 sek sollte bumper frei sein
			if (counter < 3) {
				if (srvBumperSensor.isBumperActivated()) {   //wurde bumper bei berfreiung wieder betÃ¤tigt? Dann wurde Motor auch hard gestoppt von sensor und man muss erneut aktiviert werden. Max. 3x
					sprintf(errorHandler.msg, "!03,FreeBumper Bumper bei Befreiung erneut betaetigt\r\n");
					errorHandler.setInfo();
					buffer = counter; // Save counter value in order it will be reset in onInitialize
					onInitialize(bb); // erneut Befreiungsrichtung ud Weg berrechnen
					counter = buffer;
					setTimeInNode(millis());
					counter++;
				}
			}
			if (counter >= 3) {
				errorHandler.setError(F("!03,freeBumper  counter >= 3\r\n"));
			}
		}



		switch (bb.history0.driveDirection) {
		case DD_FORWARD:
			//if(!bb.bumperSensor.isBumperActivated() &&  getTimeInNode() > 1000) {
			if (srvMotor.isPositionReached()) {
				errorHandler.setInfo(F("!03,freeBumper escape from DD_FORWARD/OVERRUN finished\r\n"));
				return BH_SUCCESS;
			}
			break;

		case DD_ROTATECW:
			if (srvMotor.isPositionReached()) {
				errorHandler.setInfo(F("!03,freeBumper escape from DD_ROTATECW finished\r\n"));
				return BH_SUCCESS;
			}
			break;
		case DD_ROTATECC:
			if (srvMotor.isPositionReached()) {
				errorHandler.setInfo(F("!03,freeBumper escape from DD_ROTATECC finished\r\n"));
				return BH_SUCCESS;
			}
			break;

		default:
			sprintf(errorHandler.msg, "!03,TFreeBumper driveDirection not found: %s", enuDriveDirectionString[bb.history0.driveDirection]);
			errorHandler.setError();
			break;
		}

		return BH_RUNNING;
	}


	virtual void onTerminate(NodeStatus status, Blackboard& bb) {


		lastTimeCalled = millis();
	}
};

class TConditionBumperNotFound : public Action    // Each task will be a class (derived from Node of course).
{
private:

public:

	TConditionBumperNotFound() {}



	virtual NodeStatus onUpdate(Blackboard& bb) {


		// When not useing zone recognition, then TOverRun changes the drivedirection to DD_FORWARD if both coils inside again and the branch is executed until
		// this node. Therfore this Node will be executed with the driveDirection==DD_Forward.
		// To prevent setting an error, we have to chech this here..
		if (bb.history0.driveDirection == DD_FORWARD) {
			return BH_SUCCESS;
		}
		errorHandler.setError(F("!03,TConditionPerimeterNotFound not found %s\r\n"), enuDriveDirectionString[bb.history0.driveDirection]);
		return BH_SUCCESS;
	}


};


#endif

