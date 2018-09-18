/*
Robotic Lawn Mower
Copyright (c) 2017 by Kai WÃ¼rtz

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

#ifndef BH_BUMPER_H
#define BH_BUMPER_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "BehaviourTree.h"
#include "config.h"



class TSetflagBumperActivatedLR : public Node    // Each task will be a class (derived from Node of course).
{
private:

public:

	TSetflagBumperActivatedLR() {}

	virtual void onInitialize(Blackboard& bb) {
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		// Latch which bumper was activated because we need it later in TSetArcFEO_ROT.
		// When TSetArcFEO_ROT is called, the bumper is already freed.

		if (bb.bumperSensor.isBumperActivatedLeft()) {
			bb.flagBumperActivatedLeft = true;
			if(bb.bumperSensor.flagShowBumper){
				errorHandler.setInfo(F("!03,TSetflagBumperActivatedLR bb.flagBumperActivatedLeft = true;\r\n"));
			}
		}
		else {
			bb.flagBumperActivatedLeft = false;
			if (bb.bumperSensor.flagShowBumper) {
				errorHandler.setInfo(F("!03,TSetflagBumperActivatedLR bb.flagBumperActivatedLeft = false;\r\n"));
			}
		}


		if (bb.bumperSensor.isBumperActivatedRight()) {
			bb.flagBumperActivatedRight = true;
			if (bb.bumperSensor.flagShowBumper) {
				errorHandler.setInfo(F("!03,TSetflagBumperActivatedLR bb.flagBumperActivatedRight = true;\r\n"));
			}
		}
		else {
			bb.flagBumperActivatedRight = false;
			if (bb.bumperSensor.flagShowBumper) {
				errorHandler.setInfo(F("!03,TSetflagBumperActivatedLR bb.flagBumperActivatedRight = false;\r\n"));
			}
		}

		return BH_SUCCESS;
	}


};


class THardstop : public Node
{
private:

public:

	THardstop() {}

	virtual void onInitialize(Blackboard& bb) {
		bb.motor.hardStop();
	}


	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (bb.motor.isCLCStopped()) { // Warten bis motor gestoppt
			return BH_SUCCESS;
		}

		return BH_RUNNING;
	}
};


class TFreeBumper : public Node
{
private:
	int counter, counter1;
	unsigned long lastTimeCalled;
public:

	TFreeBumper() :counter(0), counter1(0), lastTimeCalled(0) {}

	virtual void onInitialize(Blackboard& bb) {
		long distance;

		distance = 5;

		bb.motor.enableDefaultRamping();
		bb.cruiseSpeed = bb.CRUISE_SPEED_OBSTACLE;
		bb.flagPerimeterStateChanged = PSC_NONE;

		counter = 0;

		// If bumped after freeing in 10 sec again, then increase counter.
        // If this happens 10 times, give out error. This means if robot is not drive 10sec without a bump 10 times, 
        // then give out an error because the mower stucks maybe.
		// Abfangen, wenn bumperereignis wie bei AreaX und Perimetertracking nicht weiter bearbeitet wird. Dann faehrt robbi immer wieder gegen Hindernis. 
		if (millis() - lastTimeCalled < 10000) {
			counter1++;
            errorHandler.setInfo(F("freeBumper counter1++ at: %lu\r\n"),millis());
		}
		else {
			counter1 = 0;
		}

		if (counter1 > 10) {
			errorHandler.setError(F("!03,freeBumper counter1 > 10\r\n"));
		}

		if (bb.perimeterSensoren.isLeftOutside() || bb.perimeterSensoren.isRightOutside()) {
			bb.flagBumperOutsidePerActivated = true;
			bb.flagBumperInsidePerActivated = false;
			sprintf(errorHandler.msg, "!03,*flagBumperOutsidePerActivated=true dd was: %s\r\n", enuDriveDirectionString[bb.driveDirection]);
			errorHandler.setInfo();
		}
		else {
			bb.flagBumperOutsidePerActivated = false;
			bb.flagBumperInsidePerActivated = true;
			sprintf(errorHandler.msg, "!03,*flagBumperInsidePerActivated=true  dd was: %s\r\n", enuDriveDirectionString[bb.driveDirection]);
			errorHandler.setInfo();
		}



		// This is only filled here if it comes to problems. At the end of TFreeBumper you can write code to show the information to the console for debugging
		
		bb.hist.driveDirection = DD_FREEBUMPER;  // wird nur für History verwendet
		bb.hist.distanceDriven = 0;
		bb.hist.coilFirstOutside = CO_NONE;    // which coil was first outside jet
		bb.hist.rotAngleSoll = 0;
		bb.hist.rotAngleIst = 0;
		bb.hist.flagForceRotDirection = FRD_NONE;
		bb.hist.restored = true;

		switch (bb.driveDirection) {
		case DD_FORWARD:
		case DD_OVERRUN:


			bb.hist.distanceDriven = abs(bb.history[0].distanceDriven);
			if (bb.hist.distanceDriven < CONF_BUMPER_REVERSE_CM) {
				if (bb.hist.distanceDriven < 5.0f) { // drive 5cm if distance is near 0
					bb.hist.distanceDriven = -5.0f;
				}
				else {
					bb.hist.distanceDriven = -bb.hist.distanceDriven;
				}
				bb.motor.rotateCM(bb.hist.distanceDriven, bb.cruiseSpeed);
			}
			else {
				bb.motor.rotateCM(-CONF_BUMPER_REVERSE_CM, bb.cruiseSpeed);
				bb.hist.distanceDriven = -CONF_BUMPER_REVERSE_CM;
			}
			errorHandler.setInfo(F("!03,freeBumper escape from DD_FORWARD/OVERRUN\r\n"));
			break;

			//case DD_FORWARD20:

		case DD_REVERSE_INSIDE:  //NOT USED AT THE MOMENT
			bb.hist.distanceDriven = -bb.history[0].distanceDriven;

			bb.motor.rotateCM(bb.hist.distanceDriven, bb.cruiseSpeed);
			errorHandler.setInfo(F("!03,freeBumper escape from DD_REVERSE_INSIDE\r\n"));
			break;

		case DD_FORWARD_INSIDE:
			bb.motor.rotateCM(-5, bb.cruiseSpeed);
			bb.hist.distanceDriven = -5;
			errorHandler.setInfo(F("!03,freeBumper escape from DD_FORWARD_INSIDE\r\n"));
			break;

		case DD_LINE_FOLLOW:
			bb.motor.rotateCM(-CONF_BUMPER_REVERSE_CM, bb.cruiseSpeed);
			bb.hist.distanceDriven = -CONF_BUMPER_REVERSE_CM;
			errorHandler.setInfo(F("!03,freeBumper escape from DD_LINE_FOLLOW\r\n"));
			break;

		case DD_REVERSE_ESC_OBST:
			// Nur eingefügt um Kompilationsfehler zu vermeiden.
			// Wird in mseqPerimeterActive bearbeitet
			break;

		case DD_ROTATECW:
		case DD_ROTATECW1:
			distance = bb.motor.getAngleRotatedDistanceCM();
			if (distance < 1.0f) { // Bumper wurde aktiviert, ohne dass gefahren wurde=> Bumper steckt fest. 
				distance = 5.0f;
				errorHandler.setInfo(F("!03,freeBumper escape from DD_ROTATECW set distance=5.0\r\n"));
			}
			bb.motor.rotateCM(-distance, distance, bb.cruiseSpeed, bb.cruiseSpeed);
			bb.hist.distanceDriven = -distance;
			bb.hist.flagForceRotDirection = FRD_CC;
			errorHandler.setInfo(F("!03,freeBumper escape from DD_ROTATECW\r\n"));

			break;
		case DD_ROTATECC:
		case DD_ROTATECC1:
			distance = bb.motor.getAngleRotatedDistanceCM();
			if (distance < 1.0f) { // Bumper wurde aktiviert, ohne dass gefahren wurde=> Bumper steckt fest. 
				distance = 5.0f;
				errorHandler.setInfo(F("!03,freeBumper escape from DD_ROTATECC set distance=5.0\r\n"));
			}
			bb.motor.rotateCM(distance, -distance, bb.cruiseSpeed, bb.cruiseSpeed);
			bb.hist.distanceDriven = distance;
			bb.hist.flagForceRotDirection = FRD_CW;
			errorHandler.setInfo(F("!03,freeBumper escape from DD_ROTATECC\r\n"));
			break;

		case DD_FEOROTATECW:
		case DD_FEOROTATECW1:
		case DD_FEOROTATECW2:
			distance = bb.motor.getAngleRotatedDistanceCM();
			if (distance < 1.0f) { // Bumper wurde aktiviert, ohne dass gefahren wurde=> Bumper steckt fest. 
				distance = 5.0f;
				errorHandler.setInfo(F("!03,freeBumper escape from DD_FEOROTATECW,1,2 set distance=5.0\r\n"));
			}
			bb.motor.rotateCM(-distance, distance, bb.cruiseSpeed, bb.cruiseSpeed);
			bb.hist.distanceDriven = -distance;
			bb.hist.flagForceRotDirection = FRD_CC;
			errorHandler.setInfo(F("!03,freeBumper escape from DD_FEOROTATECW,1,2\r\n"));
			break;

		case DD_FEOROTATECC:
		case DD_FEOROTATECC1:
		case DD_FEOROTATECC2:

			distance = bb.motor.getAngleRotatedDistanceCM();
			if (distance < 1.0f) { // Bumper wurde aktiviert, ohne dass gefahren wurde=> Bumper steckt fest. 
				distance = 5.0f;
				errorHandler.setInfo(F("!03,freeBumper escape from DD_FEOROTATECC,1,2 set distance=5.0\r\n"));
			}
			bb.motor.rotateCM(distance, -distance, bb.cruiseSpeed, bb.cruiseSpeed);
			bb.hist.distanceDriven = distance;
			bb.hist.flagForceRotDirection = FRD_CW;
			errorHandler.setInfo(F("!03,freeBumper 03,freeBumper escape from DD_FEOROTATECC,1,2\r\n"));
			break;

		case DD_SPIRAL_CW:
			bb.cruiseSpeed = bb.CRUISE_SPEED_MEDIUM;
			bb.motor.L->setSpeed(-1 * bb.CRUISE_SPEED_MEDIUM);
			bb.motor.R->setSpeed(-5);
			bb.hist.distanceDriven = -CONF_BUMPER_REVERSE_CM; // Stimmt nicht aktuell
			errorHandler.setInfo(F("!03,freeBumper 03,freeBumper escape from DD_SPIRAL_CW\n"));
			break;

		default:
			sprintf(errorHandler.msg, "!03,TFreeBumper driveDirection not found: %s", enuDriveDirectionString[bb.driveDirection]);
			errorHandler.setError();
			break;
		}

		// Overwrite the bb.hist structure. 
		// Here we now delete the restored history entry. But in TselEscapeAlgorithm we need the value bb.history[0].distanceDriven. 
		// Therefore we need to save the deleted history item to bb.hist.
		bb.hist = bb.history[0];
		if (bb.driveDirection != DD_REVERSE_ESC_OBST) {
			bb.deleteLastHistoryEntry();
			//	bb.addHistoryEntry(h.driveDirection, h.distanceDriven, h.rotAngleSoll, h.rotAngleIst, h.flagForceRotDirection, h.coilFirstOutside);
			//	bb.markLastHistoryEntryAsRestored();
		}


	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		int buffer;

		if (getTimeInNode() > 10000) {
			errorHandler.setError(F("!03,freeBumper too long in state\r\n"));
		}

		if (getTimeInNode() > 3000) { // Nach 3 sek sollte bumper frei sein
			if (counter < 3) {
				if (bb.bumperSensor.isBumperActivated()) {   //wurde bumper bei berfreiung wieder betÃ¤tigt? Dann wurde Motor auch hard gestoppt von sensor und man muss erneut aktiviert werden. Max. 3x
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



		switch (bb.driveDirection) {
		case DD_FORWARD:
		case DD_OVERRUN:
			//if(!bb.bumperSensor.isBumperActivated() &&  getTimeInNode() > 1000) {
			if (bb.motor.isPositionReached()) {
				errorHandler.setInfo(F("!03,freeBumper escape from DD_FORWARD/OVERRUN finished\r\n"));
				return BH_SUCCESS;
			}
			break;
			//case DD_FORWARD20:    
		case DD_REVERSE_INSIDE:
			if (bb.motor.isPositionReached()) {
				errorHandler.setInfo(F("!03,freeBumper escape from DD_REVERSE_INSIDE finished\r\n"));
				return BH_SUCCESS;
			}
			break;
		case DD_FORWARD_INSIDE:
			if (bb.motor.isPositionReached()) {
				errorHandler.setInfo(F("!03,freeBumper escape from DD_FORWARD_INSIDE finished\r\n"));
				return BH_SUCCESS;
			}
			break;
		case DD_LINE_FOLLOW:
			if (bb.motor.isPositionReached()) {
				errorHandler.setInfo(F("!03,freeBumper escape from DD_LINE_FOLLOW finished\r\n"));
				return BH_SUCCESS;
			}
			break;

		case DD_REVERSE_ESC_OBST:
			// Nur eingefügt um Kompilationsfehler zu vermeiden.
			// Wird in mseqPerimeterActive bearbeitetrn BH_SUCCESS;
			//} 
			break;


		case DD_ROTATECW:
		case DD_ROTATECW1:
			if (bb.motor.isPositionReached()) {
				errorHandler.setInfo(F("!03,freeBumper escape from DD_ROTATECW finished\r\n"));
				return BH_SUCCESS;
			}
			break;
		case DD_ROTATECC:
		case DD_ROTATECC1:
			if (bb.motor.isPositionReached()) {
				errorHandler.setInfo(F("!03,freeBumper escape from DD_ROTATECC finished\r\n"));
				return BH_SUCCESS;
			}
			break;

		case DD_FEOROTATECW:
		case DD_FEOROTATECW1:
		case DD_FEOROTATECW2:
			if (bb.motor.isPositionReached()) {
				errorHandler.setInfo(F("!03,freeBumper escape from DD_FEOROTATECW finished\r\n"));
				return BH_SUCCESS;
			}
			break;
		case DD_FEOROTATECC:
		case DD_FEOROTATECC1:
		case DD_FEOROTATECC2:
			if (bb.motor.isPositionReached()) {
				errorHandler.setInfo(F("!03,freeBumper escape from DD_FEOROTATECC finished\r\n"));
				return BH_SUCCESS;
			}
			break;
		case DD_SPIRAL_CW:
			if (!bb.bumperSensor.isBumperActivated() && getTimeInNode() > 1500) {
				errorHandler.setInfo(F("!03,freeBumper escape from DD_SPIRAL_CW finished\r\n"));
				return BH_SUCCESS;
			}
			break;

		default:
			sprintf(errorHandler.msg, "!03,TFreeBumper driveDirection not found: %s", enuDriveDirectionString[bb.driveDirection]);
			errorHandler.setError();
			break;
		}

		return BH_RUNNING;
	}


	virtual void onTerminate(NodeStatus status, Blackboard& bb) {
		if (status != BH_ABORTED) {
			// Hat sich perimeterstatus geÃ¤ndert. von innen nach auÃŸen oder anderherum?
			if ((bb.perimeterSensoren.isLeftOutside() || bb.perimeterSensoren.isRightOutside()) && bb.flagBumperInsidePerActivated) {
				bb.flagPerimeterStateChanged = PSC_IO;
				sprintf(errorHandler.msg, "!03,flagPerimeterStateChanged i->o = true\r\n");
				errorHandler.setInfo();
			}
			else if ((bb.perimeterSensoren.isLeftInside() && bb.perimeterSensoren.isRightInside()) && bb.flagBumperOutsidePerActivated) {
				bb.flagPerimeterStateChanged = PSC_OI;
				sprintf(errorHandler.msg, "!03,flagPerimeterStateChanged o->i  = true\r\n");
				errorHandler.setInfo();
			}


		}

		lastTimeCalled = millis();
	}
};



class TFreeBumper2 : public Node
{
private:
	int counter, counter1;
	unsigned long lastTimeCalled;
public:

	TFreeBumper2() :counter(0), counter1(0), lastTimeCalled(0) {}

	virtual void onInitialize(Blackboard& bb) {
		long distance;

		distance = 5;

		bb.motor.enableDefaultRamping();
		bb.cruiseSpeed = bb.CRUISE_SPEED_OBSTACLE;
		bb.flagPerimeterStateChanged = PSC_NONE;

		counter = 0;

		// Wenn innerhalb von jeweils 50Sek  5x bumped. Fehler ausgeben.
		// Abfangen, wenn bumperereignis wie bei AreaX und Perimetertracking nicht weiter bearbeitet wird. Dann faehrt robbi immer wieder gegen Hindernis. 
		if (millis() - lastTimeCalled < 10000) {
			counter1++;
		}
		else {
			counter1 = 0;
		}

		if (counter1 > 5) {
			errorHandler.setError(F("!03,freeBumper counter1 > 5\r\n"));
		}

		if (bb.perimeterSensoren.isLeftOutside() /*&& bb.perimeterSensoren.isRightOutside()*/) {
			bb.flagBumperOutsidePerActivated = true;
			bb.flagBumperInsidePerActivated = false;
			sprintf(errorHandler.msg, "!03,*flagBumperOutsidePerActivated=true dd was: %s\r\n", enuDriveDirectionString[bb.driveDirection]);
			errorHandler.setInfo();
		}
		else {
			bb.flagBumperOutsidePerActivated = false;
			bb.flagBumperInsidePerActivated = true;
			sprintf(errorHandler.msg, "!03,*flagBumperInsidePerActivated=true  dd was: %s\r\n", enuDriveDirectionString[bb.driveDirection]);
			errorHandler.setInfo();
		}


		// This is only filled here if it comes to problems. At the end of TFreeBumper you can write code to show the information to the console for debugging
		bb.hist.driveDirection = DD_FREEBUMPER;  // wird nur für History verwendet
		bb.hist.distanceDriven = 0;
		bb.hist.coilFirstOutside = CO_NONE;    // which coil was first outside jet
		bb.hist.rotAngleSoll = 0;
		bb.hist.rotAngleIst = 0;
		bb.hist.flagForceRotDirection = FRD_NONE;
		bb.hist.restored = true;


		switch (bb.driveDirection) {

		case DD_REVERSE_ESC_OBST:
			// Nur eingefügt um Kompilationsfehler zu vermeiden.
			// Wird in mseqPerimeterActive bearbeitetrn BH_SUCCESS;
			//} 
			break;
		case DD_ROTATECC:
			distance = bb.motor.getAngleRotatedDistanceCM();
			if (distance < 1.0f) { // Bumper wurde aktiviert, ohne dass gefahren wurde=> Bumper steckt fest. 
				distance = 5.0f;
				errorHandler.setInfo(F("!03,TFreeBumper2 set distance=5.0\r\n"));
			}
			bb.motor.rotateCM(distance, -distance, bb.cruiseSpeed, bb.cruiseSpeed);
			bb.hist.distanceDriven = distance;
			bb.hist.flagForceRotDirection = FRD_CW;
			errorHandler.setInfo(F("!03,TFreeBumper2 escape from DD_ROTATECC\r\n"));
			break;
		case DD_FORWARD:
		case DD_LINE_FOLLOW:
			bb.motor.rotateCM(-CONF_BUMPER_REVERSE_CM, bb.cruiseSpeed);
			bb.hist.distanceDriven = -CONF_BUMPER_REVERSE_CM;
			errorHandler.setInfo(F("!03,TFreeBumper2 escape from DD_LINE_FOLLOW\r\n"));
			break;

		default:
			errorHandler.setError(F("!03,TFreeBumper2 driveDirection not found: %s"), enuDriveDirectionString[bb.driveDirection]);
			break;
		}

		// Overwrite the bb.hist structure. 
		// Here we now delete the restored history entry. But in TselEscapeAlgorithm we need the value bb.history[0].distanceDriven. 
		// Therefore we need to save the deleted history item to bb.hist.
		bb.hist = bb.history[0];
		if (bb.driveDirection != DD_REVERSE_ESC_OBST) {
			bb.deleteLastHistoryEntry();
			//bb.addHistoryEntry(h.driveDirection, h.distanceDriven, h.rotAngleSoll, h.rotAngleIst, h.flagForceRotDirection, h.coilFirstOutside);
			//bb.markLastHistoryEntryAsRestored();
		}


	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		int buffer;

		if (getTimeInNode() > 10000) {
			errorHandler.setError(F("!03,TFreeBumper2 too long in state\r\n"));
		}

		if (getTimeInNode() > 3000) { // Nach 3 sek sollte bumper frei sein
			if (counter < 3) {
				if (bb.bumperSensor.isBumperActivated()) {   //wurde bumper bei berfreiung wieder betÃ¤tigt? Dann wurde Motor auch hard gestoppt von sensor und man muss erneut aktiviert werden. Max. 3x
					errorHandler.setInfo(F("!03,TFreeBumper2 Bumper2 bei Befreiung erneut betaetigt\r\n"));
					buffer = counter; // Save counter value in order it will be reset in onInitialize
					onInitialize(bb); // erneut Befreiungsrichtung ud Weg berrechnen
					counter = buffer;
					setTimeInNode(millis());
					counter++;
				}
			}
			if (counter >= 3) {
				errorHandler.setError(F("!03,TFreeBumper2  counter >= 3\r\n"));
			}
		}



		switch (bb.driveDirection) {

		case DD_REVERSE_ESC_OBST:
			// Nur eingefügt um Kompilationsfehler zu vermeiden.
			// Wird in mseqPerimeterActive bearbeitetrn BH_SUCCESS;
			//} 
			break;
		case DD_ROTATECC:
			if (bb.motor.isPositionReached()) {
				bb.flagEnableSecondReverse = true;
				errorHandler.setInfo(F("!03,TFreeBumper2 escape from DD_ROTATECC finished\r\n"));
				// Befire we rotate CC again, we habe to drive back
				return BH_SUCCESS;
			}
			break;

		case DD_FORWARD:
		case DD_LINE_FOLLOW:
			if (bb.flagBumperOutsidePerActivated == false) {
				if (bb.perimeterSensoren.isLeftOutside() && bb.perimeterSensoren.isRightOutside()) {
					errorHandler.setInfo(F("!03,TFreeBumper2 both coils outside\r\n"));
					bb.motor.stopPC();
					return BH_SUCCESS;
				}
			}
			if (bb.motor.isPositionReached()) {
				errorHandler.setInfo(F("!03,TFreeBumper2  escape from DD_LINE_FOLLOW finished\r\n"));
				return BH_SUCCESS;
			}
			break;



		default:
			errorHandler.setError(F("!03,TFreeBumper2 driveDirection not found: %s\r\n"), enuDriveDirectionString[bb.driveDirection]);
			return BH_SUCCESS;
			break;
		}

		return BH_RUNNING;
	}


	virtual void onTerminate(NodeStatus status, Blackboard& bb) {
		if (status != BH_ABORTED) {
			bb.flagRotateAtPer = true;
			bb.flagDriveCurve = true;
			/*
			// Hat sich perimeterstatus geÃ¤ndert. von innen nach auÃŸen oder anderherum?
			if ((bb.perimeterSensoren.isLeftOutside() || bb.perimeterSensoren.isRightOutside()) && bb.flagBumperInsidePerActivated) {
				bb.flagPerimeterStateChanged = PSC_IO;
				sprintf(errorHandler.msg, "!03,flagPerimeterStateChanged i->o = true\r\n");
				errorHandler.setInfo();
			}
			else if ((bb.perimeterSensoren.isLeftInside() && bb.perimeterSensoren.isRightInside()) && bb.flagBumperOutsidePerActivated) {
				bb.flagPerimeterStateChanged = PSC_OI;
				sprintf(errorHandler.msg, "!03,flagPerimeterStateChanged o->i  = true\r\n");
				errorHandler.setInfo();
			}
			*/


		}

		lastTimeCalled = millis();
	}
};



class TselEscapeAlgorithm : public Node
{
private:
	int counter;
	const char* text;
public:

	TselEscapeAlgorithm() :counter(0), text("!03,EscapeAlgorithm %s\r\n") {}


	virtual NodeStatus onUpdate(Blackboard& bb) {

		//if(bb.flagPerimeterStateChanged == PSC_NONE && bb.flagBumperInsidePerActivated) {
		if (bb.flagBumperInsidePerActivated) {
			switch (bb.driveDirection) {
			case DD_FORWARD:
				// Check first for sequences to determine if revese history should be executed
				if (bb.hist.distanceDriven < 30) {
					if ( //bb.history[0].driveDirection == DD_FORWARD && was deleted in the drive back above
						(bb.history[0].driveDirection == DD_ROTATECC || bb.history[0].driveDirection == DD_ROTATECW) &&
						bb.history[1].driveDirection == DD_REVERSE_ESC_OBST &&
						bb.history[2].driveDirection == DD_OVERRUN &&
						bb.history[3].driveDirection == DD_FORWARD
						) {
						errorHandler.setInfo(F("!03,TselEscapeAlgorithm DD_FORWARD sequence 1 found\r\n"));
						if (bb.history[3].distanceDriven > 100) {
							bb.history[3].distanceDriven = 100;
						}

						bb.flagEnableRestoreHistory = true;
						bb.numberToRestoreHist = 4;
						// in the next tick, restore hostiry is called.

						// After restore history, mower should rotate.
						// therefore use the following lines below will be called also after the if statment.
						//bb.flagBumperInsidePerActivated = true;
						//bb.flagEscabeObstacleConFlag = FEO_ROT;

					}
					else if ( //bb.history[0].driveDirection == DD_FORWARD && was deleted in the drive back above
						(bb.history[0].driveDirection == DD_ROTATECC || bb.history[0].driveDirection == DD_ROTATECW) &&
						bb.history[1].driveDirection == DD_OVERRUN &&
						bb.history[2].driveDirection == DD_FORWARD
						) {
						errorHandler.setInfo(F("!03,TselEscapeAlgorithm DD_FORWARD sequence 2 found\r\n"));
						if (bb.history[2].distanceDriven > 100) {
							bb.history[2].distanceDriven = 100;
						}

						bb.flagEnableRestoreHistory = true;
						bb.numberToRestoreHist = 3;
						// in the next tick, restore hostiry is called.

						// After restore history, mower should rotate.
						// therefore use the following lines below will be called also after the if statment.
						//bb.flagBumperInsidePerActivated = true;
						//bb.flagEscabeObstacleConFlag = FEO_ROT;

					}
				 }
				// Robbi ist nun 20cm zurückgefahren. DD_REVERSE_ESC_OBST wird gesetzt, falls rückwärts rausgefahren. Wird in mseqPerimeterActive bearbeitet
				// ACHTUNG wenn coil immer noch draußen ist, wird mseqPRLCO/mseqPRRCO aufgerufen, welche den robbi wieder inside fahren.
				// Dann ist aber immer noch bb.flagEscabeObstacleConFlag = FEO_BACK180 gesetzt, so dass danach in selEscabeObstacle1 gedreht wird.
				bb.driveDirection = DD_REVERSE_ESC_OBST;
				bb.flagEscabeObstacleConFlag = FEO_ROT;
				sprintf(errorHandler.msg, text, enuFlagEscabeObstacleConFlagString[bb.flagEscabeObstacleConFlag]);
				errorHandler.setInfo();
				break;
			case DD_OVERRUN://Kann hier normal nicht auftreten
			case DD_REVERSE_INSIDE:
			case DD_SPIRAL_CW:

				// Robbi ist nun 20cm zurückgefahren. DD_REVERSE_ESC_OBST wird gesetzt, falls rückwärts rausgefahren. Wird in mseqPerimeterActive bearbeitet
				// ACHTUNG wenn coil immer noch draußen ist, wird mseqPRLCO/mseqPRRCO aufgerufen, welche den robbi wieder inside fahren.
				// Dann ist aber immer noch bb.flagEscabeObstacleConFlag = FEO_BACK180 gesetzt, so dass danach in selEscabeObstacle1 gedreht wird.
				bb.driveDirection = DD_REVERSE_ESC_OBST;
				bb.flagEscabeObstacleConFlag = FEO_ROT;
				sprintf(errorHandler.msg, text, enuFlagEscabeObstacleConFlagString[bb.flagEscabeObstacleConFlag]);
				errorHandler.setInfo();
				break;
			case DD_FORWARD_INSIDE: //Kann hier nicht auftreten
				errorHandler.setError(F("!03,TselEscapeAlgorithm DD_FORWARD_INSIDE in Bumper innerhalb Perimeter aktiviert 1\r\n"));
				break;
			case DD_ROTATECW:
				bb.flagEscabeObstacleConFlag = FEO_ROTCC1;
				//bb.flagForceRotateDirection = FRD_CC;
				// If  Bumper was inside activated and now coils are outside, simulate bumper outside activated, because we don't want to call mseqPerimeterActive
				if (bb.flagPerimeterStateChanged == PSC_IO) {
					bb.flagEscabeObstacleConFlag = FEO_ROTCW2;
					bb.flagBumperInsidePerActivated = false;
					bb.flagBumperOutsidePerActivated = true;
					errorHandler.setInfo(F("!03,TselEscapeAlgorithm changed: bb.flagBumperOutsidePerActivated = true;\r\n"));

				}
				sprintf(errorHandler.msg, text, enuFlagEscabeObstacleConFlagString[bb.flagEscabeObstacleConFlag]);
				errorHandler.setInfo();
				break;
			case DD_ROTATECC:
				bb.flagEscabeObstacleConFlag = FEO_ROTCW1;
				//bb.flagForceRotateDirection = FRD_CW;
				// If  Bumper was inside activated and now coils are outside, simulate bumper outside activated, because we don't want to call mseqPerimeterActive
				if (bb.flagPerimeterStateChanged == PSC_IO) {
					bb.flagEscabeObstacleConFlag = FEO_ROTCC2;
					bb.flagBumperInsidePerActivated = false;
					bb.flagBumperOutsidePerActivated = true;
					errorHandler.setInfo(F("!03,TselEscapeAlgorithm changed: bb.flagBumperOutsidePerActivated = true;\r\n"));
				}
				sprintf(errorHandler.msg, text, enuFlagEscabeObstacleConFlagString[bb.flagEscabeObstacleConFlag]);
				errorHandler.setInfo();
				break;

			case DD_ROTATECW1:
				bb.flagEscabeObstacleConFlag = FEO_ROTCW2;
				sprintf(errorHandler.msg, text, enuFlagEscabeObstacleConFlagString[bb.flagEscabeObstacleConFlag]);
				errorHandler.setInfo();
				break;
			case DD_ROTATECC1:
				bb.flagEscabeObstacleConFlag = FEO_ROTCC2;
				sprintf(errorHandler.msg, text, enuFlagEscabeObstacleConFlagString[bb.flagEscabeObstacleConFlag]);
				errorHandler.setInfo();
				break;

			case DD_FEOROTATECW:
				bb.flagEscabeObstacleConFlag = FEO_ROTCC1;
				// If  Bumper was inside activated and now coils are outside, simulate bumper outside activated, because we don't want to call mseqPerimeterActive
				if (bb.flagPerimeterStateChanged == PSC_IO) {
					bb.flagEscabeObstacleConFlag = FEO_ROTCW2;
					bb.flagBumperInsidePerActivated = false;
					bb.flagBumperOutsidePerActivated = true;
					errorHandler.setInfo(F("!03,TselEscapeAlgorithm changed: bb.flagBumperOutsidePerActivated = true;\r\n"));

				}
				sprintf(errorHandler.msg, text, enuFlagEscabeObstacleConFlagString[bb.flagEscabeObstacleConFlag]);
				errorHandler.setInfo();
				break;
			case DD_FEOROTATECC:
				bb.flagEscabeObstacleConFlag = FEO_ROTCW1;

				// If  Bumper was inside activated and now coils are outside, simulate bumper outside activated, because we don't want to call mseqPerimeterActive
				if (bb.flagPerimeterStateChanged == PSC_IO) {
					bb.flagEscabeObstacleConFlag = FEO_ROTCC2;
					bb.flagBumperInsidePerActivated = false;
					bb.flagBumperOutsidePerActivated = true;
					errorHandler.setInfo(F("!03,TselEscapeAlgorithm changed: bb.flagBumperOutsidePerActivated = true;\r\n"));

				}
				sprintf(errorHandler.msg, text, enuFlagEscabeObstacleConFlagString[bb.flagEscabeObstacleConFlag]);
				errorHandler.setInfo();
				break;

			case DD_FEOROTATECW1:
				bb.flagEscabeObstacleConFlag = FEO_ROTCC2;
				// If  Bumper was inside activated and now coils are outside, simulate bumper outside activated, because we don't want to call mseqPerimeterActive
				if (bb.flagPerimeterStateChanged == PSC_IO) {
					bb.flagEscabeObstacleConFlag = FEO_ROTCW2;
					bb.flagBumperInsidePerActivated = false;
					bb.flagBumperOutsidePerActivated = true;
					errorHandler.setInfo(F("!03,TselEscapeAlgorithm changed: bb.flagBumperOutsidePerActivated = true;\r\n"));

				}
				sprintf(errorHandler.msg, text, enuFlagEscabeObstacleConFlagString[bb.flagEscabeObstacleConFlag]);
				errorHandler.setInfo();
				break;
			case DD_FEOROTATECC1:
				bb.flagEscabeObstacleConFlag = FEO_ROTCW2;

				// If  Bumper was inside activated and now coils are outside, simulate bumper outside activated, because we don't want to call mseqPerimeterActive
				if (bb.flagPerimeterStateChanged == PSC_IO) {
					bb.flagEscabeObstacleConFlag = FEO_ROTCC2;
					bb.flagBumperInsidePerActivated = false;
					bb.flagBumperOutsidePerActivated = true;
					errorHandler.setInfo(F("!03,TselEscapeAlgorithm changed: bb.flagBumperOutsidePerActivated = true;\r\n"));

				}
				sprintf(errorHandler.msg, text, enuFlagEscabeObstacleConFlagString[bb.flagEscabeObstacleConFlag]);
				errorHandler.setInfo();
				break;

			case DD_FEOROTATECW2:
				bb.flagEscabeObstacleConFlag = FEO_ROTCC1;
				// If  Bumper was inside activated and now coils are outside, simulate bumper outside activated, because we don't want to call mseqPerimeterActive
				if (bb.flagPerimeterStateChanged == PSC_IO) {
					bb.flagEscabeObstacleConFlag = FEO_ROTCW2;
					bb.flagBumperInsidePerActivated = false;
					bb.flagBumperOutsidePerActivated = true;
					errorHandler.setInfo(F("!03,TselEscapeAlgorithm changed: bb.flagBumperOutsidePerActivated = true;\r\n"));

				}
				sprintf(errorHandler.msg, text, enuFlagEscabeObstacleConFlagString[bb.flagEscabeObstacleConFlag]);
				errorHandler.setInfo();
				break;
			case DD_FEOROTATECC2:
				bb.flagEscabeObstacleConFlag = FEO_ROTCW1;

				// If  Bumper was inside activated and now coils are outside, simulate bumper outside activated, because we don't want to call mseqPerimeterActive
				if (bb.flagPerimeterStateChanged == PSC_IO) {
					bb.flagEscabeObstacleConFlag = FEO_ROTCC2;
					bb.flagBumperInsidePerActivated = false;
					bb.flagBumperOutsidePerActivated = true;
					errorHandler.setInfo(F("!03,TselEscapeAlgorithm changed: bb.flagBumperOutsidePerActivated = true;\r\n"));

				}
				sprintf(errorHandler.msg, text, enuFlagEscabeObstacleConFlagString[bb.flagEscabeObstacleConFlag]);
				errorHandler.setInfo();
				break;
			case DD_LINE_FOLLOW: // Aktuell nicht behandelt da voraussetung, kein Hinderniss auf Perimeter 
				bb.flagBumperOutsidePerActivated = false;
				bb.flagBumperInsidePerActivated = false;
				break;
			case DD_REVERSE_ESC_OBST:
				// Nur eingefügt um Kompilationsfehler zu vermeiden.
				// Wird in mseqPerimeterActive bearbeitetrn BH_SUCCESS;
				break;
			default:
				sprintf(errorHandler.msg, "!03,TselEscapeAlgorithm driveDirection not found: %s\r\n", enuFlagEscabeObstacleConFlagString[bb.flagEscabeObstacleConFlag]);
				errorHandler.setError();
				break;
			}
		}


		//if(bb.flagPerimeterStateChanged == PSC_NONE && bb.flagBumperOutsidePerActivated) {
		// else ist hier wichtig´, da oben in der if abfrage flagBumperOutsidePerActivated ggf. aud true gesetzt wird.
		else if (bb.flagBumperOutsidePerActivated) {

			switch (bb.driveDirection) {
			case DD_FORWARD:
			case DD_OVERRUN:
			case DD_REVERSE_INSIDE:
			case DD_SPIRAL_CW:
				bb.flagEscabeObstacleConFlag = FEO_BACKINSIDE;
				sprintf(errorHandler.msg, text, enuFlagEscabeObstacleConFlagString[bb.flagEscabeObstacleConFlag]);
				errorHandler.setInfo();
				break;
			case DD_FORWARD_INSIDE:
				errorHandler.setError(F("!03,TselEscapeAlgorithm DD_FORWARD_INSIDE in Bumper außerhalb Perimeter aktiviert 1\r\n"));
				break;
			case DD_ROTATECW:
				bb.flagEscabeObstacleConFlag = FEO_ROTCW2;
				//bb.flagForceRotateDirection = FRD_CC;
				sprintf(errorHandler.msg, text, enuFlagEscabeObstacleConFlagString[bb.flagEscabeObstacleConFlag]);
				errorHandler.setInfo();
				break;
			case DD_ROTATECC:
				bb.flagEscabeObstacleConFlag = FEO_ROTCC2;
				//bb.flagForceRotateDirection = FRD_CW;
				sprintf(errorHandler.msg, text, enuFlagEscabeObstacleConFlagString[bb.flagEscabeObstacleConFlag]);
				errorHandler.setInfo();
				break;
			case DD_ROTATECW1:
				bb.flagEscabeObstacleConFlag = FEO_ROTCW2;
				sprintf(errorHandler.msg, text, enuFlagEscabeObstacleConFlagString[bb.flagEscabeObstacleConFlag]);
				errorHandler.setInfo();
				break;
			case DD_ROTATECC1:
				bb.flagEscabeObstacleConFlag = FEO_ROTCC2;
				sprintf(errorHandler.msg, text, enuFlagEscabeObstacleConFlagString[bb.flagEscabeObstacleConFlag]);
				errorHandler.setInfo();
				break;
			case DD_FEOROTATECW:
			case DD_FEOROTATECW1:
			case DD_FEOROTATECW2:
				bb.flagEscabeObstacleConFlag = FEO_ROTCC2;
				sprintf(errorHandler.msg, text, enuFlagEscabeObstacleConFlagString[bb.flagEscabeObstacleConFlag]);
				errorHandler.setInfo();
				break;
			case DD_FEOROTATECC:
			case DD_FEOROTATECC1:
			case DD_FEOROTATECC2:
				bb.flagEscabeObstacleConFlag = FEO_ROTCW2;
				sprintf(errorHandler.msg, text, enuFlagEscabeObstacleConFlagString[bb.flagEscabeObstacleConFlag]);
				errorHandler.setInfo();
				break;


			case DD_LINE_FOLLOW: // Aktuell nicht behandelt da voraussetung, kein Hinderniss auf Perimeter
				bb.flagBumperOutsidePerActivated = false;
				bb.flagBumperInsidePerActivated = false;
				break;
			case DD_REVERSE_ESC_OBST:
				// Nur eingefügt um Kompilationsfehler zu vermeiden.
				// Wird in mseqPerimeterActive bearbeitetrn BH_SUCCESS;
				//} 
				break;
			default:
				sprintf(errorHandler.msg, "!03,TselEscapeAlgorithm driveDirection not found: %s\r\n", enuFlagEscabeObstacleConFlagString[bb.flagEscabeObstacleConFlag]);
				errorHandler.setError();
				break;
			}
		}

		/*
				//////////////////////////////////////
				if(bb.flagPerimeterStateChanged == PSC_IO) {

					// Nur bei wechsel von inside nach outside richtung anpassen
					// Coils waren bei bumperereignis innen und sind nun durch befreieung auÃŸerhalb

					bb.flagBumperOutsidePerActivated  = true; // OK, ich bin nun ausÃŸerhalb und so tun als ob ereignis auÃŸerhalb aufgetreten ist.
					bb.flagBumperInsidePerActivated  = false;

					switch(bb.driveDirection) {
						case DD_FORWARD:
						case DD_OVERRUN:
							bb.driveDirection = DD_REVERSE_ESC_OBST;
							sprintf(errorHandler.msg,text,enuDriveDirectionString[bb.driveDirection]);
							errorHandler.setInfo();
							break;
						case DD_FORWARD_INSIDE:
							bb.driveDirection = DD_REVERSE_ESC_OBST;
							sprintf(errorHandler.msg,text,enuDriveDirectionString[bb.driveDirection]);
							errorHandler.setInfo();
							break;

						case DD_REVERSE_INSIDE_CURVE_CW:
						case DD_REVERSE_INSIDE_CURVE_CC:
						case DD_REVERSE_INSIDE:
							bb.driveDirection = DD_REVERSE_ESC_OBST;
							sprintf(errorHandler.msg,text,enuDriveDirectionString[bb.driveDirection]);
							errorHandler.setInfo();
							break;
						case DD_REVERSE_ESC_OBST:
							bb.driveDirection = DD_OVERRUN;
							sprintf(errorHandler.msg,text,enuDriveDirectionString[bb.driveDirection]);
							errorHandler.setInfo();
							break;
						case DD_ROTATECW:
							bb.driveDirection = DD_ROTATECC;
							sprintf(errorHandler.msg,text,enuDriveDirectionString[bb.driveDirection]);
							errorHandler.setInfo();
							break;
						case DD_ROTATECC:
							bb.driveDirection = DD_ROTATECW;
							sprintf(errorHandler.msg,text,enuDriveDirectionString[bb.driveDirection]);
							errorHandler.setInfo();
							break;
						default:
							sprintf(errorHandler.msg,"!03,driveDirection not found: %s",enuDriveDirectionString[bb.driveDirection]);
							errorHandler.setError();
							break;
					}
				}
				//////////////////////////////////////

		*/

		return BH_SUCCESS;
	}


};



#endif

