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
#ifndef BH_FOLLOWLINE_H
#define BH_FOLLOWLINE_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "BehaviourTree.h"
#include "positioncontrol.h"
#include "config.h"


class TperTrackChargingStationReached : public Action {
private:

public:

	TperTrackChargingStationReached() {
	}

	virtual void onInitialize(Blackboard& bb) {
		srvMotor.enableDefaultRamping();
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		bb.setBehaviour(BH_CHARGING);
		//debug->printf("bb.setBehaviour(BH_CHARGING);");
		return BH_SUCCESS;
	}
};



class TLFRotateCC105 : public Action {
private:

public:

	TLFRotateCC105() {

	}

	virtual void onInitialize(Blackboard& bb) {

		THistory hist = bb.getInitialisedHistoryEntry();
		hist.cruiseSpeed = bb.CRUISE_ROTATE_HIGH;
		hist.driveDirection = DD_ROTATECC;
		hist.distanceSoll = -105;
		bb.addHistoryEntry(hist);
		srvMotor.turnTo(-105, hist.cruiseSpeed);

	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (getTimeInNode() > 10000) {
			errorHandler.setError(F("!03,TFLRotateCC105 too long in state\r\n"));
		}

		if (srvMotor.isPositionReached()) {
			bb.setBehaviour(BH_FINDPERIMETER);
			return BH_SUCCESS;
		}

		return BH_RUNNING;
	}


	
};

class TlineFollow : public Action {
private:
	bool waitForRightInside;
	double integral;
	unsigned long lastRun;
	unsigned long lastTransitionTime;

public:
	double Ki;

	TlineFollow() {
		Ki = 1.1f / 1.5;
		lastRun = 0;
		integral = 0;
	}

	virtual void onInitialize(Blackboard& bb) {
		lastRun = millis() - 50ul;
		integral = 0;
		waitForRightInside = false;
		lastTransitionTime = millis();
		srvMotor.enablePerTrackRamping();
		//errorHandler.setInfoNoLog(F("TlineFollow onInitialize \r\n"));

		THistory hist = bb.getInitialisedHistoryEntry();
		hist.cruiseSpeed = bb.LINEFOLLOW_SPEED_HIGH;
		hist.driveDirection = DD_FORWARD;
		hist.distanceSoll = 40000;
		bb.addHistoryEntry(hist);
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {


		if (millis() - lastRun < 33) return BH_RUNNING;
		lastRun = millis();


		// Calculate Speed
		//============================================
		double error = srvPerSensoren.magnetudeR0; //important use magnetudeR0 here with includes 0 also
		if (error < 0) {
			error = -1;
		}
		else {
			error = 1;
		}

		integral = integral + (Ki * error);

		//debug->printf("error: %f\r\n",error);

		//Set integral to 0 if crossing the line
		if (sign0minus(error) != sign0minus(integral)) { //sign0minus => 0 belongs to minus
			integral = 0;
			lastTransitionTime = millis();
		}

		//debug->printf("trans: %lu\r\n",millis()-srvPerSensoren.perimeterLastTransitionTimeB);

		double Output = integral;

		//debug->printf("p:%f i%f d:%f o:%f\r\n",Kp * error , integral, Kd * derivate, Output);

		if (Output > bb.LINEFOLLOW_SPEED_HIGH) Output = bb.LINEFOLLOW_SPEED_HIGH;
		if (Output < -1 * bb.LINEFOLLOW_SPEED_HIGH) Output = -1 * bb.LINEFOLLOW_SPEED_HIGH;


		bb.history0.cruiseSpeed = bb.LINEFOLLOW_SPEED_HIGH;
		bb.history0.driveDirection = DD_FORWARD;

		// If left coil outside, rotate CC until right is inside

		if (waitForRightInside == true) {
			if (srvPerSensoren.isRightInside()) {
				waitForRightInside = false;
			}
		}

		// Follow line
		if (waitForRightInside == false) {
			if (error < 0.0f) { //Set Speed Outside Perimeter 

				if (srvPerSensoren.isLeftOutside() == true) {
					bb.history0.cruiseSpeed = 25;
					bb.history0.driveDirection = DD_ROTATECC;
					srvMotor.L->setSpeed(-25);
					srvMotor.R->setSpeed(25);
					waitForRightInside = true;
				}
				else if ((millis() - lastTransitionTime) > 2000) { // If more than 3.5sec Outside rotate full
					bb.history0.cruiseSpeed = 25;
					bb.history0.driveDirection = DD_ROTATECC;
					srvMotor.L->setSpeed(-25);
					srvMotor.R->setSpeed(25);
				}
				else if ((millis() - lastTransitionTime) > 1500) { // If more than 2.8sec Outside rotate more aggressive
					srvMotor.L->setSpeed((bb.history0.cruiseSpeed + Output));
					srvMotor.R->setSpeed((bb.history0.cruiseSpeed + 10));
				}
				else if ((millis() - lastTransitionTime) > 1000) { // If more than 2sec Outside rotate aggressive
					srvMotor.L->setSpeed((bb.history0.cruiseSpeed + Output));
					srvMotor.R->setSpeed((bb.history0.cruiseSpeed + 5));
				}
				else {
					srvMotor.L->setSpeed((bb.history0.cruiseSpeed + Output));
					srvMotor.R->setSpeed((bb.history0.cruiseSpeed));
				}

			}
			else { //Set Speed Inside Perimeter

				if (srvPerSensoren.isLeftOutside() == true) {
					bb.history0.cruiseSpeed = 25;
					bb.history0.driveDirection = DD_ROTATECC;
					srvMotor.L->setSpeed(-25);
					srvMotor.R->setSpeed(25);
					waitForRightInside = true;
				}
				else if ((millis() - lastTransitionTime) > 1800) { // // If more than 2sec inside rotate full
					bb.history0.cruiseSpeed = 25;
					bb.history0.driveDirection = DD_ROTATECW;
					srvMotor.L->setSpeed(25);
					srvMotor.R->setSpeed(-25);
				}
				else if ((millis() - lastTransitionTime) > 1500) { // If more than 1.5sec inside rotate more aggressive
					srvMotor.L->setSpeed((bb.history0.cruiseSpeed + 10));
					srvMotor.R->setSpeed(-35);
				}
				else if ((millis() - lastTransitionTime) > 1000) { // If more than 1sec inside rotate aggressive
					srvMotor.L->setSpeed((bb.history0.cruiseSpeed + 10)); //50+10=60
					srvMotor.R->setSpeed(-25);  // -8
				}
				else {
					srvMotor.L->setSpeed((bb.history0.cruiseSpeed + 5)); //50+10=60
					srvMotor.R->setSpeed((bb.history0.cruiseSpeed - 20)); //50-25=25

				}
			}
		}

		if ((millis() - lastTransitionTime) > 12000) {
			errorHandler.setError(F("!03,TlineFollow line crossing > 12000\r\n"));
		}
		return BH_RUNNING;
	}

	virtual void onTerminate(NodeStatus status, Blackboard& bb) {
		//errorHandler.setInfoNoLog(F("TlineFollow onTerminate \r\n"));
		//srvMotor.enableDefaultRamping();
	}
};



/*
  Fuerr das finden eines Dreiecks in der Schleife wird staendig nach einer Linkskurve "Ausschau" gehalten.
  Dazu werden alle 500ms die Encoderdaten ausgelesen und ausgewertet.
  Wenn eine Linkskurve erkannt wurde wird die Zeit festgehalten und timeLeftTurnFound gesetzt.
  Wenn ein Dreieck vohanden ist, wird der Robbi nicht mehr so einfach von innen nach aussen wechseln koennen,
  da er nach der scharfen Linkskurfe bereits Ã¼ber das Ende des Dreieckes gefahren ist. Daher wird er auf jeden Fall in die if Abfrage:
  "if ( (millis()-lastTransitionTime) > 3500 )" gehen wo er rotiert. Hier wird geguckt, ob die Linkskurve innerhalb
  der letzten 5 Sek. gefunden wurde. Wenn ja, wird flagTriangleFound = true gesetzt und rotiert, bis die andere Seite des Dreiecks erreicht
  wurde. Wenn diese erreicht wurde, geht die Spule von innen nach auÃŸen. Bei diesem Ãœbergang wird geprÃ¼ft ob ein Dreieck gefunden wurde mit
  flagTriangleFound. Weiterhin muss mindesten noch eine Kurve von 80 nach rechts gefahren worden sein und die  Zeit fÃ¼r die Rechtskurve muss mindestens
  4Sek. gedauert haben. Dann werden die Zweige im BehaviourTree mit  bb.flagFollowLine = false;  bb.flagGoHome = true; umgeschaltet, so dass der Mower wieder normal fÃ¤hrt
  bis die andere Seite des Perimeters erreicht wurde.

*/

#define ENCDELTAARRAY 5  // Check curve for 2.5 sec.

class TfindTriangle : public Action {
private:
	unsigned long lastRunAngleCalculation;
	int encDeltaL[ENCDELTAARRAY]; //Array measured encoder ticks/100ms
	int encDeltaR[ENCDELTAARRAY];
	int idxL;
	int idxR;
	float angle;
	long lastEncTicksL;
	long lastEncTicksR;
	uint8_t state;


public:
	double Ki;
	bool flagShowFindTriangleStates;

	TfindTriangle() {
		Ki = 1.1f;
		flagShowFindTriangleStates = false;
	}

	virtual void onInitialize(Blackboard& bb) {
		lastRunAngleCalculation = 0;
		memset(&encDeltaL[0], 0, ENCDELTAARRAY * sizeof(int));
		memset(&encDeltaR[0], 0, ENCDELTAARRAY * sizeof(int));
		idxL = 0;
		idxR = 0;
		lastEncTicksL = srvMotor.getEncoderTickCountsL();
		lastEncTicksR = srvMotor.getEncoderTickCountsR();
		angle = 0;
		state = 0;
		//errorHandler.setInfoNoLog(F("ON INITIALIZE"));

	}

	virtual NodeStatus onUpdate(Blackboard& bb) {
		long sumL = 0;
		long sumR = 0;
		long buffL, buffR;
		float distance;

		if (CONF_DISABLE_FAST_RETURN == true) {
			return BH_RUNNING;
		}

		//============================================
		// Calculate driven angle every 500ms.
		//============================================
		if (millis() - lastRunAngleCalculation >= 500) {
			lastRunAngleCalculation = millis();

			buffL = srvMotor.getEncoderTickCountsL();
			buffR = srvMotor.getEncoderTickCountsR();
			encDeltaL[idxL++] = buffL - lastEncTicksL;
			encDeltaR[idxR++] = buffR - lastEncTicksR;
			lastEncTicksL = buffL;
			lastEncTicksR = buffR;

			if (idxL > ENCDELTAARRAY - 1) idxL = 0;
			if (idxR > ENCDELTAARRAY - 1) idxR = 0;

			/*
			for(int i=0; i<ENCDELTAARRAY; i++) {
				errorHandler.setInfoNoLog(F("%d  %d  %d\r\n"),encDeltaL[i], encDeltaR[i], encDeltaL[i] - encDeltaR[i]);
			}
			errorHandler.setInfoNoLog(F("==============\r\n"));
			*/

			for (int i = 0; i < ENCDELTAARRAY; i++) {
				sumL += encDeltaL[i];
				sumR += encDeltaR[i];
			}

			//The average angle the robot has traveled
			//theta = (LeftEncoderDistance−RightEncoderDistance) / wheelbase
			//theta = (36.8 - 27.97) / 3 = 2.95 radians
			//1 Grad = 0,017453292519943295769236907684886 Bogenmaß (radiant) 1/0,017... = 57,2957795  => angle=(360/(2Pi))*Rad
			//theta = 2.95/0.017453292 oder 2.95*57.2957795
			float cmL = srvMotor.pcL->getCMForCounts(sumL);
			float cmR = srvMotor.pcR->getCMForCounts(sumR);
			angle = (cmL - cmR) * 57.2957795f / CONF_DISTANCE_BETWEEN_WHEELS_CM;
			if (flagShowFindTriangleStates) {
				errorHandler.setInfoNoLog(F("Winkel: %f cmL %f cmR %f\r\n"), angle, cmL, cmR);
			}
		}


		// Check for left curve / right curve / second left curve
		switch (state) {
		case 0:  // search for left curve
			if (angle < -25) {
				if (flagShowFindTriangleStates) {
					errorHandler.setInfo(F("!03,s0 set state = 1 Left turn found angle: %f ms: %lu\r\n"), angle, millis());
				}
				srvMotor.startDistanceMeasurementTriangle();
				// array loeschen damit letzte linkskurve die rechts winkelmessung nicht beeinflusst
				memset(&encDeltaL[0], 0, ENCDELTAARRAY * sizeof(int));
				memset(&encDeltaR[0], 0, ENCDELTAARRAY * sizeof(int));
				state = 1; // Activate triangle searching
			}
			break;

		case 1: // search for right turn
			distance = srvMotor.getDistanceInCMForTriangle();
			if (distance > 70) {
				state = 0;
				if (flagShowFindTriangleStates) {
					errorHandler.setInfo(F("!03,s1 set state = 0 distance %f > 65 ms: %lu\r\n"), distance, millis());
				}
			}

			else if (angle > 50) {
				if (flagShowFindTriangleStates) {
					errorHandler.setInfo(F("!03,s1 set state = 2 angle %f > 50 distance %f\r\n"), angle, distance);
				}
				srvMotor.startDistanceMeasurementTriangle();
				// array loeschen damit letzte linkskurve die rechts winkelmessung nicht beeinflusst
				memset(&encDeltaL[0], 0, ENCDELTAARRAY * sizeof(int));
				memset(&encDeltaR[0], 0, ENCDELTAARRAY * sizeof(int));
				state = 2;
			}

			break;

		case 2:  // search for second left curve
			distance = srvMotor.getDistanceInCMForTriangle();
			if (distance > 65) {
				state = 0;
				if (flagShowFindTriangleStates) {
					errorHandler.setInfo(F("!03,s2 set state = 0 distance %f > 65 ms: %lu\r\n"), distance, millis());
				}
			}

			else if (angle < -25) {
				if (flagShowFindTriangleStates) {
					errorHandler.setInfo(F("!03,s2 set state = 3 angle %f distance %f < -22\r\n"), angle, distance);
				}
				srvMotor.startDistanceMeasurementTriangle();
				// array loeschen damit letzte linkskurve die rechts winkelmessung nicht beeinflusst
				memset(&encDeltaL[0], 0, ENCDELTAARRAY * sizeof(int));
				memset(&encDeltaR[0], 0, ENCDELTAARRAY * sizeof(int));
				state = 3;

			}

			break;

		case 3:

			if (flagShowFindTriangleStates) {
				errorHandler.setInfo(F("!03,s3 set state=0  Cross Lawn Activated\r\n"));
			}
			state = 0;
			//srvMotor.enableDefaultRamping();
			return BH_SUCCESS;

			break;

		default:
			errorHandler.setError(F("!03,TtrackPerimeter state %d not found\r\n"), state);
			break;
		}

		return BH_RUNNING;
	}


	virtual void onTerminate(NodeStatus status, Blackboard& bb) {
		//debug->printf("onTerminate TFLfollowLine enableDefaultRamping()\r\n" );
		//srvMotor.enableDefaultRamping();
	}

};


class TSetArc45CC : public Action    // Each task will be a class (derived from Node of course).
{
private:
public:

	TSetArc45CC() {}

	virtual void onInitialize(Blackboard& bb) {

		errorHandler.setInfo(F("!05,TSetArc45CW\r\n"));

		THistory hist = bb.getInitialisedHistoryEntry();
		hist.cruiseSpeed = bb.CRUISE_SPEED_LOW;
		hist.driveDirection = DD_ROTATECC;
		hist.distanceSoll = -45;
		bb.addHistoryEntry(hist);
		srvMotor.enableDefaultRamping();
		
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		return BH_SUCCESS;
	}

};


class TcheckOutsideAgain : public Action    // Each task will be a class (derived from Node of course).
{
private:
public:

	TcheckOutsideAgain() {}

	virtual void onInitialize(Blackboard& bb) {
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {
		if (getTimeInNode() > 1000ul) {
			if (srvPerSensoren.isLeftOutside() || srvPerSensoren.isRightOutside()) {
				errorHandler.setInfo(F("!05,TcheckOutsideAgain perimeter outside\r\n"));
				bb.flagDriveCurve = false;
			}
			else {
				errorHandler.setInfo(F("!05,TcheckOutsideAgain perimeter NOT outside\r\n"));
				//bb.flagDriveCurve = true;  //has not changed, therefore I don't have to set it again
			}
			return BH_SUCCESS;
		}
		return BH_RUNNING;
	}

};


class TdriveCurve : public Action {
private:
	bool oneCoilsOutside;

public:

	TdriveCurve() {}

	virtual void onInitialize(Blackboard& bb) {

		//Check if one coil is outside when start driving
		if (srvPerSensoren.isLeftOutside() || srvPerSensoren.isRightOutside()) {
			oneCoilsOutside = true;
		}
		else {
			oneCoilsOutside = false;
		}


		THistory hist = bb.getInitialisedHistoryEntry();
		hist.cruiseSpeed = bb.CRUISE_SPEED_LOW;
		hist.driveDirection = DD_FORWARD;
		hist.distanceSoll = 1000;
		bb.addHistoryEntry(hist);


		srvMotor.pcL->rotateCM(1000, 50);
		srvMotor.pcR->rotateCM(1000, 40);
		//if (bb.flagShowRotateX) {
		errorHandler.setInfo(F("!03,TdriveCurve\r\n"));
		//}


	}

	virtual NodeStatus onUpdate(Blackboard& bb) {


		if (getTimeInNode() > 60000) { // drive no longer than 60 seconds for searching the perimeter (for security)
			errorHandler.setError(F("!03,TdriveCurve too long in state\r\n"));
			return BH_SUCCESS;
		}


		// If one coil outside while start driving, wait until both coils are inside again.
		if (oneCoilsOutside == false) {
			if (srvPerSensoren.isLeftOutside() || srvPerSensoren.isRightOutside()) {
				errorHandler.setInfo(F("!05,TdriveCurve perimeter found\r\n"));
				return BH_SUCCESS;
			}
		}
		else { // Wait that both coils inside
			if (srvPerSensoren.isLeftInside() && srvPerSensoren.isRightInside()) {
				oneCoilsOutside = false;
			}
			else {
				// If after 20cm driveway both coils are not inside then error.
				if (bb.history0.distanceIst > 20) {
					errorHandler.setError(F("!05,TdriveCurve one or both coils outside after 20cm\r\n"));
				}
			}

		}

		//Because the rigth wheel rotates slower, whe have to ask each wheel individually if its position is reached.
		//It could be, that the left wheel has reached the 10m but not the right wheel. Then the left wheel would be stopped  
		//and the right wheel would drive further.
		if (srvMotor.pcL->isPositionReached() || srvMotor.pcR->isPositionReached()) {
			errorHandler.setError(F("!05,TdriveCurve position reached\r\n"));
			return BH_SUCCESS;
		}


		return BH_RUNNING;
	}

	virtual void onTerminate(NodeStatus status, Blackboard& bb) {
		//bb.flagDriveCurve = false;
		/*
		if(status != BH_ABORTED) {

		}
		*/
	}

};

#endif

