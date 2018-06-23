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


/*
class TconStopOvershootLeft: public Node    // Condition
{
private:

public:

	TconStopOvershootLeft() {}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if(bb.perimeterSensoren.isBackInside()) {
			if ( (millis()-bb.perimeterSensoren.perimeterLastTransitionTimeB) > 10000 ) { // Links  ueber Rand
				return BH_SUCCESS;
			}
		}

		return BH_FAILURE;
	}
};


class TconStopOvershootRight: public Node    // Condition
{
private:

public:

	TconStopOvershootRight() {}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if(bb.perimeterSensoren.isBackOutside()) {
			if ( (millis()-bb.perimeterSensoren.perimeterLastTransitionTimeB) > 10000 ) { // Rechts  ueber Rand
				return BH_SUCCESS;
			}
		}

		return BH_FAILURE;
	}
};

*/


class TdriveCurve : public Node
{
private:
	bool oneCoilsOutside;
	
public:

	TdriveCurve() {}

	virtual void onInitialize(Blackboard& bb) {

		//Check if one coil is outside when start driving
		if (bb.perimeterSensoren.isLeftOutside() || bb.perimeterSensoren.isRightOutside()) {
			oneCoilsOutside = true;
		}
		else {
			oneCoilsOutside = false;
		}

		bb.driveDirection = DD_FORWARD;
		bb.cruiseSpeed = bb.CRUISE_SPEED_LOW;
		bb.motor.pcL->rotateCM(1000, 50);
		bb.motor.pcR->rotateCM(1000, 40);
		//if (bb.flagShowRotateX) {
			errorHandler.setInfo(F("!03,TdriveCurve\r\n"));
		//}

		bb.addHistoryEntry(bb.driveDirection, 0.0f, 0.0f, 0.0f, FRD_NONE, CO_NONE);

	}

	virtual NodeStatus onUpdate(Blackboard& bb) {
		
		bb.history[0].distanceDriven = bb.motor.getDistanceInCM();

		if (getTimeInNode() > 60000) { // drive no longer than 60 seconds for searching the perimeter (for security)
			errorHandler.setError(F("!03,TdriveCurve too long in state\r\n"));
			return BH_SUCCESS;
		}


		// If one coil outside while start driving, wait until both coils are inside again.
		if (oneCoilsOutside == false) {
			if (bb.perimeterSensoren.isLeftOutside() || bb.perimeterSensoren.isRightOutside()) {
				errorHandler.setInfo(F("!05,TdriveCurve perimeter found\r\n"));
				return BH_SUCCESS;
			}
		}
		else { // Wait that both coils inside
			if (bb.perimeterSensoren.isLeftInside() && bb.perimeterSensoren.isRightInside()) {
				oneCoilsOutside = false;
			}
			else {
				// If after 20cm driveway both coils are not inside then error.
				if (bb.history[0].distanceDriven > 20) {
					errorHandler.setError(F("!05,TdriveCurve one or both coils outside after 20cm\r\n"));
				}
			}

		}

		//Because the rigth wheel rotates slower, whe have to ask each wheel individually if its position is reached.
		//It could be, that the left wheel has reached the 10m but not the right wheel. Then the left wheel would be stopped  
		//and the right wheel would drive further.
		if (bb.motor.pcL->isPositionReached() || bb.motor.pcR->isPositionReached()){
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


class TcheckOutsideAgain : public Node    // Each task will be a class (derived from Node of course).
{
private:
public:

	TcheckOutsideAgain() {}

	virtual void onInitialize(Blackboard& bb) {
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {
     if(getTimeInNode() > 1000ul){
		    if (bb.perimeterSensoren.isLeftOutside() || bb.perimeterSensoren.isRightOutside()) {
			      errorHandler.setInfo(F("!05,TcheckOutsideAgain perimeter outside\r\n"));
			      bb.flagDriveCurve = false;
		    } else {
            errorHandler.setInfo(F("!05,TcheckOutsideAgain perimeter NOT outside\r\n"));
            //bb.flagDriveCurve = true;  //has not changed, therefore I don't have to set it again
		    }
        return BH_SUCCESS;
     } 
		 return BH_RUNNING;
	}

};


class TSetArc45CC : public Node    // Each task will be a class (derived from Node of course).
{
private:
public:

	TSetArc45CC() {}

	virtual void onInitialize(Blackboard& bb) {
		bb.flagForceRotateDirection = FRD_CC;
		bb.driveDirection = DD_ROTATECC;
		bb.arcRotateXArc = 45;
		bb.flagForceSmallRotAngle = 0;
		bb.flagDeactivateRotInside = false;
		bb.flagCoilFirstOutside = CO_RIGHT;
		bb.flagRotateAtPer = false;
		//if (bb.flagShowRotateX) {
			errorHandler.setInfo(F("!05,TSetArc45CW\r\n"));
		//}
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {
		
		return BH_SUCCESS;
	}

};




class TperTrackChargingStationReached : public Node
{
private:

public:

	TperTrackChargingStationReached() {


	}

	virtual void onInitialize(Blackboard& bb) {
		bb.motor.enableDefaultRamping();

	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		bb.setBehaviour(BH_CHARGING);
		//debug->printf("bb.setBehaviour(BH_CHARGING);");
		return BH_SUCCESS;
	}
};


/*
class TFLRotateCC : public Node
{
private:

public:

	TFLRotateCC() {

	}

	virtual void onInitialize(Blackboard& bb) {
		bb.cruiseSpeed = 15;
		bb.driveDirection = DD_ROTATECC;
		bb.motor.L->setSpeed(-15);
		bb.motor.R->setSpeed(15);
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (getTimeInNode() > 10000) {
			errorHandler.setError("!03,FLRotateLeft too long in state\r\n");
		}

		if (bb.perimeterSensoren.isRightInsideMag()) { // Spule ueber Fahrspurmittelpunkt
			return BH_SUCCESS;
		}



		return BH_RUNNING;
	}
};


class TLFRotateCW : public Node
{
private:

public:

	TLFRotateCW() {

	}

	virtual void onInitialize(Blackboard& bb) {
		bb.cruiseSpeed = bb.CRUISE_ROTATE_LOW;
		bb.driveDirection = DD_ROTATECW;
		bb.motor.L->setSpeed(bb.cruiseSpeed);
		bb.motor.R->setSpeed(-bb.cruiseSpeed);
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (getTimeInNode() > 10000) {
			errorHandler.setError("!03,TFLRotateCW too long in state\r\n");
		}


		if (bb.perimeterSensoren.isRightOutsideMag()) {  // Spule ueber Fahrbahnmitte nach innen
			return BH_SUCCESS;
		}


		return BH_RUNNING;
	}
};

*/

class TLFRotateCC105 : public Node
{
private:

public:

	TLFRotateCC105() {

	}

	virtual void onInitialize(Blackboard& bb) {
		bb.cruiseSpeed = bb.CRUISE_ROTATE_HIGH;
		bb.driveDirection = DD_ROTATECC;
		bb.motor.turnTo(-105, bb.cruiseSpeed);
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (getTimeInNode() > 10000) {
			errorHandler.setError(F("!03,TFLRotateCC105 too long in state\r\n"));
		}

		if (bb.motor.isPositionReached()) {
			bb.setBehaviour(BH_FINDPERIMETER);
			return BH_SUCCESS;
		}

		return BH_RUNNING;
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

class TfindTriangle : public Node
{
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
		lastEncTicksL = bb.encoderL.getTickCounter();
		lastEncTicksR = bb.encoderR.getTickCounter();
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

			buffL = bb.encoderL.getTickCounter();
			buffR = bb.encoderR.getTickCounter();
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
			float cmL = bb.motor.pcL->getCMForCounts(sumL);
			float cmR = bb.motor.pcR->getCMForCounts(sumR);
			angle = (cmL - cmR)  * 57.2957795f / CONF_DISTANCE_BETWEEN_WHEELS_CM;
			if (flagShowFindTriangleStates) {
				errorHandler.setInfoNoLog(F("Winkel: %f cmL %f cmR %f\r\n"), angle, cmL, cmR);
			}
		}


		// Check for left curve / right curve / second left curve
		switch (state)
		{
		case 0:  // search for left curve
			if (angle < -25) { 
				if (flagShowFindTriangleStates) {
					errorHandler.setInfo(F("!03,s0 set state = 1 Left turn found angle: %f ms: %lu\r\n"), angle, millis());
				}
				bb.motor.startDistanceMeasurementTriangle();
				// array loeschen damit letzte linkskurve die rechts winkelmessung nicht beeinflusst
				memset(&encDeltaL[0], 0, ENCDELTAARRAY * sizeof(int));
				memset(&encDeltaR[0], 0, ENCDELTAARRAY * sizeof(int));
				state = 1; // Activate triangle searching
			}
			break;

		case 1: // search for right turn
			distance = bb.motor.getDistanceInCMForTriangle();
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
				bb.motor.startDistanceMeasurementTriangle();
				// array loeschen damit letzte linkskurve die rechts winkelmessung nicht beeinflusst
				memset(&encDeltaL[0], 0, ENCDELTAARRAY * sizeof(int));
				memset(&encDeltaR[0], 0, ENCDELTAARRAY * sizeof(int));
				state = 2;
			}

			break;

		case 2:  // search for second left curve
			distance = bb.motor.getDistanceInCMForTriangle();
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
				bb.motor.startDistanceMeasurementTriangle();
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
			//bb.motor.enableDefaultRamping();
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
		//bb.motor.enableDefaultRamping();
	}

};


class TMotorStopFast : public Node
{
private:

public:

	TMotorStopFast() {}

	virtual void onInitialize(Blackboard& bb) {
		bb.motor.enableFastStopRamping();
		bb.motor.stopCLC();
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (getTimeInNode() > 10000) {
			errorHandler.setError(F("!03,motorStop too long in state\r\n"));
		}

		if (bb.motor.isCLCStopped()) {
			bb.cruiseSpeed = 0;
			bb.history[0].rotAngleIst = bb.motor.getAngleRotatedAngleDeg();
			bb.history[0].distanceDriven = bb.motor.getDistanceInCM();
			//debug->printf("onUpdate enableFastStopRamping()\r\n");
			//bb.motor.enableFastStopRamping();
			return BH_SUCCESS;
		}

		return BH_RUNNING;
	}

	virtual void onTerminate(NodeStatus status, Blackboard& bb) {
		//debug->printf("onTerminate FastStopRamping()\r\n");
		//bb.motor.enableDefaultRamping();
	}
};


/********************************
class TfindTriangle : public Node
{
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
lastEncTicksL = bb.encoderL.getTickCounter();
lastEncTicksR = bb.encoderR.getTickCounter();
angle = 0;
state = 0;
//errorHandler.setInfoNoLog(F("ON INITIALIZE"));

}

virtual NodeStatus onUpdate(Blackboard& bb) {
long sumL = 0;
long sumR = 0;
long buffL, buffR;
float distance;

//============================================
// Calculate driven angle every 500ms.
//============================================
if (millis() - lastRunAngleCalculation >= 500) {
lastRunAngleCalculation = millis();

buffL = bb.encoderL.getTickCounter();
buffR = bb.encoderR.getTickCounter();
encDeltaL[idxL++] = buffL - lastEncTicksL;
encDeltaR[idxR++] = buffR - lastEncTicksR;
lastEncTicksL = buffL;
lastEncTicksR = buffR;

if (idxL > ENCDELTAARRAY - 1) idxL = 0;
if (idxR > ENCDELTAARRAY - 1) idxR = 0;


//for(int i=0; i<ENCDELTAARRAY; i++) {
//errorHandler.setInfoNoLog(F("%d  %d  %d\r\n"),encDeltaL[i], encDeltaR[i], encDeltaL[i] - encDeltaR[i]);
//}
//errorHandler.setInfoNoLog(F("==============\r\n"));


for (int i = 0; i < ENCDELTAARRAY; i++) {
	sumL += encDeltaL[i];
	sumR += encDeltaR[i];
}

//The average angle the robot has traveled
//theta = (LeftEncoderDistance−RightEncoderDistance) / wheelbase
//theta = (36.8 - 27.97) / 3 = 2.95 radians
//1 Grad = 0,017453292519943295769236907684886 Bogenmaß (radiant) 1/0,017... = 57,2957795  => angle=(360/(2Pi))*Rad
//theta = 2.95/0.017453292 oder 2.95*57.2957795
float cmL = bb.motor.pcL->getCMForCounts(sumL);
float cmR = bb.motor.pcR->getCMForCounts(sumR);
angle = (cmL - cmR)  * 57.2957795f / CONF_DISTANCE_BETWEEN_WHEELS_CM;
//errorHandler.setInfoNoLog(F("Winkel: %f cmL %f cmR %f\r\n"), angle, cmL, cmR);
		}


		// Check for left curve / trianle / second left curve
		switch (state)
		{
		case 0:  // search for left curve
			if (angle < -40) { // -45 found out during test by try and error

				if (flagShowFindTriangleStates) {
					errorHandler.setInfo(F("!03,s0 Left turn found angle: %f ms: %lu\r\n"), angle, millis());
				}
				bb.motor.startDistanceMeasurementTriangle();
				state = 1; // Activate triangle searching
			}
			break;
		case 1: // wait for DD_ROTATECW is activated; Stop waiting if distance is to high
			distance = bb.motor.getDistanceInCMForTriangle();
			if (distance > 50) {
				state = 0;
				if (flagShowFindTriangleStates) {
					errorHandler.setInfo(F("!03,s1 set state = 0 distance %f > 50 ms: %lu\r\n"), distance, millis());
				}
			}

			else if (bb.driveDirection == DD_ROTATECW) { // RotateCW will be activated by lineFollow
				state = 2;
				if (flagShowFindTriangleStates) {
					errorHandler.setInfo(F("!03,s1 start RotateCW set state = 2 distance %f < 50\r\n"), distance);
				}
				// array durchlaufen und alles löschen wo L<R ist damit letzte linkskurve die rechts winkelmessung nicht beeinflusst
				for (int i = 0; i < ENCDELTAARRAY; i++) {
					if (encDeltaL[i] < encDeltaR[i]) {
						encDeltaL[i] = 0;
						encDeltaR[i] = 0;
					}
				}
			}

			break;

		case 2:  // Wait that RotateCW has finished
			if (bb.flagLFPerCrossedInsideOutside == true) { // Will be set by lineFollow if mower crosses perimeter from inside to outside
				bb.flagLFPerCrossedInsideOutside = false; // reset flag
				if (angle > 70) { // Rotated CW
					state = 3;
					if (flagShowFindTriangleStates) {
						errorHandler.setInfo(F("!03,s2 CrossedInsideOutside set state=3 angle %f\r\n"), angle);
					}
					//debug->printf("bb.setBehaviour(BH_FINDPERIMETER);\r\n");
				}
				else {
					state = 0;
					if (flagShowFindTriangleStates) {
						errorHandler.setInfo(F("!03,s2 CrossedInsideOutside reset state=0 angle %f\r\n"), angle);
					}
				}
				memset(&encDeltaL[0], 0, ENCDELTAARRAY * sizeof(int));
				memset(&encDeltaR[0], 0, ENCDELTAARRAY * sizeof(int));
			}
			break;

		case 3:  // wait for second left curve
			distance = bb.motor.getDistanceInCMForTriangle();
			if (angle < -35) { // -50 found out during test by try and error
				memset(&encDeltaL[0], 0, ENCDELTAARRAY * sizeof(int));
				memset(&encDeltaR[0], 0, ENCDELTAARRAY * sizeof(int));
				if (flagShowFindTriangleStates) {
					errorHandler.setInfo(F("!03,s3 Second Left turn found set state=0 angle: %f distance %f\r\n"), angle, distance);
					errorHandler.setInfo(F("!03,s3 Cross Lawn Activated\r\n"));
				}
				state = 0;
				//bb.motor.enableDefaultRamping();
				return BH_SUCCESS;

			}
			if (distance > 140) { // Distance greater than 140cm from first left curve
				state = 0;
				if (flagShowFindTriangleStates) {
					errorHandler.setInfo(F("!03,Second Left turn search canceled set reset state=0  distance %f\r\n"), distance);
				}

			}
			break;

		default:
			errorHandler.setError(F("!03,TtrackPerimeter state %d not found\r\n"), state);
			break;
		}

		return BH_RUNNING;
	}


	virtual void onTerminate(NodeStatus status, Blackboard& bb) {
		//debug->printf("onTerminate TFLfollowLine enableDefaultRamping()\r\n" );
		//bb.motor.enableDefaultRamping();
	}

};
*/

/**********************************
class TFLfollowLine: public Node
{
private:

	double Setpoint, Amplitude, Output; // Pid setpoint, input und oputput
	double last_error, integral;
	int counter;

public:
	//PIDPOS myPID;
	double Kp, Ki, Kd;
	unsigned long nextTimeMotorPerimeterControl;

	TFLfollowLine() {
		Kp = 0.0021f; //0.003f 21
		Ki = 0.00013f; //0.00004f;0.000168f000247
		Kd = 0.00446f; //0.003f0.0065625f
		Setpoint = 0.0f;
		Amplitude = 0.0f;
		Output = 0.0f; // Pid setpoint, input und oputput

		nextTimeMotorPerimeterControl = 0;
		last_error=0;
		integral = 0;
	}

	virtual void onInitialize(Blackboard& bb) {
		Output = 0;
		Setpoint = bb.perimeterSensoren.magLineFollow;
		//myPID.Initialize();
		nextTimeMotorPerimeterControl = 0;
		last_error=0;
		integral = 0;
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {
		if (millis() < nextTimeMotorPerimeterControl) return BH_RUNNING;
		nextTimeMotorPerimeterControl = millis() + 100;

		Amplitude =  bb.perimeterSensoren.magnetudeB;
		if(Amplitude < 0)
			Amplitude *= 1.34f; // 1.34f;1,5

		double error = Amplitude;
		double derivate = error-last_error;
		integral = integral + (Ki*error);

		//debug->printf("error: %f\r\n",error);


		//Set integral to 0 if crossing the line
		if (sign0minus(error) != sign0minus(integral)) {
			integral = 0;  //sign0minus => 0 belongs to minus
		}

		//     debug->printf("trans: %lu\r\n",millis()-bb.perimeterSensoren.perimeterLastTransitionTimeB);

		//      if(integral > bb.LINEFOLLOW_SPEED_LOW) integral = bb.LINEFOLLOW_SPEED_LOW;
		//      if(integral < -1*bb.LINEFOLLOW_SPEED_LOW) integral = -1*bb.LINEFOLLOW_SPEED_LOW;

		Output = Kp  * error + integral + Kd * derivate ;


		//if(millis()-bb.perimeterSensoren.perimeterLastTransitionTimeB > 900){
		//    Output *=1.3;
		//}


		//debug->printf("p:%f i%f d:%f ",Kp * error , integral, Kd * derivate);
		//debug->printf("output: %f\r\n",Output);

		if(Output > bb.LINEFOLLOW_SPEED_LOW) Output = bb.LINEFOLLOW_SPEED_LOW;
		if(Output < -1*bb.LINEFOLLOW_SPEED_LOW) Output = -1*bb.LINEFOLLOW_SPEED_LOW;

		last_error = error;

		bb.cruiseSpeed = bb.LINEFOLLOW_SPEED_LOW;
		bb.driveDirection = DD_REVERSE_LINE_FOLLOW;

		if(error>0.0f) {
			bb.motor.L->setSpeed(-(bb.cruiseSpeed - Output));
			bb.motor.R->setSpeed(-(bb.cruiseSpeed));
		} else {
			bb.motor.L->setSpeed(-(bb.cruiseSpeed+5));
			bb.motor.R->setSpeed(-(bb.cruiseSpeed + Output));
		}


		return BH_RUNNING;
	}
};

********************************/

/**********************************
class TFLfollowLine: public Node
{
private:

	double Setpoint, Amplitude, Output; // Pid setpoint, input und oputput
	double last_error, integral;
	int amplitudenVerlauf[10];
	int counter;

public:
	//PIDPOS myPID;
	double Kp, Ki, Kd;
	unsigned long nextTimeMotorPerimeterControl;

	TFLfollowLine() {
		Kp = 18;
		Ki = 1.2;
		Kd = 0;
		Setpoint = 0.0f;
		Amplitude = 0.0f;
		Output = 0.0f; // Pid setpoint, input und oputput

		nextTimeMotorPerimeterControl = 0;
		last_error=0;
		integral = 0;
		memset(&amplitudenVerlauf[0], 0, 10*sizeof(int));
	}

	virtual void onInitialize(Blackboard& bb) {
		Output = 0;
		Setpoint = bb.perimeterSensoren.magLineFollow;
		//myPID.Initialize();
		nextTimeMotorPerimeterControl = 0;
		last_error=0;
		integral = 0;
		memset(&amplitudenVerlauf[0], 0, 10*sizeof(int));

	}

	virtual NodeStatus onUpdate(Blackboard& bb) {
		if (millis() < nextTimeMotorPerimeterControl) return BH_RUNNING;
		nextTimeMotorPerimeterControl = millis() + 100;

		Amplitude =  bb.perimeterSensoren.magnetudeB;
		if(Amplitude <= 0){
			Amplitude =-1;
		}else{
			Amplitude = 1;
		}

		double error = Amplitude;
		double derivate = error-last_error;
		integral = integral + (Ki*error);

		//debug->printf("error: %f\r\n",error);


		//Set integral to 0 if crossing the line
		if (sign0minus(error) != sign0minus(integral)) {
			integral = 0;  //sign0minus => 0 belongs to minus
		}

		//     debug->printf("trans: %lu\r\n",millis()-bb.perimeterSensoren.perimeterLastTransitionTimeB);

		//      if(integral > bb.LINEFOLLOW_SPEED_HIGH) integral = bb.LINEFOLLOW_SPEED_HIGH;
		//      if(integral < -1*bb.LINEFOLLOW_SPEED_HIGH) integral = -1*bb.LINEFOLLOW_SPEED_HIGH;

		Output = Kp  * error + integral + Kd * derivate ;


		//if(millis()-bb.perimeterSensoren.perimeterLastTransitionTimeB > 900){
		//    Output *=1.3;
		// }



		debug->printf("p:%f i%f d:%f o:%f\r\n",Kp * error , integral, Kd * derivate, Output);


		if(Output > bb.LINEFOLLOW_SPEED_HIGH) Output = bb.LINEFOLLOW_SPEED_HIGH;
		if(Output < -1*bb.LINEFOLLOW_SPEED_HIGH) Output = -1*bb.LINEFOLLOW_SPEED_HIGH;

		last_error = error;

		bb.cruiseSpeed = bb.LINEFOLLOW_SPEED_HIGH;
		bb.driveDirection = DD_REVERSE_LINE_FOLLOW;

		if(error>0.0f) {
			bb.motor.L->setSpeed(-(bb.cruiseSpeed - Output));
			bb.motor.R->setSpeed(-(bb.cruiseSpeed));
		} else {
			bb.motor.L->setSpeed(-(bb.cruiseSpeed+5));
			//bb.motor.R->setSpeed(-(bb.cruiseSpeed -30));
			bb.motor.R->setSpeed(-(bb.cruiseSpeed -40));
		}


		return BH_RUNNING;
	}
};

*/


/**********************************


class TFLfollowLine: public Node
{
private:

	double Setpoint, Amplitude, Output; // Pid setpoint, input und oputput
	double last_error, integral;
	int amplitudenVerlauf[10];
	int counter;

public:
	//PIDPOS myPID;
	double Kp, Ki, Kd;
	unsigned long nextTimeMotorPerimeterControl;

	TFLfollowLine() {
		Kp = 0.0021f; //0.003f 21
		Ki = 0.00013f; //0.00004f;0.000168f000247
		Kd = 0.00446f; //0.003f0.0065625f
		Setpoint = 0.0f;
		Amplitude = 0.0f;
		Output = 0.0f; // Pid setpoint, input und oputput

		nextTimeMotorPerimeterControl = 0;
		last_error=0;
		integral = 0;
		memset(&amplitudenVerlauf[0], 0, 10*sizeof(int));
	}

	virtual void onInitialize(Blackboard& bb) {
		Output = 0;
		Setpoint = bb.perimeterSensoren.magLineFollow;
		//myPID.Initialize();
		nextTimeMotorPerimeterControl = 0;
		last_error=0;
		integral = 0;
		memset(&amplitudenVerlauf[0], 0, 10*sizeof(int));

	}

	virtual NodeStatus onUpdate(Blackboard& bb) {
		if (millis() < nextTimeMotorPerimeterControl) return BH_RUNNING;
		nextTimeMotorPerimeterControl = millis() + 100;

		Amplitude =  bb.perimeterSensoren.magnetudeB;
		if(Amplitude < 0)
			Amplitude *= 1.5f; // 1.34f;

		double error = Amplitude;
		double derivate = error-last_error;
		integral = integral + (Ki*error);

		//debug->printf("error: %f\r\n",error);

		//memcpy(destination, source, number_of_bytes).
 //       memcpy(&amplitudenVerlauf[0],&amplitudenVerlauf[1],sizeof(int) * (10-1));
 //       amplitudenVerlauf[9]= bb.perimeterSensoren.magnetudeB;

 //        for(int i=0; i<10; i++) {
 //           debug->printf("error: %d\r\n",amplitudenVerlauf[i]);
 //       }
 //       debug->printf("==============\r\n");


 //       counter = 0;

 //       for(int i=0; i<9; i++) {
 //           if( amplitudenVerlauf[i]< 0) {
 //               if ( amplitudenVerlauf[i] < amplitudenVerlauf[i+1]) {
 //                   counter++;
 //               } else {
 //                  if(counter < 5)
 //                       counter = 0;
 //               }
 //           } else { // Positive amplitude gefunden
 //               counter = 0;
 //               break;
 //           }
 //       }

 //       debug->printf("counter: %d\r\n",counter);



		//Set integral to 0 if crossing the line
		if (sign0minus(error) != sign0minus(integral)) {
			integral = 0;  //sign0minus => 0 belongs to minus
		}

		//     debug->printf("trans: %lu\r\n",millis()-bb.perimeterSensoren.perimeterLastTransitionTimeB);

		//      if(integral > bb.LINEFOLLOW_SPEED_HIGH) integral = bb.LINEFOLLOW_SPEED_HIGH;
		//      if(integral < -1*bb.LINEFOLLOW_SPEED_HIGH) integral = -1*bb.LINEFOLLOW_SPEED_HIGH;

		Output = Kp  * error + integral + Kd * derivate ;


		//if(millis()-bb.perimeterSensoren.perimeterLastTransitionTimeB > 900){
		//    Output *=1.3;
		//}


		//debug->printf("p:%f i%f d:%f ",Kp * error , integral, Kd * derivate);
		//debug->printf("output: %f\r\n",Output);

		if(Output > bb.LINEFOLLOW_SPEED_HIGH) Output = bb.LINEFOLLOW_SPEED_HIGH;
		if(Output < -1*bb.LINEFOLLOW_SPEED_HIGH) Output = -1*bb.LINEFOLLOW_SPEED_HIGH;

		last_error = error;

		bb.cruiseSpeed = bb.LINEFOLLOW_SPEED_HIGH;
		bb.driveDirection = DD_REVERSE_LINE_FOLLOW;

		if(error>0.0f) {
			bb.motor.L->setSpeed(-(bb.cruiseSpeed - Output));
			bb.motor.R->setSpeed(-(bb.cruiseSpeed));
		} else {
			bb.motor.L->setSpeed(-(bb.cruiseSpeed));
			//bb.motor.R->setSpeed(-(bb.cruiseSpeed -30));
			bb.motor.R->setSpeed(-(bb.cruiseSpeed + Output));
		}


		return BH_RUNNING;
	}
};

***********************************/

#endif

