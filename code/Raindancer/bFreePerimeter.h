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

#ifndef BH_PERIMETER_H
#define BH_PERIMETER_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "Blackboard.h"
#include "BehaviourTree.h"



class TSetflagCoilFirstOutsideLatched : public Node    // Each task will be a class (derived from Node of course).
{
private:

public:

	TSetflagCoilFirstOutsideLatched() {}

	virtual void onInitialize(Blackboard& bb) {
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (bb.perimeterSensoren.isLeftInside() && bb.perimeterSensoren.isRightInside()) {
			bb.flagCoilFirstOutsideLatched = CO_NONE;
			//errorHandler.setInfo(F("flagCoilFirstOutsideLatched = CO_NONE"));
		}

		if (bb.flagCoilFirstOutsideLatched == CO_NONE) { //Latch first coil outside until both coils are inside
			if (bb.perimeterSensoren.isLeftOutside() && bb.perimeterSensoren.isRightOutside()) {
				bb.flagCoilFirstOutsideLatched = CO_BOTH;
				sprintf(errorHandler.msg, "!03,->%s flagCoilFirstOutsideLatched = CO_BOTH\r\n", nodeName);
				if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
				else { errorHandler.writeToLogOnly(); }
			}
			else if (bb.perimeterSensoren.isLeftOutside()) {
				bb.flagCoilFirstOutsideLatched = CO_LEFT;
				sprintf(errorHandler.msg, "!03,->%s flagCoilFirstOutsideLatched = CO_LEFT\r\n", nodeName);
				if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
				else { errorHandler.writeToLogOnly(); }
			}
			else if (bb.perimeterSensoren.isRightOutside()) {
				bb.flagCoilFirstOutsideLatched = CO_RIGHT;
				sprintf(errorHandler.msg, "!03,->%s flagCoilFirstOutsideLatched= CO_RIGHT\r\n", nodeName);
				if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
				else { errorHandler.writeToLogOnly(); }
			}
		}

		return BH_FAILURE;
	}


};




class TSetflagCoilFirstOutside : public Node    // Each task will be a class (derived from Node of course).
{
private:

public:

	TSetflagCoilFirstOutside() {}

	virtual void onInitialize(Blackboard& bb) {
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		bb.flagCoilFirstOutside = bb.flagCoilFirstOutsideLatched;
		return BH_SUCCESS;

		/*
		if (bb.perimeterSensoren.isLeftOutside() && bb.perimeterSensoren.isRightOutside()) {
			bb.flagCoilFirstOutside = CO_BOTH;
			sprintf(errorHandler.msg, "!03,->%s flagCoilFirstOutside = CO_BOTH\r\n", nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}
		else if (bb.perimeterSensoren.isLeftOutside()) {
			bb.flagCoilFirstOutside = CO_LEFT;
			sprintf(errorHandler.msg, "!03,->%s flagCoilFirstOutside = CO_LEFT\r\n", nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}
		else if (bb.perimeterSensoren.isRightOutside()) {
			bb.flagCoilFirstOutside = CO_RIGHT;
			sprintf(errorHandler.msg, "!03,->%s flagCoilFirstOutside= CO_RIGHT\r\n", nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

	  errorHandler.setError(F("TSetflagCoilFirstOutside no coil was outside"));
	return BH_FAILURE;
		*/
		// Routine should never come here

		//bb.flagCoilFirstOutside = CO_BOTH;
		//sprintf(errorHandler.msg, "!03,->%s flagCoilFirstOutside2!! = CO_BOTH\r\n", nodeName);
		//if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
		//else { errorHandler.writeToLogOnly(); }
		//return BH_SUCCESS;



	}


};


class TConditionPerimeterNotFound : public Node    // Each task will be a class (derived from Node of course).
{
private:

public:

	TConditionPerimeterNotFound() {}



	virtual NodeStatus onUpdate(Blackboard& bb) {


		// When not useing zone recognition, then TOverRun changes the drivedirection to DD_FORWARD if both coils inside again and the branch is executed until
		// this node. Therfore this Node will be executed with the driveDirection==DD_Forward.
		// To prevent setting an error, we have to chech this here..
		if (bb.driveDirection == DD_FORWARD) {
			return BH_SUCCESS;
		}
		errorHandler.setError("!03,TConditionPerimeterNotFound not found %s\r\n", enuDriveDirectionString[bb.driveDirection]);
		return BH_SUCCESS;
	}


};

class TOverRun : public Node    // Each task will be a class (derived from Node of course).
{
private:
	float weg;
public:

	TOverRun() {}

	virtual void onInitialize(Blackboard& bb) {

		bb.motor.startDistanceMeasurementCoilOut();
		//errorHandler.setInfoNoLog(F("CurSpeed at perimeter: %d\r\n"), bb.cruiseSpeed);
		//bb.motor.startDistanceMeasurementOverRun();
		bb.motor.stopPCAtPerimeter();
		bb.driveDirection = DD_OVERRUN;

		bb.addHistoryEntry(bb.driveDirection, 0.0f, 0.0f, 0.0f, FRD_NONE, bb.flagCoilFirstOutside);
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (getTimeInNode() > 10000) {
			errorHandler.setError("!03,Runover too long in node\r\n");
		}


		if (bb.perimeterSensoren.isLeftOutside()) {
			bb.motor.stopDistanceMeasurementLCoilOut();
		}

		if (bb.perimeterSensoren.isRightOutside()) {
			bb.motor.stopDistanceMeasurementRCoilOut();
		}

		bb.history[0].distanceDriven = bb.motor.getDistanceInCM();

		if (bb.motor.isPositionReached()) {

			bb.coilsOutsideAngle = bb.motor.getDistanceAngleCoilOut();
			//float drivenDistance = bb.motor.getDistanceInCMForOverRun();
			//errorHandler.setInfoNoLog(F("drivenDistance: %f\r\n"), drivenDistance);

/*
			if (bb.perimeterSensoren.isLeftOutside() && bb.perimeterSensoren.isRightOutside()) {
				//bb.driveDirection =  DD_FORWARD;
				//return BH_FAILURE;
				bb.flagCoilOutsideAfterOverrun = CO_BOTH;

			}
			else if (bb.perimeterSensoren.isLeftOutside()) {
				bb.flagCoilOutsideAfterOverrun = CO_LEFT;
			}
			else if (bb.perimeterSensoren.isRightOutside()) {
				bb.flagCoilOutsideAfterOverrun = CO_RIGHT;
			}
			*/


			if (CONF_USE_ZONE_RECOGNITION == false) { // When useing zone recognition, don't drive further.
				if (bb.perimeterSensoren.isLeftInside() && bb.perimeterSensoren.isRightInside()) {
					bb.driveDirection = DD_FORWARD;
					return BH_FAILURE;
				}
			}

			return BH_SUCCESS;
		}
		return BH_RUNNING;
	}

};


class TRunTempService : public Node    // Each task will be a class (derived from Node of course).
{
  private:

  public:

	  TRunTempService() {}

    virtual void onInitialize(Blackboard& bb) {  // executed each time the node is call
     
    }

    virtual NodeStatus onUpdate(Blackboard& bb) {

	  bb.dht.run();
      return BH_SUCCESS;

    }
};




class TReverseInside : public Node    // Each task will be a class (derived from Node of course).
{

public:
	bool alreadyInside;

	TReverseInside() {}

	virtual void onInitialize(Blackboard& bb) {

		if (bb.perimeterSensoren.isLeftInside() && bb.perimeterSensoren.isRightInside()) {
			alreadyInside = true;
			//bb.motor.startDistanceMeasurementCoilOut(true);
			return;
		}

		alreadyInside = false;
		bb.cruiseSpeed = bb.CRUISE_SPEED_LOW;

		bb.motor.rotateCM(-70, -bb.cruiseSpeed);
		bb.driveDirection = DD_REVERSE_INSIDE;
		//sprintf(errorHandler.msg,"!03,#%s\r\n",nodeName);
		//errorHandler.setInfo();
		/*
		if (bb.perimeterSensoren.isLeftOutside() && bb.perimeterSensoren.isRightOutside()) {
			bb.flagCoilOutsideAfterOverrun = CO_BOTH;
			bb.motor.startDistanceMeasurementCoilOut(true);
		}
		else if (bb.perimeterSensoren.isLeftOutside()) {
			bb.flagCoilOutsideAfterOverrun = CO_LEFT;
			bb.motor.startDistanceMeasurementCoilOut(false);
		}
		else if (bb.perimeterSensoren.isRightOutside()) {
			bb.flagCoilOutsideAfterOverrun = CO_RIGHT;
			bb.motor.startDistanceMeasurementCoilOut(false);
		}
		*/


	}


	virtual NodeStatus onUpdate(Blackboard& bb) {

		// Wenn zeit in state > 10 Sek dann fehler oder anhalten undnochmal checken.
		if (getTimeInNode() > 10000) {
			sprintf(errorHandler.msg, "!03,->%s too long in state\r\n", nodeName);
			errorHandler.setError();
		}

		/*
		if (bb.perimeterSensoren.isLeftInside()) {
			bb.motor.stopDistanceMeasurementLCoilOut();
		}

		if (bb.perimeterSensoren.isRightInside()) {
			bb.motor.stopDistanceMeasurementRCoilOut();
		}
		*/


		//errorHandler.setInfoNoLog(F("ri speed: %f%%\r\n"), bb.motor.L->getCurrentSpeedInPerc());

		if (alreadyInside == true) {
			return BH_SUCCESS;
		}

		if (bb.perimeterSensoren.isLeftInside() && bb.perimeterSensoren.isRightInside()) {
			return BH_SUCCESS;
		}

		if (getTimeInNode() > 3000) { // Fall nach 3 Sekunden noch nicht inside, dann Kurve einleiten

			/*            if (bb.perimeterSensoren.isLeftInside() && bb.perimeterSensoren.isRightOutside()) {
				bb.motor.L->setSpeed(-(bb.cruiseSpeed-13));
				bb.motor.R->setSpeed(-(bb.cruiseSpeed+13));
				//sprintf(errorHandler.msg,"!03,->%s >3000\r\n",nodeName);
				//errorHandler.setInfo();
			}
			if (bb.perimeterSensoren.isLeftOutside() && bb.perimeterSensoren.isRightInside()) {
				bb.motor.L->setSpeed(-(bb.cruiseSpeed+13));
				bb.motor.R->setSpeed(-(bb.cruiseSpeed-13));
				//sprintf(errorHandler.msg,"!03,->%s >3000\r\n",nodeName);
				//errorHandler.setInfo();
			}
			*/

		}



		return BH_RUNNING;
	}

};


class TReverseFurtherInside : public Node
{
private:
	long weg;
public:

	TReverseFurtherInside() {}

	virtual void onInitialize(Blackboard& bb) {


		bb.cruiseSpeed = bb.CRUISE_SPEED_LOW;
		//bb.motor.rotateCM(-5,bb.cruiseSpeed); // 5cm zurueckfahren
		bb.motor.rotateCM(-10, bb.cruiseSpeed); // 5cm zurueckfahren

	}

	virtual NodeStatus onUpdate(Blackboard& bb) {
		if (getTimeInNode() > 5000) {
			errorHandler.setError("!03,reverseFurtherInside too long in state\r\n");
		}

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


class TPerDriveBack : public Node
{
private:
	long weg;
public:

	TPerDriveBack() {}

	virtual void onInitialize(Blackboard& bb) {

		// if 0 cm then do nothing
		if (CONF_PERIMETER_DRIVE_BACK_CM < 0.1f) {
			return;
		}

		if (CONF_USE_ZONE_RECOGNITION == false){
			if (bb.coilsOutsideAngle > CONF_PERIMETER_DRIVE_BACK_ANGLE) {
				return;
			}
			bb.cruiseSpeed = bb.CRUISE_SPEED_LOW;
			bb.motor.rotateCM(-CONF_PERIMETER_DRIVE_BACK_CM, bb.cruiseSpeed); // x cm zurueckfahren
			bb.driveDirection = DD_REVERSE_ESC_OBST; ; // DD_REVERSE_INSIDE;
		}

		if (CONF_USE_ZONE_RECOGNITION == true) {
			if (bb.perimeterSensoren.isLeftInside() && bb.perimeterSensoren.isRightInside()) {
				// When using zone recoginition, it could be that the mower runs over the 13cm zone boarder with both coils. If this happend, drive back 10 more cm.
				bb.cruiseSpeed = bb.CRUISE_SPEED_LOW;
				bb.motor.rotateCM(-CONF_PERIMETER_DRIVE_BACK_CM - 10, bb.cruiseSpeed); // x cm zurueckfahren
				bb.driveDirection = DD_REVERSE_ESC_OBST; ; // DD_REVERSE_INSIDE;
			}
			else {
				// if not both coils inside again, drive like  CONF_USE_ZONE_RECOGNITION == false
				if (bb.coilsOutsideAngle > CONF_PERIMETER_DRIVE_BACK_ANGLE) {
					return;
				}
				bb.cruiseSpeed = bb.CRUISE_SPEED_LOW;
				bb.motor.rotateCM(-CONF_PERIMETER_DRIVE_BACK_CM, bb.cruiseSpeed); // x cm zurueckfahren
				bb.driveDirection = DD_REVERSE_ESC_OBST; ; // DD_REVERSE_INSIDE;
			}
		}


		//bb.addHistoryEntry(bb.driveDirection, 0.0f, 0.0f, 0.0f, FRD_NONE, bb.flagCoilFirstOutside);

	}

	virtual NodeStatus onUpdate(Blackboard& bb) {
		if (getTimeInNode() > 8000) {
			errorHandler.setError("!03,TPerDriveBack too long in state\r\n");
		}

		//bb.history[0].distanceDriven = bb.motor.getDistanceInCM();

		// if 0 cm then do nothing
		if (CONF_PERIMETER_DRIVE_BACK_CM < 0.1f) {
			return BH_SUCCESS;
		}

		if (bb.coilsOutsideAngle > CONF_PERIMETER_DRIVE_BACK_ANGLE) {
			return BH_SUCCESS;
		}

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

class TMotorStop : public Node
{
private:

public:

	TMotorStop() {}

	virtual void onInitialize(Blackboard& bb) {
		bb.motor.enableDefaultRamping();
		bb.motor.stopPC();
		//errorHandler.setError("!03,onInitializeTMotorStop\r\n");
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (getTimeInNode() > 10000) {
			errorHandler.setError("!03,motorStop too long in state\r\n");
		}

		if (bb.motor.isPositionReached()) {
			bb.cruiseSpeed = 0;
			bb.history[0].rotAngleIst = bb.motor.getAngleRotatedAngleDeg();
			bb.history[0].distanceDriven = bb.motor.getDistanceInCM();
			//errorHandler.setInfo("!03,motorStopped\r\n");
			return BH_SUCCESS;
		}

		return BH_RUNNING;
	}
};


class TRotateBackCW : public Node
{
private:
	long distance;
public:

	TRotateBackCW() {}

	virtual void onInitialize(Blackboard& bb) {

		distance = bb.motor.getAngleRotatedDistanceCM();
		bb.driveDirection = DD_REVERSE_ESC_OBST;
		bb.cruiseSpeed = bb.CRUISE_SPEED_LOW;
		bb.motor.rotateCM(distance, -distance, bb.cruiseSpeed);
		if (bb.flagShowRotateX) {
			errorHandler.setInfo(F("!03,TRotateBackCW escape to DD_ROTATECC\r\n"));
		}

		// Delete last rotation in history because we restore it here
		bb.deleteLastHistoryEntry();

		/*
		bb.addHistoryEntry(bb.driveDirection, 0.0f, bb.arcRotateXArc, 0.0f, bb.flagForceRotateDirection, bb.flagCoilFirstOutside);
		// Set this rotation as restored because we we don't want it to restore again
		bb.markLastHistoryEntryAsRestored();
		*/
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {
		if (getTimeInNode() > 10000) {
			errorHandler.setError("!03,TRotateBackCW too long in state\r\n");
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

class TRotateBackCC : public Node
{
private:
	long distance;
public:

	TRotateBackCC() {}

	virtual void onInitialize(Blackboard& bb) {

		distance = bb.motor.getAngleRotatedDistanceCM();
		bb.driveDirection = DD_REVERSE_ESC_OBST;
		bb.cruiseSpeed = bb.CRUISE_SPEED_LOW;
		bb.motor.rotateCM(-distance, distance, bb.cruiseSpeed);
		if (bb.flagShowRotateX) {
			errorHandler.setInfo(F("!03,TRotateBackCC escape to DD_ROTATECC\r\n"));
		}

		// Delete last rotation in history because we restore it here
		bb.deleteLastHistoryEntry();

		//bb.addHistoryEntry(bb.driveDirection, 0.0f, bb.arcRotateXArc, 0.0f, bb.flagForceRotateDirection, bb.flagCoilFirstOutside);

		// Set this rotation as restored because we we don't want it to restore again
		//bb.markLastHistoryEntryAsRestored();


	}

	virtual NodeStatus onUpdate(Blackboard& bb) {
		if (getTimeInNode() > 10000) {
			errorHandler.setError("!03,TRotateBackCC too long in state\r\n");
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

/*
class TPerRotateInsideCW : public Node
{
private:

public:

	TPerRotateInsideCW() {}

	virtual void onInitialize(Blackboard& bb) {


		bb.cruiseSpeed = bb.CRUISE_SPEED_OBSTACLE;
		bb.motor.turnTo(380, bb.cruiseSpeed);
		bb.driveDirection = DD_ROTATECW;
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (getTimeInNode() > 15000) {
			errorHandler.setError("!03,PerRotateInsideCW  too long in state\r\n");
		}


		if (bb.perimeterSensoren.isLeftInside() && bb.perimeterSensoren.isRightInside()) { // wenn beide coils innen dann weitermÃ¤hen
			//bb.motor.pcL->reset(); // Mower fÃ¤hrt weiter in normalem MÃ¤hbetrieb. Daher Posisioncontrol deaktivieren. Robbi dreht weiter und wird durch cruise wieder gerade gelenkt.
			//bb.motor.pcR->reset();
			bb.flagForceRotateDirection = FRD_CW;
			bb.flagForceSmallRotAngle = 1;
			return BH_SUCCESS;
		}

		if (bb.motor.isPositionReached()) {
			errorHandler.setError("!03,PerRotateInsideCWnot able to rotate both coils to inside\r\n");
		}

		return BH_RUNNING;
	}

	virtual void onTerminate(NodeStatus status, Blackboard& bb) {

				if(status != BH_ABORTED) {

				}

	}
};
*/

/*
class TPerRotateInsideCC : public Node
{
private:

public:

	TPerRotateInsideCC() {}

	virtual void onInitialize(Blackboard& bb) {

		bb.cruiseSpeed = bb.CRUISE_SPEED_OBSTACLE;

		bb.motor.turnTo(-380, bb.cruiseSpeed);
		bb.driveDirection = DD_ROTATECC;

	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (getTimeInNode() > 15000) {
			errorHandler.setError("!03,PerRotateInsideCC  too long in state\r\n");
		}


		if (bb.perimeterSensoren.isLeftInside() && bb.perimeterSensoren.isRightInside()) { // wenn beide coils innen dann weitermÃ¤hen
			//bb.motor.pcL->reset(); // Mower fÃ¤hrt weiter in normalem MÃ¤hbetrieb. Daher Posisioncontrol deaktivieren. Robbi dreht weiter und wird durch cruise wieder gerade gelenkt.
			//bb.motor.pcR->reset();
			bb.flagForceRotateDirection = FRD_CC;
			bb.flagForceSmallRotAngle = 1;
			return BH_SUCCESS;
		}

		if (bb.motor.isPositionReached()) {
			errorHandler.setError("!03,PerRotateInsideCC not able to rotate both coils to inside\r\n");
		}

		return BH_RUNNING;
	}

	virtual void onTerminate(NodeStatus status, Blackboard& bb) {

				if(status != BH_ABORTED) {

				}

	}

};
*/



/*
class TRotateOtherDirection: public Node    // Each task will be a class (derived from Node of course).
{
private:
	bool flagRotationFurther;
	unsigned long lastTimeCalled;
	int state;
	int arcRotate;
	int count;

public:

	TRotateOtherDirection():lastTimeCalled(0),state(0), arcRotate(0), count(0)  {}

	virtual void onInitialize(Blackboard& bb) {

		unsigned long now = millis();
		if( now-lastTimeCalled < 15000) {

			if(count > 2) {
				count = 0;
			} else {
				flagRotationFurther = true;
				arcRotate += 50;
				count++;
			}

		} else {
			flagRotationFurther = false;
			arcRotate = 0;
			count = 0;
		}

		bb.cruiseSpeed = bb.CRUISE_SPEED_OBSTACLE;

		if (bb.driveDirection ==  DD_ROTATECC) {
			bb.motor.turnTo(380,bb.cruiseSpeed);
			bb.driveDirection =  DD_ROTATECW;
		} else {
			bb.motor.turnTo(-380,bb.cruiseSpeed);
			bb.driveDirection =  DD_ROTATECC;
		}

		state = 0;
		lastTimeCalled = millis();
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if(getTimeInNode()> 15000) {
			bb.errorHandler.setError("!03,RotateOtherDirection  too long in state\r\n");
		}

		if(state == 0) {
			if ( bb.perimeterSensoren.isLeftInside() && bb.perimeterSensoren.isRightInside() && flagRotationFurther == false) { // wenn beide coils innen dann weitermÃ¤hen
				bb.motor.pcL->reset(); // Mower fÃ¤hrt weiter in normalem MÃ¤hbetrieb. Daher Posisioncontrol deaktivieren. Robbi dreht weiter und wird durch cruise wieder gerade gelenkt.
				bb.motor.pcR->reset();
				return BH_SUCCESS;
			}

			if ( bb.perimeterSensoren.isLeftInside() && bb.perimeterSensoren.isRightInside() && flagRotationFurther == true) { // wenn beide coils innen aber innerhlab von 15sek erneut aufgerufen, dann nochmal arcRotate weiterdrehen
				if (bb.driveDirection ==  DD_ROTATECC) {
					bb.motor.turnTo(-1* arcRotate,bb.cruiseSpeed);
				} else {
					bb.motor.turnTo(arcRotate,bb.cruiseSpeed);
				}
				state = 1;
			}
		}

		if(state == 1) {
			if (bb.motor.isPositionReached()) {
				state = 0;
				return BH_SUCCESS;
			}
		}

		if(state == 0) {
			if (bb.motor.isPositionReached()) {
				bb.errorHandler.setError("!03,bMow STBH_ROTATE_OTHERDIRECTION not able to rotate both coils to inside\r\n");
			}
		}


		return BH_RUNNING;
	}


};
*/

class TForwardInsideError : public Node    // Each task will be a class (derived from Node of course).
{

public:

	TForwardInsideError() {}

	virtual void onInitialize(Blackboard& bb) {
	}


	virtual NodeStatus onUpdate(Blackboard& bb) {

		errorHandler.setError("!03,TForwardInsideError  was called\r\n");

		return BH_SUCCESS;
	}

	virtual void onTerminate(NodeStatus status, Blackboard& bb) {

	}

};


class TDirectionOverrunError : public Node    // Each task will be a class (derived from Node of course).
{

public:

	TDirectionOverrunError() {}

	virtual void onInitialize(Blackboard& bb) {
	}


	virtual NodeStatus onUpdate(Blackboard& bb) {

		errorHandler.setError("!03,TDirectionOverrunError  was called\r\n");

		return BH_SUCCESS;
	}

	virtual void onTerminate(NodeStatus status, Blackboard& bb) {

	}

};

class TReverseInsideError : public Node    // Each task will be a class (derived from Node of course).
{

public:

	TReverseInsideError() {}

	virtual void onInitialize(Blackboard& bb) {
	}


	virtual NodeStatus onUpdate(Blackboard& bb) {

		errorHandler.setError("!03,TReverseInsideError  was called\r\n");

		return BH_SUCCESS;
	}

	virtual void onTerminate(NodeStatus status, Blackboard& bb) {

	}

};


class TForwardInside : public Node    // Each task will be a class (derived from Node of course).
{

public:

	TForwardInside() {}

	virtual void onInitialize(Blackboard& bb) {

		bb.driveDirection = DD_FORWARD_INSIDE;
		bb.cruiseSpeed = bb.CRUISE_SPEED_LOW;
		bb.motor.rotateCM(30, bb.cruiseSpeed);
		if (bb.flagShowRotateX) {
			sprintf(errorHandler.msg, "!03,TForwardInside\r\n");
			errorHandler.setInfo();
		}

		bb.addHistoryEntry(bb.driveDirection, 0.0f, 0.0f, 0.0f, FRD_NONE, bb.flagCoilFirstOutside);

	}


	virtual NodeStatus onUpdate(Blackboard& bb) {

		//errorHandler.setError("!03,TForwardInside  was called\r\n");

		// Wenn zeit in state > 10 Sek dann fehler oder anhalten undnochmal checken.
		if (getTimeInNode() > 10000) {
			errorHandler.setError("!03,TForwardInside  too long in state\r\n");
		}

		bb.history[0].distanceDriven = bb.motor.getDistanceInCM();

		if (bb.motor.isPositionReached()) {

			if (bb.perimeterSensoren.isLeftInside() && bb.perimeterSensoren.isRightInside()) {
				return BH_SUCCESS;
			}
			else {
				errorHandler.setError("!03,TForwardInside not able to drive inside\r\n");
			}

		}

		return BH_RUNNING;
	}

	virtual void onTerminate(NodeStatus status, Blackboard& bb) {

	}

};



class TSetArc90CW1 : public Node    // Each task will be a class (derived from Node of course).
{
private:
public:

	TSetArc90CW1() {}

	virtual void onInitialize(Blackboard& bb) {

		if (bb.perimeterSensoren.isRightOutside()) {
			bb.flagForceRotateDirection = FRD_CC;
			bb.driveDirection = DD_ROTATECC1;
			bb.arcRotateXArc = myRandom(90, 135);
			bb.flagDeactivateRotInside = false;

			if (bb.flagShowRotateX) {
				errorHandler.setInfo(F("!05,TSetArc90CW1->CC\r\n"));
			}
		}
		else {
			bb.flagForceRotateDirection = FRD_CW;
			bb.driveDirection = DD_ROTATECW1;
			bb.arcRotateXArc = myRandom(90, 135);
			bb.flagDeactivateRotInside = false;

			if (bb.flagShowRotateX) {
				errorHandler.setInfo(F("!05,TSetArc90CW1\r\n"));
			}
		}

	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		return BH_SUCCESS;
	}

};

class TSetArc90CC1 : public Node    // Each task will be a class (derived from Node of course).
{
private:
public:

	TSetArc90CC1() {}

	virtual void onInitialize(Blackboard& bb) {

		if (bb.perimeterSensoren.isLeftOutside()) {
			bb.flagForceRotateDirection = FRD_CW;
			bb.driveDirection = DD_ROTATECW1;
			bb.arcRotateXArc = myRandom(90, 135);
			bb.flagDeactivateRotInside = false;
			if (bb.flagShowRotateX) {
				errorHandler.setInfo(F("!05,TSetArc90CC->CW1\r\n"));
			}
		}
		else {
			bb.flagForceRotateDirection = FRD_CC;
			bb.driveDirection = DD_ROTATECC1;
			bb.arcRotateXArc = myRandom(90, 135);
			bb.flagDeactivateRotInside = false;
			if (bb.flagShowRotateX) {
				errorHandler.setInfo(F("!05,TSetArc90CC1\r\n"));
			}
		}
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		return BH_SUCCESS;
	}

};

class TSetArc90CW : public Node    // Each task will be a class (derived from Node of course).
{
private:
public:

	TSetArc90CW() {}

	virtual void onInitialize(Blackboard& bb) {

		if (bb.perimeterSensoren.isRightOutside()) {
			bb.flagForceRotateDirection = FRD_CC;
			bb.driveDirection = DD_ROTATECC;
			bb.arcRotateXArc = myRandom(90, 135);
			bb.flagDeactivateRotInside = false;

			if (bb.flagShowRotateX) {
				errorHandler.setInfo(F("!05,TSetArc90CW->CC\r\n"));
			}
		}
		else {
			bb.flagForceRotateDirection = FRD_CW;
			bb.driveDirection = DD_ROTATECW;
			bb.arcRotateXArc = myRandom(90, 135);
			bb.flagDeactivateRotInside = false;

			if (bb.flagShowRotateX) {
				errorHandler.setInfo(F("!05,TSetArc90CW\r\n"));
			}
		}

	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		return BH_SUCCESS;
	}

};


class TSetflagForceSmallRotAngle : public Node    // Each task will be a class (derived from Node of course).
{
private:
public:

	TSetflagForceSmallRotAngle() {}

	virtual void onInitialize(Blackboard& bb) {
		bb.flagForceSmallRotAngle = 2;
		if (bb.flagShowRotateX) {
			errorHandler.setInfo(F("!05,TSetflagForceSmallRotAngle\r\n"));
		}
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		return BH_SUCCESS;
	}
};

class TSetArc90CC : public Node    // Each task will be a class (derived from Node of course).
{
private:
public:

	TSetArc90CC() {}

	virtual void onInitialize(Blackboard& bb) {

		if (bb.perimeterSensoren.isLeftOutside()) {
			bb.flagForceRotateDirection = FRD_CW;
			bb.driveDirection = DD_ROTATECW;
			bb.arcRotateXArc = myRandom(90, 135);
			bb.flagDeactivateRotInside = false;
			if (bb.flagShowRotateX) {
				errorHandler.setInfo(F("!05,TSetArc90CC->CW\r\n"));
			}
		}
		else {
			bb.flagForceRotateDirection = FRD_CC;
			bb.driveDirection = DD_ROTATECC;
			bb.arcRotateXArc = myRandom(90, 135);
			bb.flagDeactivateRotInside = false;
			if (bb.flagShowRotateX) {
				errorHandler.setInfo(F("!05,TSetArc90CC\r\n"));
			}
		}
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		return BH_SUCCESS;
	}

};



#endif

