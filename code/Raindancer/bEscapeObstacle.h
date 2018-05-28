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

#ifndef BH_ESCABEOBSTACLE_H
#define BH_ESCABEOBSTACLE_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "BehaviourTree.h"
#include "config.h"



class TSecondReverse: public Node    // Each task will be a class (derived from Node of course).
{
private:

public:

	TSecondReverse(){}

    virtual void onInitialize(Blackboard& bb) {

		errorHandler.setInfo(F("!03,SecondReverse called\r\n"));
		bb.cruiseSpeed = bb.CRUISE_SPEED_LOW;
		bb.motor.rotateCM(-CONF_BUMPER_SEC_REVERSE_CM, bb.cruiseSpeed);
        bb.driveDirection = DD_REVERSE_ESC_OBST;

		bb.addHistoryEntry(bb.driveDirection, 0.0f, 0.0f, 0.0f, FRD_NONE, CO_NONE);
    }

    virtual NodeStatus onUpdate(Blackboard& bb) {

		bb.history[0].distanceDriven = bb.motor.getDistanceInCM();

        if(getTimeInNode()> 10000) {
            errorHandler.setError(F("!03,SecondReverse too long in state\r\n"));
        }

		if (bb.perimeterSensoren.isLeftOutside() && bb.perimeterSensoren.isRightOutside()) {
			errorHandler.setError(F("!03,SecondReverse both coils outside\r\n"));
			bb.motor.stopPC();
		}

        if (bb.motor.isPositionReached()) {
			errorHandler.setInfo(F("!03,SecondReverse position reached\r\n"));
            return BH_SUCCESS;
        }

        return BH_RUNNING;
		
    }

	virtual void onTerminate(NodeStatus status, Blackboard& bb) {
		bb.flagEnableSecondReverse = false;
	}
};


class TSecondReverse2 : public Node    // Each task will be a class (derived from Node of course).
{
private:
	bool bothCoilsOutside;

public:

	TSecondReverse2() {}

	virtual void onInitialize(Blackboard& bb) {

		bothCoilsOutside = false;
		if ((bb.perimeterSensoren.isLeftOutside() && bb.perimeterSensoren.isRightOutside())) {
			bothCoilsOutside = true;
		}

		errorHandler.setInfo(F("!03,SecondReverse2 called\r\n"));
		bb.cruiseSpeed = bb.CRUISE_SPEED_LOW;
		bb.motor.rotateCM(-CONF_BUMPER_SEC_REVERSE_CM, bb.cruiseSpeed);
		bb.driveDirection = DD_REVERSE_ESC_OBST;

		bb.addHistoryEntry(bb.driveDirection, 0.0f, 0.0f, 0.0f, FRD_NONE, CO_NONE);

	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		bb.history[0].distanceDriven = bb.motor.getDistanceInCM();

		if (getTimeInNode()> 10000) {
			errorHandler.setError(F("!03,SecondReverse2 too long in state\r\n"));
		}


		// Robbi ist rückwärts aus Perimeter gefahren. Wenn allerdings beide coils draußen sind, muss er eigentlich mit
		// dem Heck nach innen zeigen.
		// Kann auftreten, wenn robbi sich wieder zurückdreht über perimeter, weil er an Obstacle gestoßen ist.
		// Nur aufrufen, wenn mindestens eine Coil beim start inside ist => bothCoilsOutside==false
		if ( bb.perimeterSensoren.isLeftOutside() && bb.perimeterSensoren.isRightOutside() && bothCoilsOutside==false){ // && stopActivated == false) {
			errorHandler.setError(F("!03,SecondReverse2 both coils outside 1\r\n"));
			bb.motor.stopPC();
		}
        

		if (bb.motor.isPositionReached()) {
			errorHandler.setInfo(F("!03,SecondReverse2 position reached\r\n"));
			if (bb.perimeterSensoren.isLeftOutside() && bb.perimeterSensoren.isRightOutside()) { 
				errorHandler.setError(F("!03,SecondReverse2 both coils outside at position reached\r\n"));
				bb.motor.stopPC();
			}
			return BH_SUCCESS;
		}

		return BH_RUNNING;

	}

};




class TSecondReverse3 : public Node    // Each task will be a class (derived from Node of course).
{
private:

public:

	TSecondReverse3() {}

	virtual void onInitialize(Blackboard& bb) {

		errorHandler.setInfo(F("!03,SecondReverse3 called\r\n"));
		bb.cruiseSpeed = bb.CRUISE_SPEED_LOW;
		bb.motor.rotateCM(-CONF_BUMPER_SEC_REVERSE_CM, bb.cruiseSpeed);
		bb.driveDirection = DD_REVERSE_ESC_OBST;

		bb.addHistoryEntry(bb.driveDirection, 0.0f, 0.0f, 0.0f, FRD_NONE, CO_NONE);
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		bb.history[0].distanceDriven = bb.motor.getDistanceInCM();

		if (getTimeInNode()> 10000) {
			errorHandler.setError(F("!03,SecondReverse3 too long in state\r\n"));
		}

		if (bb.flagBumperOutsidePerActivated == false) {
			if (bb.perimeterSensoren.isLeftOutside() && bb.perimeterSensoren.isRightOutside()) {
				errorHandler.setInfo(F("!03,SecondReverse3 both coils outside\r\n"));
				bb.motor.stopPC();
				return BH_SUCCESS;
			}
		}

		if (bb.motor.isPositionReached()) {
			errorHandler.setInfo(F("!03,SecondReverse3 position reached\r\n"));
			return BH_SUCCESS;
		}

		return BH_RUNNING;

	}

	virtual void onTerminate(NodeStatus status, Blackboard& bb) {
		bb.flagEnableSecondReverse = false;
	}
};

/*
class TForward20: public Node    // Each task will be a class (derived from Node of course).
{
private:

public:

    TForward20()  {}

    virtual void onInitialize(Blackboard& bb) {
        bb.cruiseSpeed = bb.CRUISE_SPEED_LOW;
        bb.motor.rotateAngle(5,bb.cruiseSpeed); // 20 grad vorwÃ¤rtsfahren
        bb.driveDirection = DD_FORWARD;
    }

    virtual NodeStatus onUpdate(Blackboard& bb) {
        if(getTimeInNode()> 10000) {
            errorHandler.setError("!03,TForward20 too long in state\r\n");
        }

        if (bb.motor.isPositionReached()) {
            return BH_SUCCESS;
        }

        return BH_RUNNING;
    }


};
*/



class TEscRotateCC: public Node    // Each task will be a class (derived from Node of course).
{
private:

    unsigned long lastTimeCalled;
    int arcEscRotate;
public:

    TEscRotateCC (): lastTimeCalled(0),arcEscRotate(0) {}

    virtual void onInitialize(Blackboard& bb) {
        unsigned long now = millis();
        bb.flagForceRotateDirection = FRD_CC;
        if( now-lastTimeCalled < 15000) {
            arcEscRotate += 50;
 
            sprintf(errorHandler.msg,"!03,CC arcEscRotate+=50; deltatime: %lu;\r\n",now-lastTimeCalled);
            errorHandler.setInfo();
        } else {
			arcEscRotate = myRandom(60, 135);
            errorHandler.setInfo("!03,CC arcEscRotate =  myRandom(60, 135);\r\n");
        }

		if (bb.flagForceSmallRotAngle > 0) {
			arcEscRotate = myRandom(50, 60);
			bb.flagForceSmallRotAngle--;
		}

		bb.cruiseSpeed = bb.CRUISE_SPEED_LOW;
		bb.driveDirection = DD_ROTATECC;
		bb.motor.turnTo(-1 * arcEscRotate, bb.cruiseSpeed);


        lastTimeCalled = millis();
    }

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if(getTimeInNode()> 10000) {
            errorHandler.setError("!03,EscRotateCC  too long in state\r\n");
        }


        if (bb.motor.isPositionReached()) {
            return BH_SUCCESS;
        }
        return BH_RUNNING;
    }
};


class TEscRotateCW: public Node    // Each task will be a class (derived from Node of course).
{
private:
    unsigned long lastTimeCalled;
    int arcEscRotate;
public:

    TEscRotateCW (): lastTimeCalled(0),arcEscRotate(0) {}

    virtual void onInitialize(Blackboard& bb) {
        unsigned long now = millis();
		bb.randAngle = myRandom(60, 135);
        bb.flagForceRotateDirection = FRD_CW;

		if( now-lastTimeCalled < 15000) {
            arcEscRotate += 50;
            sprintf(errorHandler.msg,"!03,CW arcEscRotate+=50; deltatime: %lu;\r\n",now-lastTimeCalled);
            errorHandler.setInfo();
        } else {
            arcEscRotate =  myRandom(60, 135);
            errorHandler.setInfo("!03,CW arcEscRotate=myRandom(60, 135)\r\n");
        }


		if (bb.flagForceSmallRotAngle > 0) {
			arcEscRotate = myRandom(50, 60);
			bb.flagForceSmallRotAngle--;
		}

		bb.cruiseSpeed = bb.CRUISE_SPEED_LOW;
		bb.driveDirection = DD_ROTATECW;
		bb.motor.turnTo(1 * arcEscRotate, bb.cruiseSpeed);

        lastTimeCalled = millis();
    }

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if(getTimeInNode()> 10000) {
            errorHandler.setError("!03,EscRotateCW  too long in state\r\n");
        }



        if (bb.motor.isPositionReached()) {
            return BH_SUCCESS;
        }
        return BH_RUNNING;
    }
};

#endif

