/*
Robotic Lawn Mower
Copyright (c) 2017 by Kai Würtz

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
#ifndef BH_CONDITIONS_H
#define BH_CONDITIONS_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "BehaviourTree.h"


class TConBatLow : public Node    // Each task will be a class (derived from Node of course).
{
private:

public:

	TConBatLow() {}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (bb.perimeterSensoren.isLeftInside() && bb.perimeterSensoren.isRightInside() && bb.driveDirection == DD_FORWARD) {
			if (bb.batterieSensor.isVoltageLow() || bb.flagGoHome) {
				sprintf(errorHandler.msg, "!03,->%s\r\n", nodeName);
				if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
				else { errorHandler.writeToLogOnly(); }
				return BH_SUCCESS;
			}
		}
		return BH_FAILURE;
	}
};


/*
class TConFEO_BACK180 : public Node    // Each task will be a class (derived from Node of course).
{
public:

	TConFEO_BACK180() {}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (bb.flagEscabeObstacleConFlag == FEO_BACK180) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};
*/

/*
class TConFEO_FWD20 : public Node    // Each task will be a class (derived from Node of course).
{
public:

	TConFEO_FWD20() {}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (bb.flagEscabeObstacleConFlag == FEO_FWD20) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};
*/

class TconSecondReverse : public Node    // Each task will be a class (derived from Node of course).
{
public:

	TconSecondReverse() {}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (bb.flagEnableSecondReverse  == true) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", nodeName);
			//errorHandler.setInfo();
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};

class TconDriveCurve : public Node    // Each task will be a class (derived from Node of course).
{
public:

	TconDriveCurve() {}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (bb.flagDriveCurve == true) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", nodeName);
			//errorHandler.setInfo();
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};


class TconRotateAtPer : public Node    // Each task will be a class (derived from Node of course).
{
public:

	TconRotateAtPer() {}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (bb.flagRotateAtPer == true) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", nodeName);
			//errorHandler.setInfo();
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};


class TConFEO_ROTCC1 : public Node    // Each task will be a class (derived from Node of course).
{
public:

	TConFEO_ROTCC1() {}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (bb.flagEscabeObstacleConFlag == FEO_ROTCC1) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", nodeName);
			//errorHandler.setInfo();
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};


class TConFEO_ROTCW1 : public Node    // Each task will be a class (derived from Node of course).
{
public:

	TConFEO_ROTCW1() {}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (bb.flagEscabeObstacleConFlag == FEO_ROTCW1) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", nodeName);
			//errorHandler.setInfo();
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};

class TConFEO_ROTCC2 : public Node    // Each task will be a class (derived from Node of course).
{
public:

	TConFEO_ROTCC2() {}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (bb.flagEscabeObstacleConFlag == FEO_ROTCC2) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", nodeName);
			//errorHandler.setInfo();
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};


class TConFEO_ROTCW2 : public Node    // Each task will be a class (derived from Node of course).
{
public:

	TConFEO_ROTCW2() {}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (bb.flagEscabeObstacleConFlag == FEO_ROTCW2) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", nodeName);
			//errorHandler.setInfo();
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};


class TConFEO_ROT : public Node    // Each task will be a class (derived from Node of course).
{
public:

	TConFEO_ROT() {}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (bb.flagEscabeObstacleConFlag == FEO_ROT) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", nodeName);
			//errorHandler.setInfo();
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};


class TConFEO_BACKINSIDE : public Node    // Each task will be a class (derived from Node of course).
{
public:

	TConFEO_BACKINSIDE() {}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (bb.flagEscabeObstacleConFlag == FEO_BACKINSIDE) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};

class TConWasDirectionForward : public Node    // Each task will be a class (derived from Node of course).
{
public:

	TConWasDirectionForward() {}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (bb.driveDirection == DD_FORWARD || bb.driveDirection == DD_SPIRAL_CW) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};
     

class TConWasDirectionFeoRotateCC : public Node    // Each task will be a class (derived from Node of course).
{
public:

	TConWasDirectionFeoRotateCC() {}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (bb.driveDirection == DD_FEOROTATECC ) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};


class TConWasDirectionFeoRotateCW : public Node    // Each task will be a class (derived from Node of course).
{
public:

	TConWasDirectionFeoRotateCW() {}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (bb.driveDirection == DD_FEOROTATECW ) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};
class TConWasDirectionFeoRotateCC1 : public Node    // Each task will be a class (derived from Node of course).
{
public:

	TConWasDirectionFeoRotateCC1() {}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (bb.driveDirection == DD_FEOROTATECC1 || bb.driveDirection == DD_FEOROTATECC2) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};


class TConWasDirectionFeoRotateCW1 : public Node    // Each task will be a class (derived from Node of course).
{
public:

	TConWasDirectionFeoRotateCW1() {}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (bb.driveDirection == DD_FEOROTATECW1 || bb.driveDirection == DD_FEOROTATECW2) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};

/*
class TConWasDirectionForward20 : public Node    // Each task will be a class (derived from Node of course).
{
public:

	TConWasDirectionForward20() {}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (bb.driveDirection == DD_FORWARD20) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};
*/

class TConWasDirectionForwardInside : public Node    // Each task will be a class (derived from Node of course).
{
public:

	TConWasDirectionForwardInside() {}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (bb.driveDirection == DD_FORWARD_INSIDE) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};


class TConWasDirectionReverseObstacle : public Node    // Each task will be a class (derived from Node of course).
{
public:

	TConWasDirectionReverseObstacle() {}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (bb.driveDirection == DD_REVERSE_ESC_OBST) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};


class TConWasDirectionOverrun : public Node    // Each task will be a class (derived from Node of course).
{
public:

	TConWasDirectionOverrun() {}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (bb.driveDirection == DD_OVERRUN) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};




class TConWasDirectionReverseInside : public Node    // Each task will be a class (derived from Node of course).
{
public:

	TConWasDirectionReverseInside() {}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (bb.driveDirection == DD_REVERSE_INSIDE) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};



class TConWasDirectionRotateCW : public Node    // Each task will be a class (derived from Node of course).
{
public:

	TConWasDirectionRotateCW() {}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (bb.driveDirection == DD_ROTATECW || bb.driveDirection == DD_ROTATECW1) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};


class TConWasDirectionRotateCC : public Node    // Each task will be a class (derived from Node of course).
{
public:

	TConWasDirectionRotateCC() {}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (bb.driveDirection == DD_ROTATECC || bb.driveDirection == DD_ROTATECC1) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};



class TConPerOutside : public Node    // Each task will be a class (derived from Node of course).
{
private:

public:

	TConPerOutside() {}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (bb.flagCoilFirstOutsideLatched != CO_NONE) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}
		/*
		if (bb.perimeterSensoren.isLeftOutside() || bb.perimeterSensoren.isRightOutside()) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}
		*/
		return BH_FAILURE;
	}
};


class TConBumperActive : public Node    // Condition
{
private:

public:

	TConBumperActive() {}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (bb.driveDirection == DD_REVERSE_ESC_OBST) {
			return BH_FAILURE;
		}

		if (bb.bumperSensor.isBumperActivated()) {   //for bumper.  hard stop is activated by bumper sensor. Therfore I have to check this first before doing further steps
	
       /* NEVER set any variables or do any action in a condition
       //bber--------------------------------------------------
      if (bb.bumperSensor.isBumperActivatedLeft()) {
          bb.flagBumperActivatedLeft=true;
        }
      if (bb.bumperSensor.isBumperActivatedRight()) {
          bb.flagBumperActivatedRight=true;
        }
		//-----------------------------------------------------
      */ 

			sprintf(errorHandler.msg, "!03,->%s\r\n", nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};





class TConLeftCoilOutside : public Node    // Each task will be a class (derived from Node of course).
{
private:

public:

	TConLeftCoilOutside() {}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (bb.perimeterSensoren.isLeftOutside() && bb.perimeterSensoren.isRightInside()) {
			sprintf(errorHandler.msg, "!03,->%s BH_SUCCESS\r\n", nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}
		sprintf(errorHandler.msg, "!03,->%s BH_FAILURE\r\n", nodeName);
		if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
		else { errorHandler.writeToLogOnly(); }
		return BH_FAILURE;
	}
};

class TConRightCoilOutside : public Node    // Each task will be a class (derived from Node of course).
{
private:

public:

	TConRightCoilOutside() {}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (bb.perimeterSensoren.isLeftInside() && bb.perimeterSensoren.isRightOutside()) {
			sprintf(errorHandler.msg, "!03,->%s BH_SUCCESS\r\n", nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}
		sprintf(errorHandler.msg, "!03,->%s BH_FAILURE\r\n", nodeName);
		if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
		else { errorHandler.writeToLogOnly(); }
		return BH_FAILURE;
	}
};


class TConBothCoilsOutside : public Node    // Each task will be a class (derived from Node of course).
{
private:

public:

	TConBothCoilsOutside() {}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (bb.perimeterSensoren.isLeftOutside() && bb.perimeterSensoren.isRightOutside()) {
			sprintf(errorHandler.msg, "!03,->%s BH_SUCCESS\r\n", nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}
		sprintf(errorHandler.msg, "!03,->%s BH_FAILURE\r\n", nodeName);
		if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
		else { errorHandler.writeToLogOnly(); }
		return BH_FAILURE;
	}
};

class TConOneCoilOutside : public Node    // Each task will be a class (derived from Node of course).
{
private:

public:

	TConOneCoilOutside() {}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if ((bb.perimeterSensoren.isLeftInside() && bb.perimeterSensoren.isRightOutside()) ||
			(bb.perimeterSensoren.isLeftOutside() && bb.perimeterSensoren.isRightInside())
			) {
			sprintf(errorHandler.msg, "!03,->%s BH_SUCCESS\r\n", nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}
		sprintf(errorHandler.msg, "!03,->%s BH_FAILURE\r\n", nodeName);
		if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
		else { errorHandler.writeToLogOnly(); }
		return BH_FAILURE;
	}
};


class TConInDockingStation : public Node    // Each task will be a class (derived from Node of course).
{
private:

public:

	TConInDockingStation() {}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (bb.chargeSystem.isInChargingStation()) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}
		//errorHandler.setInfo("!03,TchargeSystem NOT detected\r\n");
		return BH_FAILURE;
	}
};


class TConIsOutsidePerimeter : public Node    // Each task will be a class (derived from Node of course).
{
private:

public:

	TConIsOutsidePerimeter() {}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (bb.perimeterSensoren.isLeftOutside() || bb.perimeterSensoren.isRightOutside()) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}
		//errorHandler.setInfo("!03,TConIsOutsidePerimeter: Both coils inside\r\n");
		return BH_FAILURE;
	}
};


class TconAreaReached : public Node    // Each task will be a class (derived from Node of course).
{
private:

public:

	TconAreaReached() {}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (bb.motor.getDistanceInMeterAreax() >= bb.areaTargetDistanceInMeter) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}
		//errorHandler.setInfo("!03,TchargeSystem NOT detected\r\n");
		return BH_FAILURE;
	}
};

#endif

