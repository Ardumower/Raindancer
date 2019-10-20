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
#ifndef BH_CONDITIONS_H
#define BH_CONDITIONS_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "BehaviourTree.h"



/*
class TConFEO_BACK180 : public Condition    // Each task will be a class (derived from Condition of course).
{
public:

	TConFEO_BACK180() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if (bb.flagEscabeObstacleConFlag == FEO_BACK180) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", m_nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};
*/

/*
class TConFEO_FWD20 : public Condition    // Each task will be a class (derived from Condition of course).
{
public:

	TConFEO_FWD20() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if (bb.flagEscabeObstacleConFlag == FEO_FWD20) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", m_nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};
*/

class TconSecondReverse : public Condition    // Each task will be a class (derived from Condition of course).
{
public:

	TconSecondReverse() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if (bb.flagEnableSecondReverse  == true) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", m_nodeName);
			//errorHandler.setInfo();
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};

class TconDriveCurve : public Condition    // Each task will be a class (derived from Condition of course).
{
public:

	TconDriveCurve() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if (bb.flagDriveCurve == true) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", m_nodeName);
			//errorHandler.setInfo();
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};


class TconRotateAtPer : public Condition    // Each task will be a class (derived from Condition of course).
{
public:

	TconRotateAtPer() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if (bb.flagRotateAtPer == true) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", m_nodeName);
			//errorHandler.setInfo();
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};


class TConFEO_ROTCC1 : public Condition    // Each task will be a class (derived from Condition of course).
{
public:

	TConFEO_ROTCC1() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if (bb.flagEscabeObstacleConFlag == FEO_ROTCC1) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", m_nodeName);
			//errorHandler.setInfo();
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};


class TConFEO_ROTCW1 : public Condition    // Each task will be a class (derived from Condition of course).
{
public:

	TConFEO_ROTCW1() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if (bb.flagEscabeObstacleConFlag == FEO_ROTCW1) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", m_nodeName);
			//errorHandler.setInfo();
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};

class TConFEO_ROTCC2 : public Condition    // Each task will be a class (derived from Condition of course).
{
public:

	TConFEO_ROTCC2() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if (bb.flagEscabeObstacleConFlag == FEO_ROTCC2) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", m_nodeName);
			//errorHandler.setInfo();
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};


class TConFEO_ROTCW2 : public Condition    // Each task will be a class (derived from Condition of course).
{
public:

	TConFEO_ROTCW2() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if (bb.flagEscabeObstacleConFlag == FEO_ROTCW2) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", m_nodeName);
			//errorHandler.setInfo();
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};


class TConFEO_ROT : public Condition    // Each task will be a class (derived from Condition of course).
{
public:

	TConFEO_ROT() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {
		errorHandler.setInfoNoLog(F(" TConFEO_ROT called: %s\r\n"), m_nodeName);

		if (bb.flagEscabeObstacleConFlag == FEO_ROT) {
			errorHandler.setInfoNoLog(F(" TConFEO_ROT flag found: %s\r\n"), m_nodeName);
			sprintf(errorHandler.msg, "!03,->%s\r\n", m_nodeName);
			//errorHandler.setInfo();
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};


class TConFEO_BACKINSIDE : public Condition    // Each task will be a class (derived from Condition of course).
{
public:

	TConFEO_BACKINSIDE() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if (bb.flagEscabeObstacleConFlag == FEO_BACKINSIDE) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", m_nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};

class TConWasDirectionForward : public Condition    // Each task will be a class (derived from Condition of course).
{
public:

	TConWasDirectionForward() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if (bb.driveDirection == DD_FORWARD || bb.driveDirection == DD_SPIRAL_CW) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", m_nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};
     

class TConWasDirectionFeoRotateCC : public Condition    // Each task will be a class (derived from Condition of course).
{
public:

	TConWasDirectionFeoRotateCC() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if (bb.driveDirection == DD_FEOROTATECC ) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", m_nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};


class TConWasDirectionFeoRotateCW : public Condition    // Each task will be a class (derived from Condition of course).
{
public:

	TConWasDirectionFeoRotateCW() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if (bb.driveDirection == DD_FEOROTATECW ) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", m_nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};
class TConWasDirectionFeoRotateCC1 : public Condition    // Each task will be a class (derived from Condition of course).
{
public:

	TConWasDirectionFeoRotateCC1() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if (bb.driveDirection == DD_FEOROTATECC1 || bb.driveDirection == DD_FEOROTATECC2) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", m_nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};


class TConWasDirectionFeoRotateCW1 : public Condition    // Each task will be a class (derived from Condition of course).
{
public:

	TConWasDirectionFeoRotateCW1() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if (bb.driveDirection == DD_FEOROTATECW1 || bb.driveDirection == DD_FEOROTATECW2) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", m_nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};

/*
class TConWasDirectionForward20 : public Condition    // Each task will be a class (derived from Condition of course).
{
public:

	TConWasDirectionForward20() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if (bb.driveDirection == DD_FORWARD20) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", m_nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};
*/

class TConWasDirectionForwardInside : public Condition    // Each task will be a class (derived from Condition of course).
{
public:

	TConWasDirectionForwardInside() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if (bb.driveDirection == DD_FORWARD_INSIDE) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", m_nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};


class TConWasDirectionReverseObstacle : public Condition    // Each task will be a class (derived from Condition of course).
{
public:

	TConWasDirectionReverseObstacle() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if (bb.driveDirection == DD_REVERSE_ESC_OBST) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", m_nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};


class TConWasDirectionOverrun : public Condition    // Each task will be a class (derived from Condition of course).
{
public:

	TConWasDirectionOverrun() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if (bb.driveDirection == DD_OVERRUN) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", m_nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};




class TConWasDirectionReverseInside : public Condition    // Each task will be a class (derived from Condition of course).
{
public:

	TConWasDirectionReverseInside() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if (bb.driveDirection == DD_REVERSE_INSIDE) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", m_nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};



class TConWasDirectionRotateCW : public Condition    // Each task will be a class (derived from Condition of course).
{
public:

	TConWasDirectionRotateCW() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if (bb.driveDirection == DD_ROTATECW || bb.driveDirection == DD_ROTATECW1) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", m_nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};


class TConWasDirectionRotateCC : public Condition    // Each task will be a class (derived from Condition of course).
{
public:

	TConWasDirectionRotateCC() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if (bb.driveDirection == DD_ROTATECC || bb.driveDirection == DD_ROTATECC1) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", m_nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};



class TConPerOutside : public Condition    // Each task will be a class (derived from Condition of course).
{
private:

public:

	TConPerOutside() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if (bb.flagCoilFirstOutsideLatched != CO_NONE) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", m_nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}
		/*
		if (bb.perimeterSensoren.isLeftOutside() || bb.perimeterSensoren.isRightOutside()) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", m_nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}
		*/
		return BH_FAILURE;
	}
};


class TConBumperActive : public Condition    // Condition
{
private:

public:

	TConBumperActive() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if (bb.driveDirection == DD_REVERSE_ESC_OBST) {
			return BH_FAILURE;
		}

		if (bb.bumperSensor.isBumperActivated()) {  
	
			sprintf(errorHandler.msg, "!03,->%s\r\n", m_nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};


class TConditionFEONotFound : public Condition    // Each task will be a class (derived from Node of course).
{
private:

public:

	TConditionFEONotFound() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {
		errorHandler.setError(F("!03,TConditionFEONotFound not found %s\r\n"), enuFlagEscabeObstacleConFlagString[bb.flagEscabeObstacleConFlag]);
		return BH_SUCCESS;
	}


};


class TConLeftCoilOutside : public Condition    // Each task will be a class (derived from Condition of course).
{
private:

public:

	TConLeftCoilOutside() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if (bb.perimeterSensoren.isLeftOutside() && bb.perimeterSensoren.isRightInside()) {
			sprintf(errorHandler.msg, "!03,->%s BH_SUCCESS\r\n", m_nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}
		sprintf(errorHandler.msg, "!03,->%s BH_FAILURE\r\n", m_nodeName);
		if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
		else { errorHandler.writeToLogOnly(); }
		return BH_FAILURE;
	}
};

class TConRightCoilOutside : public Condition    // Each task will be a class (derived from Condition of course).
{
private:

public:

	TConRightCoilOutside() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if (bb.perimeterSensoren.isLeftInside() && bb.perimeterSensoren.isRightOutside()) {
			sprintf(errorHandler.msg, "!03,->%s BH_SUCCESS\r\n", m_nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}
		sprintf(errorHandler.msg, "!03,->%s BH_FAILURE\r\n", m_nodeName);
		if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
		else { errorHandler.writeToLogOnly(); }
		return BH_FAILURE;
	}
};


class TConBothCoilsOutside : public Condition    // Each task will be a class (derived from Condition of course).
{
private:

public:

	TConBothCoilsOutside() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if (bb.perimeterSensoren.isLeftOutside() && bb.perimeterSensoren.isRightOutside()) {
			sprintf(errorHandler.msg, "!03,->%s BH_SUCCESS\r\n", m_nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}
		sprintf(errorHandler.msg, "!03,->%s BH_FAILURE\r\n", m_nodeName);
		if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
		else { errorHandler.writeToLogOnly(); }
		return BH_FAILURE;
	}
};

class TConOneCoilOutside : public Condition    // Each task will be a class (derived from Condition of course).
{
private:

public:

	TConOneCoilOutside() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if ((bb.perimeterSensoren.isLeftInside() && bb.perimeterSensoren.isRightOutside()) ||
			(bb.perimeterSensoren.isLeftOutside() && bb.perimeterSensoren.isRightInside())
			) {
			sprintf(errorHandler.msg, "!03,->%s BH_SUCCESS\r\n", m_nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}
		sprintf(errorHandler.msg, "!03,->%s BH_FAILURE\r\n", m_nodeName);
		if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
		else { errorHandler.writeToLogOnly(); }
		return BH_FAILURE;
	}
};


class TConInDockingStation : public Condition    // Each task will be a class (derived from Condition of course).
{
private:

public:

	TConInDockingStation() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if (bb.chargeSystem.isInChargingStation()) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", m_nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}
		//errorHandler.setInfo("!03,TchargeSystem NOT detected\r\n");
		return BH_FAILURE;
	}
};


class TConIsOutsidePerimeter : public Condition    // Each task will be a class (derived from Condition of course).
{
private:

public:

	TConIsOutsidePerimeter() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if (bb.perimeterSensoren.isLeftOutside() || bb.perimeterSensoren.isRightOutside()) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", m_nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}
		//errorHandler.setInfo("!03,TConIsOutsidePerimeter: Both coils inside\r\n");
		return BH_FAILURE;
	}
};


class TconAreaReached : public Condition    // Each task will be a class (derived from Condition of course).
{
private:

public:

	TconAreaReached() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if (bb.motor.getDistanceInMeterAreax() >= bb.areaTargetDistanceInMeter) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", m_nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}
		//errorHandler.setInfo("!03,TchargeSystem NOT detected\r\n");
		return BH_FAILURE;
	}
};

#endif

