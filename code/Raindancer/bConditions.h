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
#include "UseServices.h"





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




class TConIsDirectionForward : public Condition    // Each task will be a class (derived from Condition of course).
{
public:

	TConIsDirectionForward() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if (bb.history0.driveDirection == DD_FORWARD || bb.history0.driveDirection == DD_SPIRAL_CW) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", m_nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};



class TconLCO_and_DDCC : public Condition    // Each task will be a class (derived from Condition of course).
{
public:

	TconLCO_and_DDCC() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if (srvPerSensoren.isLeftOutside() && bb.history0.driveDirection == DD_ROTATECC) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", m_nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};



class TconRCO_and_DDCW : public Condition    // Each task will be a class (derived from Condition of course).
{
public:

	TconRCO_and_DDCW() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if (srvPerSensoren.isRightOutside() && bb.history0.driveDirection == DD_ROTATECW) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", m_nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};


class TConIsDirectionRotateCW : public Condition    // Each task will be a class (derived from Condition of course).
{
public:

	TConIsDirectionRotateCW() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if (bb.history0.driveDirection == DD_ROTATECW) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", m_nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};


class TConIsDirectionRotateCC : public Condition    // Each task will be a class (derived from Condition of course).
{
public:

	TConIsDirectionRotateCC() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if (bb.history0.driveDirection == DD_ROTATECC) {
			sprintf(errorHandler.msg, "!03,->%s\r\n", m_nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};


class TConIsDirectionRotateCCorCW : public Condition    // Each task will be a class (derived from Condition of course).
{
public:

	TConIsDirectionRotateCCorCW() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if (bb.history0.driveDirection == DD_ROTATECC || bb.history0.driveDirection == DD_ROTATECW) {
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
	bool latched;
public:

	TConPerOutside() : latched(false) {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if (srvPerSensoren.isLeftOutside() || srvPerSensoren.isRightOutside()) {
			if (latched == false) {
				sprintf(errorHandler.msg, "!03,->%s\r\n", m_nodeName);
				if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
				else { errorHandler.writeToLogOnly(); }
				latched = true;
			}
			return BH_SUCCESS;
		}

		latched = false;
		return BH_FAILURE;
	}
};


class TConditionPerimeterNotFound : public Action    // Each task will be a class (derived from Node of course).
{
private:

public:

	TConditionPerimeterNotFound() {}



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


class TConBumperActive : public Condition    // Condition
{
private:

public:

	TConBumperActive() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if (bb.history0.driveDirection == DD_REVERSE) {
			return BH_FAILURE;
		}

		if (srvBumperSensor.isBumperActivated()) {
			
			sprintf(errorHandler.msg, "!03,->%s\r\n", m_nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}
};



class TConLeftCoilOutside : public Condition    // Each task will be a class (derived from Condition of course).
{
private:

public:

	TConLeftCoilOutside() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if (srvPerSensoren.isLeftOutside() && srvPerSensoren.isRightInside()) {
			sprintf(errorHandler.msg, "!03,->%s BH_SUCCESS\r\n", m_nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}
		/*
		sprintf(errorHandler.msg, "!03,->%s BH_FAILURE\r\n", m_nodeName);
		if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
		else { errorHandler.writeToLogOnly(); }
		*/
		return BH_FAILURE;
	}
};

class TConRightCoilOutside : public Condition    // Each task will be a class (derived from Condition of course).
{
private:

public:

	TConRightCoilOutside() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if (srvPerSensoren.isLeftInside() && srvPerSensoren.isRightOutside()) {
			sprintf(errorHandler.msg, "!03,->%s BH_SUCCESS\r\n", m_nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		/*
		sprintf(errorHandler.msg, "!03,->%s BH_FAILURE\r\n", m_nodeName);
		if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
		else { errorHandler.writeToLogOnly(); }
		*/
		return BH_FAILURE;
	}
};


class TConBothCoilsOutside : public Condition    // Each task will be a class (derived from Condition of course).
{
private:

public:

	TConBothCoilsOutside() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if (srvPerSensoren.isLeftOutside() && srvPerSensoren.isRightOutside()) {
			sprintf(errorHandler.msg, "!03,->%s BH_SUCCESS\r\n", m_nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		/*
		sprintf(errorHandler.msg, "!03,->%s BH_FAILURE\r\n", m_nodeName);
		if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
		else { errorHandler.writeToLogOnly(); }
		*/

		return BH_FAILURE;
	}
};

class TConOneCoilOutside : public Condition    // Each task will be a class (derived from Condition of course).
{
private:

public:

	TConOneCoilOutside() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if ((srvPerSensoren.isLeftInside() && srvPerSensoren.isRightOutside()) ||
			(srvPerSensoren.isLeftOutside() && srvPerSensoren.isRightInside())
			) {
			sprintf(errorHandler.msg, "!03,->%s BH_SUCCESS\r\n", m_nodeName);
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
			return BH_SUCCESS;
		}

		/*		sprintf(errorHandler.msg, "!03,->%s BH_FAILURE\r\n", m_nodeName);
		if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
		else { errorHandler.writeToLogOnly(); }
		*/

		return BH_FAILURE;
	}
};


class TConInDockingStation : public Condition    // Each task will be a class (derived from Condition of course).
{
private:

public:

	TConInDockingStation() {}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if (srvChargeSystem.isInChargingStation()) {
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

		if (srvPerSensoren.isLeftOutside() || srvPerSensoren.isRightOutside()) {
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

		if (srvMotor.getDistanceInMeterAreax() >= bb.areaTargetDistanceInMeter) {
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

