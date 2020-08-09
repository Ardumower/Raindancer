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
#ifndef BH_CRUISE_H
#define BH_CRUISE_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "BehaviourTree.h"
#include "Blackboard.h"
#include "config.h"
#include "UseServices.h"





class TCheckOscillating : public Action    // Each task will be a class (derived from Node of course).
{
private:

public:

	TCheckOscillating() {}


	virtual void onInitialize(Blackboard& bb) {
	}


	virtual NodeStatus onUpdate(Blackboard& bb) {


		if (
			(bb.history[0].driveDirection == DD_ROTATECC) &&
			(bb.history[1].driveDirection == DD_ROTATECW) &&
			(bb.history[2].driveDirection == DD_ROTATECC) &&
			(bb.history[3].driveDirection == DD_ROTATECW) &&
			(bb.history[4].driveDirection == DD_ROTATECC)) {

			errorHandler.setError(F("!03,TCheckOscillating oscillating detected\r\n"));
			return BH_SUCCESS;
		}

		return BH_FAILURE;
	}

	virtual void onTerminate(NodeStatus status, Blackboard& bb) {
	}
};


class TcruiseSpeedToMotor : public Action    // Each task will be a class (derived from Node of course).
{
private:
	unsigned long timeCruiseSpeedSet;

public:

	TcruiseSpeedToMotor() : timeCruiseSpeedSet(0) {}


	virtual void onInitialize(Blackboard& bb) {
		THistory hist = bb.getInitialisedHistoryEntry();
		hist.driveDirection = DD_FORWARD;
		hist.cruiseSpeed = bb.CRUISE_SPEED_HIGH;
		hist.distanceSoll = CONF_DRIVE_MAX_CM;
		bb.addHistoryEntry(hist);
		srvMotor.rotateCM(hist.distanceSoll, hist.cruiseSpeed);
	}


	virtual NodeStatus onUpdate(Blackboard& bb) {

		// Calculate speed

		if (srvRangeSensor.isNearObstacle()) {
			bb.history0.cruiseSpeed = bb.CRUISE_SPEED_OBSTACLE;
			timeCruiseSpeedSet = millis();
		}
		else if (srvPerSensoren.isNearPerimeter() || srvMowMotorSensor.motorUnderHeavyLoad) {
			bb.history0.cruiseSpeed = bb.CRUISE_SPEED_MEDIUM;
			timeCruiseSpeedSet = millis();
		}
		else if (millis() - timeCruiseSpeedSet > 1000) { // After 1 Seconds set accelerate cruise speed to high
			bb.history0.cruiseSpeed = bb.CRUISE_SPEED_HIGH;
		}

		srvMotor.changeSpeedPC(bb.history0.cruiseSpeed);


		if (srvMotor.isPositionReached()) {
			errorHandler.setError(F("!03,TCruiseSpeedToMotor Position reached\r\n"));
		}
		return BH_RUNNING;
	}

	virtual void onTerminate(NodeStatus status, Blackboard& bb) {
	}
};




class TcruiseStartMowMotor : public Action {
private:
	unsigned long startTime;
public:

	TcruiseStartMowMotor() {

	}

	virtual void onInitialize(Blackboard& bb) {

		if (CONF_DISABLE_MOW_MOTOR == true) {
			return;
		}

		if (!srvMotor.isMowMotRunning()) {
			srvMotor.mowMotStart();
			srvMotor.stopCLC(); // Drivemotors must be stopped. Normally they are, but here only for safety.
			startTime = millis();
		}
		else {
			startTime = millis() - 10000ul;
		}
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (CONF_DISABLE_MOW_MOTOR == true) {
			return BH_FAILURE;
		}
		// Wait 5 Sekonds until mow motor is running full speed
		if (millis() - startTime < 5000ul) {
			return BH_RUNNING;
		}

		// Set starttime in order that if( millis()- startTime < 5000ul) will not called again. Also not when millis overrun.
		// The mow mnotor is stopped in bb.setBehaviour()
		startTime = millis() - 10000ul;
		return BH_FAILURE;

	}
};



class TCheckChargStationDisabled : public Action    // Each task will be a class (derived from Node of course).
{
private:

public:

	TCheckChargStationDisabled() {}


	virtual NodeStatus onUpdate(Blackboard& bb) {


#if CONF_DISABLE_CHARGINGSTATION == true
		errorHandler.setError(F("BAT LOW\r\n"));
#endif


		return BH_SUCCESS;

	}
};

class TCruiseRotCW : public Action    // Each task will be a class (derived from Node of course).
{
private:
	int state;
	THistory hist;
public:

	TCruiseRotCW() {}

	virtual void onInitialize(Blackboard& bb) {

		hist = bb.getInitialisedHistoryEntry();
		hist.cruiseSpeed = bb.CRUISE_ROTATE_HIGH;

		if (bb.history0.coilFirstOutside == CO_RIGHT) {
			// Do nothing. Follow Line handles this
			state = 0;
			hist.driveDirection = DD_FORWARD;
		}
		else {
			srvMotor.turnTo(380, hist.cruiseSpeed);
			hist.driveDirection = DD_ROTATECW;
			state = 1;
		}

		bb.addHistoryEntry(hist);

	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		switch (state) {
		case 0:
			errorHandler.setInfo(F("!03,TCruiseRotCW calls: bb.setBehaviour(BH_PERITRACK);\r\n"));
			bb.setBehaviour(BH_PERITRACK);
			return BH_SUCCESS;
			break;
		case 1: //turn cw until both coils are inside
			if (srvPerSensoren.isRightInside() && srvPerSensoren.isLeftInside()) {
				state = 2;
			}
			break;
		case 2: // Wait until right coil is outside to start line following
			if (srvPerSensoren.isRightOutside()) {
				srvMotor.stopPC();
				/////				bb.cruiseSpeed = 0;
				state = 3;
			}
			break;

		case 3: // Wait until motor stopped
			if (srvMotor.isPositionReached()) {
				bb.setBehaviour(BH_PERITRACK);
				errorHandler.setInfo(F("!03,bb.setBehaviour(BH_PERITRACK);\r\n"));
				return BH_SUCCESS;
			}
			break;

		default:
			break;
		}

		if (srvMotor.isPositionReached()) {
			errorHandler.setError(F("!03,TCruiseRotCW line not found\r\n"));
			return BH_FAILURE;
		}

		return BH_RUNNING;
	}
};




class TacCruiseBatLow : public ActionCond {
private:

public:

	TacCruiseBatLow() {

	}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if (srvPerSensoren.isLeftInside() && srvPerSensoren.isRightInside() && (bb.history0.driveDirection == DD_FORWARD || bb.history0.driveDirection == DD_SPIRAL_CW)) {
			if (srvBatSensor.isVoltageLow() || bb.flagGoHome) {
				sprintf(errorHandler.msg, "!03,->%s\r\n", m_nodeName);
				if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
				else { errorHandler.writeToLogOnly(); }
				return BH_SUCCESS;
			}
		}
		return BH_FAILURE;
	}

	virtual void onInitialize(Blackboard& bb) {

	}

	virtual NodeStatus onUpdate(Blackboard& bb) {
		bb.flagGoHome = false;
		bb.setBehaviour(BH_FINDPERIMETER);
		errorHandler.setInfo(F("!03,TCruiseBatLow bat low detected\r\n"));
		return BH_SUCCESS;
	}
};


class TacCruiseRaining : public ActionCond {
private:

public:

	TacCruiseRaining() {
	}
	virtual NodeStatus onCheckCondition(Blackboard& bb) {

		if (srvPerSensoren.isLeftInside() && srvPerSensoren.isRightInside() && (bb.history0.driveDirection == DD_FORWARD || bb.history0.driveDirection == DD_SPIRAL_CW)) {
			if (srvRainSensor.isRaining()) {
				sprintf(errorHandler.msg, "!03,->%s\r\n", m_nodeName);
				if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
				else { errorHandler.writeToLogOnly(); }
				return BH_SUCCESS;
			}
		}
		return BH_FAILURE;
	}

	virtual void onInitialize(Blackboard& bb) {
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {
		bb.flagGoHome = false;
		bb.setBehaviour(BH_FINDPERIMETER);
		errorHandler.setInfo(F("!03,TCruiseRaining rain detected\r\n"));
		srvMotor.mowMotStop();
		return BH_SUCCESS;
	}
};



#endif

