// bHeadStation.h

#ifndef _BHEADSTATION_h
#define _BHEADSTATION_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "BehaviourTree.h"
#include "config.h"

class TdriveBackXCS : public Node    // Each task will be a class (derived from Node of course).
{
private:

public:

	TdriveBackXCS() {}

	virtual void onInitialize(Blackboard& bb) {

		errorHandler.setInfo(F("!03,TdriveBackXCS called\r\n"));
		bb.cruiseSpeed = bb.CRUISE_SPEED_LOW;
		bb.motor.rotateCM(-CONF_HEAD_CHARGING_DRIVE_BACK_CM, bb.cruiseSpeed);
		bb.driveDirection = DD_REVERSE_ESC_OBST;

		bb.addHistoryEntry(bb.driveDirection, 0.0f, 0.0f, 0.0f, FRD_NONE, CO_NONE);
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		bb.history[0].distanceDriven = bb.motor.getDistanceInCM();

		if (getTimeInNode() > 10000) {
			errorHandler.setError(F("!03,TdriveBackXCS too long in state\r\n"));
		}


		if (bb.motor.isPositionReached()) {
			errorHandler.setInfo(F("!03,TdriveForwardXCS position reached\r\n"));
			return BH_SUCCESS;
		}

		return BH_RUNNING;

	}

	virtual void onTerminate(NodeStatus status, Blackboard& bb) {
		
	}
};


class TdriveForwardXCS : public Node    // Each task will be a class (derived from Node of course).
{
private:

public:

	TdriveForwardXCS() {}

	virtual void onInitialize(Blackboard& bb) {

		errorHandler.setInfo(F("!03,TdriveForwardXCS called\r\n"));
		bb.cruiseSpeed = bb.CRUISE_SPEED_LOW;
		bb.motor.rotateCM(CONF_HEAD_CHARGING_DRIVE_FORW_CM, bb.cruiseSpeed);
		bb.driveDirection = DD_REVERSE_ESC_OBST;

		bb.addHistoryEntry(bb.driveDirection, 0.0f, 0.0f, 0.0f, FRD_NONE, CO_NONE);
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		bb.history[0].distanceDriven = bb.motor.getDistanceInCM();

		if (getTimeInNode() > 10000) {
			errorHandler.setError(F("!03,TdriveForwardXCS too long in state\r\n"));
		}


		if (bb.motor.isPositionReached()) {
			errorHandler.setInfo(F("!03,TdriveForwardXCS position reached\r\n"));
			return BH_SUCCESS;
		}

		return BH_RUNNING;

	}

	virtual void onTerminate(NodeStatus status, Blackboard& bb) {

	}
};



class TSetArcHeadStation_ROT : public Node    // Each task will be a class (derived from Node of course).
{
private:
public:

	TSetArcHeadStation_ROT() {}

	virtual void onInitialize(Blackboard& bb) {

		// select random angle first
		int i = myRandom(0, 10000);
		if (i < 5000) {
			bb.flagForceRotateDirection = FRD_CC;
			bb.driveDirection = DD_FEOROTATECC;
		}
		else {
			bb.flagForceRotateDirection = FRD_CW;
			bb.driveDirection = DD_FEOROTATECW;
		}

		bb.arcRotateXArc = myRandom(0, 50);
		if (bb.flagShowRotateX) {
			errorHandler.setInfo(F("!05,TSetArcHeadStation_ROT:0-50: %ld\r\n"), bb.arcRotateXArc);
		}
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		return BH_SUCCESS;
	}

};

#endif

