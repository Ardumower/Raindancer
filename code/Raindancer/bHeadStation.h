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

class TdriveBackXCS : public Action    // Each task will be a class (derived from Node of course).
{
private:

public:

	TdriveBackXCS() {}

	virtual void onInitialize(Blackboard& bb) {

		errorHandler.setInfo(F("!03,TdriveBackXCS called\r\n"));


		THistory hist = bb.getInitialisedHistoryEntry();
		hist.cruiseSpeed = bb.CRUISE_SPEED_LOW;
		hist.driveDirection = DD_REVERSE;
		hist.distanceSoll = -CONF_HEAD_CHARGING_DRIVE_BACK_CM;
		bb.addHistoryEntry(hist);
		srvMotor.enableDefaultRamping();
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {
		return BH_SUCCESS;

	}

	virtual void onTerminate(NodeStatus status, Blackboard& bb) {
		
	}
};


class TRrotate90CC : public Action {
private:

public:

	TRrotate90CC() {

	}

	virtual void onInitialize(Blackboard& bb) {

		errorHandler.setInfo(F("!03,TARrotate90CC called\r\n"));
		THistory hist = bb.getInitialisedHistoryEntry();
		hist.cruiseSpeed = bb.CRUISE_SPEED_LOW;
		hist.driveDirection = DD_ROTATECC;
		hist.distanceSoll = -90;
		bb.addHistoryEntry(hist);
		
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		return BH_SUCCESS;

	}
};

class TdriveForwardXCS : public Action    // Each task will be a class (derived from Node of course).
{
private:

public:

	TdriveForwardXCS() {}

	virtual void onInitialize(Blackboard& bb) {

		errorHandler.setInfo(F("!03,TdriveForwardXCS called\r\n"));
		THistory hist = bb.getInitialisedHistoryEntry();
		hist.cruiseSpeed = bb.CRUISE_SPEED_LOW;
		hist.driveDirection = DD_FORWARD;
		hist.distanceSoll = CONF_HEAD_CHARGING_DRIVE_FORW_CM;
		bb.addHistoryEntry(hist);

	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		return BH_SUCCESS;

	}

	virtual void onTerminate(NodeStatus status, Blackboard& bb) {

	}
};



class TSetArcHeadStation_ROT : public Action    // Each task will be a class (derived from Node of course).
{
private:
public:

	TSetArcHeadStation_ROT() {}

	virtual void onInitialize(Blackboard& bb) {


		errorHandler.setInfo(F("!03,TdriveForwardXCS called\r\n"));
		THistory hist = bb.getInitialisedHistoryEntry();
		hist.cruiseSpeed = bb.CRUISE_SPEED_LOW;


		// select random angle first
		int i = myRandom(0, 10000);
		if (i < 5000) {
			hist.driveDirection = DD_ROTATECC;
			hist.distanceSoll = -myRandom(0, 50);

		}
		else {
			hist.driveDirection = DD_ROTATECW;
			hist.distanceSoll = myRandom(0, 50);

		}

		if (bb.flagShowRotateX) {
			errorHandler.setInfo(F("!05,TSetArcHeadStation_ROT:0-50: %ld\r\n"), hist.distanceSoll);
		}

		bb.addHistoryEntry(hist);

	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		return BH_SUCCESS;
	}

};

class TsetMowBehaviour : public Action {
private:

public:

	TsetMowBehaviour() {
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {
		bb.setBehaviour(BH_MOW);
		return BH_SUCCESS;
	}
};


#endif

