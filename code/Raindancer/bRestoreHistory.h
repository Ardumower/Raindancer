// bRestoreHistory.h

#ifndef _BRESTOREHISTORY_h
#define _BRESTOREHISTORY_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "BehaviourTree.h"
#include "config.h"

class TRestoreHistory : public Node
{
private:
	uint8_t state;
	uint8_t idx;
	bool finished;
	enuDriveDirection dd;
public:

	TRestoreHistory() {}

	virtual void onInitialize(Blackboard& bb) {


		finished = false;

		if (bb.numberToRestoreHist > HISTROY_BUFSIZE) {
			errorHandler.setError(F("TRestoreHistory bb.numberToRestoreHist > HISTROY_BUFSIZE"));
		}

		for (idx = 0; idx < HISTROY_BUFSIZE; idx++ ) {
			if (bb.history[idx].restored == false) {
				break; //idx is the index to the struct to restore
			}
		}
		
		// First inserted Item on Position HISTROY_BUFSIZE-1 in the array will not be restored
		if (idx >= (bb.numberToRestoreHist-1) || bb.history[idx].driveDirection == DD_NONE) {
			finished = true;
			return;
		}

		bb.history[idx].restored = true;

		bb.cruiseSpeed = bb.CRUISE_SPEED_OBSTACLE;

		// save drive direction in dd tor update() because history[0] will be deletet with bb.deleteLastHistoryEntry(); later.
		dd = bb.history[idx].driveDirection;

		switch (dd) {
		case DD_OVERRUN:
		case DD_FORWARD:
		case DD_REVERSE_INSIDE:
		case DD_FORWARD_INSIDE:
			bb.motor.rotateCM(-bb.history[idx].distanceDriven, bb.cruiseSpeed);
			errorHandler.setInfo(F("!03,RestHist driveDirection: %s way: %f\r\n"), enuDriveDirectionString[dd], -bb.history[idx].distanceDriven);
			break;
		case DD_LINE_FOLLOW:
			break;
		case DD_REVERSE_ESC_OBST:
			bb.motor.rotateCM(-bb.history[idx].distanceDriven, bb.cruiseSpeed);
			errorHandler.setInfo(F("!03,RestHist driveDirection: %s way: %f\r\n"), enuDriveDirectionString[dd], -bb.history[idx].distanceDriven);
			break;
		case DD_ROTATECW:
		case DD_ROTATECC:
		case DD_FEOROTATECW:
		case DD_FEOROTATECC:
			bb.motor.turnTo(-bb.history[idx].rotAngleIst, bb.cruiseSpeed);
			errorHandler.setInfo(F("!03,RestHist driveDirection: %s angle: %f\r\n"), enuDriveDirectionString[dd], -bb.history[idx].rotAngleIst);
			break;
		case DD_SPIRAL_CW:
			break;
		default:
			sprintf(errorHandler.msg, "!03,TRestoreHistory driveDirection not found: %s", enuDriveDirectionString[dd]);
			errorHandler.setError();
			break;
		}


	}


	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (finished) { // Warten bis motor gestoppt
			bb.setBehaviour(BH_NONE);
			return BH_SUCCESS;
		}

		switch (dd) {
		case DD_OVERRUN:
		case DD_FORWARD:
		case DD_REVERSE_INSIDE:
		case DD_FORWARD_INSIDE:
			if (bb.motor.isPositionReached()) {
				return BH_SUCCESS;
			}
			break;
		case DD_LINE_FOLLOW:
			return BH_SUCCESS;
			break;
		case DD_REVERSE_ESC_OBST:
			if (bb.motor.isPositionReached()) {
				return BH_SUCCESS;
			}
			break;
		case DD_ROTATECC:
		case DD_FEOROTATECW:
		case DD_FEOROTATECC:
		case DD_ROTATECW:
			if (bb.motor.isPositionReached()) {
				return BH_SUCCESS;
			}
			break;
		case DD_SPIRAL_CW:
			return BH_SUCCESS;
			break;
		default:
			sprintf(errorHandler.msg, "!03,TRestoreHistory driveDirection not found: %s", enuDriveDirectionString[dd]);
			errorHandler.setError();
			break;
		}
		return BH_RUNNING;

	}

};





#endif

