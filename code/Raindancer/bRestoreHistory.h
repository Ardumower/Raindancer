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

	bool finished;
	enuDriveDirection dd;
public:

	TRestoreHistory() {}

	virtual void onInitialize(Blackboard& bb) {

		errorHandler.setInfo(F("TRestoreHistory CALLED numberToRestoreHist %d\r\n"), bb.numberToRestoreHist);
		finished = false;

		if (bb.numberToRestoreHist > HISTROY_BUFSIZE - 1) {
			errorHandler.setError(F("TRestoreHistory bb.numberToRestoreHist > HISTROY_BUFSIZE\r\n"));
		}


		if (bb.numberToRestoreHist <= 0 || bb.history[0].driveDirection == DD_NONE) {
			finished = true;
			return;
		}
		bb.numberToRestoreHist--;

		bb.cruiseSpeed = bb.CRUISE_SPEED_OBSTACLE;

		// save drive direction in dd for update() because history[0] will be deletet with bb.deleteLastHistoryEntry(); later.
		dd = bb.history[0].driveDirection;

		if (!finished) {
			switch (dd) {
			case DD_OVERRUN:
			case DD_FORWARD:
			case DD_REVERSE_INSIDE:
			case DD_FORWARD_INSIDE:
				bb.motor.rotateCM(-bb.history[0].distanceDriven, bb.cruiseSpeed);
				errorHandler.setInfo(F("!03,RestHist driveDirection: %s way: %f\r\n"), enuDriveDirectionString[dd], -bb.history[0].distanceDriven);
				break;
			case DD_LINE_FOLLOW:
				break;
			case DD_REVERSE_ESC_OBST:
				bb.motor.rotateCM(-bb.history[0].distanceDriven, bb.cruiseSpeed);
				errorHandler.setInfo(F("!03,RestHist driveDirection: %s way: %f\r\n"), enuDriveDirectionString[dd], -bb.history[0].distanceDriven);
				break;
			case DD_ROTATECW:
			case DD_ROTATECC:
			case DD_ROTATECC1:
			case DD_ROTATECW1:
			case DD_FEOROTATECW:
			case DD_FEOROTATECC:
				bb.motor.turnTo(-bb.history[0].rotAngleIst, bb.cruiseSpeed);
				errorHandler.setInfo(F("!03,RestHist driveDirection: %s angle: %f\r\n"), enuDriveDirectionString[dd], -bb.history[0].rotAngleIst);
				break;
			case DD_SPIRAL_CW:
				break;
			default:
				sprintf(errorHandler.msg, "!03,TRestoreHistory driveDirection not found: %s\r\n", enuDriveDirectionString[dd]);
				errorHandler.setError();
				break;
			}

			bb.deleteLastHistoryEntry();
		}

	}


	virtual NodeStatus onUpdate(Blackboard& bb) {

		// restore finished. Deactivate branch.
		if (finished) { 
			//bb.setBehaviour(BH_NONE);
			bb.flagEnableRestoreHistory = false;
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
		case DD_ROTATECW:
    	case DD_ROTATECC1:
		case DD_ROTATECW1:
		case DD_FEOROTATECW:
		case DD_FEOROTATECC:
		
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

