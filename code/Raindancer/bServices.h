#pragma once

#ifndef _BSERVICES_h
#define _BSERVICES_h

#include "UseServices.h"

class TsrvUpdateHistory : public CompNodeService {

	void onRun(Blackboard& bb) override {

		if (bb.history0.driveDirection == DD_NONE) {
			return;
		}

		bb.historyUpdateDistance();
/*
		if (bb.history0.coilFirstOutside == CO_NONE) { //Latch first coil outside until both coils are inside
			if (srvPerSensoren.isLeftOutside() && srvPerSensoren.isRightOutside()) {
				bb.history0.coilFirstOutside = CO_BOTH;
				sprintf(errorHandler.msg, "!03,->TsrvSetflagCoilFirstOutsideLatched = CO_BOTH\r\n");
				if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
				else { errorHandler.writeToLogOnly(); }
			}
			else if (srvPerSensoren.isLeftOutside()) {
				bb.history0.coilFirstOutside = CO_LEFT;
				sprintf(errorHandler.msg, "!03,->TsrvSetflagCoilFirstOutsideLatched = CO_LEFT\r\n");
				if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
				else { errorHandler.writeToLogOnly(); }
			}
			else if (srvPerSensoren.isRightOutside()) {
				bb.history0.coilFirstOutside = CO_RIGHT;
				sprintf(errorHandler.msg, "!03,->TsrvSetflagCoilFirstOutsideLatched = CO_RIGHT\r\n");
				if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
				else { errorHandler.writeToLogOnly(); }
			}
		}

		if (bb.history0.bumperActivated == BUM_NONE) {
			if (srvBumperSensor.isBumperActivatedLeft() && srvBumperSensor.isBumperActivatedRight()) {
				bb.history0.bumperActivated = BUM_BOTH;
				if (srvBumperSensor.flagShowBumper) {
					errorHandler.setInfo(F("!03,TsrvSetFlagBumperActivated bb.history0.bumperActivated = BUM_BOTH;\r\n"));
				}
			}
			else if (srvBumperSensor.isBumperActivatedLeft()) {
				bb.history0.bumperActivated = BUM_LEFT;
				if (srvBumperSensor.flagShowBumper) {
					errorHandler.setInfo(F("!03,TsrvSetFlagBumperActivated bb.history0.bumperActivated = BUM_LEFT;\r\n"));
				}
			}
			else if (srvBumperSensor.isBumperActivatedRight()) {
				bb.history0.bumperActivated = BUM_RIGHT;
				if (srvBumperSensor.flagShowBumper) {
					errorHandler.setInfo(F("!03,TsrvSetFlagBumperActivated bb.history0.bumperActivated = BUM_RIGHT;\r\n"));
				}
			}
			else if (srvBumperSensor.isBumperDuinoActivated()) {
				bb.history0.bumperActivated = BUM_DUINO;
				if (srvBumperSensor.flagShowBumper) {
					errorHandler.setInfo(F("!03,TsrvSetFlagBumperActivated bb.history0.bumperActivated = BUM_DUINO;\r\n"));
				}
			}
		}
		*/
	}

	
};



#endif