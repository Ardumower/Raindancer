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

#include "Blackboard.h"
#include "UseServices.h"
#include "config.h"


const char* enuDriveDirectionString[] = { "DD_FORWARD",
							"DD_REVERSE",
							"DD_ROTATECW",
							"DD_ROTATECC",
							"DD_SPIRAL_CW",
							"DD_NONE",
							"UNKNOWN1",
							"UNKNOWN2",
							"UNKNOWN3",
							"UNKNOWN4",
							"UNKNOWN5",
							"UNKNOWN6"
};



const char* enuFlagForceRotateDirectionString[] = { "FRD_NONE",
	"FRD_ROTCC",
	"FRD_ROTCW",
	"UNKNOWN1",
	"UNKNOWN2",
	"UNKNOWN3",
	"UNKNOWN4",
	"UNKNOWN5",
	"UNKNOWN6"
};


const char* enuFlagCoilsOutsideString[] = { "CO_NONE",
	"CO_RIGHT",
	"CO_LEFT",
	"CO_BOTH",
	"UNKNOWN1",
	"UNKNOWN2",
	"UNKNOWN3",
	"UNKNOWN4",
	"UNKNOWN5",
	"UNKNOWN6"
};


const char* enuFlagBumperActivatedString[] = { "BUM_NONE",
	"BUM_RIGHT",  //Not used jet
	"BUM_LEFT",   //Not used jet
	"BUM_BOTH",
	"BUM_DUINO",
	"UNKNOWN1",
	"UNKNOWN2",
	"UNKNOWN3",
	"UNKNOWN4",
	"UNKNOWN5",
	"UNKNOWN6"
};




void Blackboard::setBehaviour(enuBehaviour b) {

	// save statistic

#if CONF_DISABLE_EEPROM_SERVICE == false
	if (flagEnableMowing == true && b!= BH_MOW) { //only save if in mowing mode
		errorHandler.setInfo(F("!04,Write Stats to EEPROM\r\n"));

		float mowtime = srvEeprom.readFloat(EEPADR_MOWTIME);
		unsigned long time = millis() - timeInMowBehaviour;
		double hours = (double)time / 3600000.0;
		mowtime += hours;
		srvEeprom.writeFloat(EEPADR_MOWTIME, mowtime);

		float mowway = srvEeprom.readFloat(EEPADR_MOWDIRVENWAY);
		float ticks = (srvMotor.L->myEncoder->getAbsTicksCounter() + srvMotor.R->myEncoder->getAbsTicksCounter()) / 2.0f;
		mowway += srvMotor.getMForCounts(ticks);
		srvEeprom.writeFloat(EEPADR_MOWDIRVENWAY, mowway);

		int32_t rotations = srvEeprom.read32t(EEPADR_ROTATIONCOUNT);
		rotations = rotations + numberOfRotations;
		srvEeprom.write32t(EEPADR_ROTATIONCOUNT, rotations);
	}
#endif

	flagEnableMowing = false;
	flagEnablePerimetertracking = false;
	flagEnableCharging = false;
	flagEnableGotoAreaX = false;
	flagEnableFindPerimeter = false;
	flagEnableLeaveHeadChargingStation = false;

	srvMotor.enableDefaultRamping();

	switch (b) {
	case BH_GOTOAREA:
		resetBB();  //Reset BB because here starts a new mow sequence
		flagEnableGotoAreaX = true;
		flagGotoAreaXFirstCall = true;
		//srvRangeSensor.enabled = false;
		srvChargeSystem.deactivateRelay();  //Muss hier auch abgeschaltet werden, falls user dieses ueber das UI einschaltet.
		//srvMotor.mowMotStop(); // Will be started in BH_MOW behaviour tree.
		srvMotor.stopAllMotors(); // Nur zur Sicherheit, falls diese gerade laufen.
		srvMotor.startDistanceMeasurementAreax();
		errorHandler.setInfo(F("!04,SET BEHAV -> BH_GOTOAREA\r\n"));
		break;
	case BH_CHARGING:
		flagEnableCharging = true;
		//srvRangeSensor.enabled = false;
		srvMotor.mowMotStop();
		errorHandler.setInfo(F("!04,SET BEHAV -> BH_CHARGING\r\n"));
		break;
	case BH_PERITRACK:
		flagEnablePerimetertracking = true;
		//srvRangeSensor.enabled = false;
		srvChargeSystem.deactivateRelay();
		//srvMotor.mowMotStop();
		errorHandler.setInfo(F("!04,SET BEHAV -> BH_PERITRACK\r\n"));
		break;
	case BH_FINDPERIMETER:
		flagEnableFindPerimeter = true;
		//srvRangeSensor.enabled = true;
		srvChargeSystem.deactivateRelay();
		//srvMotor.mowMotStop();
		errorHandler.setInfo(F("!04,SET BEHAV -> BH_FINDPERIMETER\r\n"));
		break;
	case BH_MOW:
		resetBB(); //Reset BB because here starts a new mow sequence
		myRandom(0, 5000, true); // init seed value
		flagEnableMowing = true;
		//srvRangeSensor.enabled = true;
		srvChargeSystem.deactivateRelay();
		// reset statistic counter
		srvMotor.resetEncoderCounter();
		numberOfRotations = 0;
		timeInMowBehaviour = millis();
		//srvMotor.startDistanceMeasurement();
		errorHandler.setInfo(F("!04,SET BEHAV -> BH_MOW\r\n"));
		break;
	case BH_LEAVE_HEAD_CS:
		resetBB();  //Reset BB because here starts a new mow sequence
		flagEnableLeaveHeadChargingStation = true;
		//srvRangeSensor.enabled = false;
		srvChargeSystem.deactivateRelay();
		srvMotor.stopAllMotors(); // Nur zur Sicherheit, falls diese gerade laufen.
		//srvMotor.startDistanceMeasurementAreax();
		errorHandler.setInfo(F("!04,SET BEHAV -> BH_LEAVE_HEAD_CS\r\n"));
		break;
	case BH_NONE:
		// Don't reset BB here because if you change in manual mode, you will probaly check the BB variables like history or so.
		//srvRangeSensor.enabled = false;
		srvChargeSystem.deactivateRelay();
		srvMotor.mowMotStop();
		errorHandler.setInfo(F("!04,SET BEHAV -> BH_NONE\r\n"));
		break;
	default:
		errorHandler.setError(F("!04,setBehaviour unbekanntes Behaviour\r\n"));
	}
}

void Blackboard::addHistoryEntry(THistory& hist) {
	//Shift History for new entry
	for (int i = HISTROY_BUFSIZE - 1; i > 0; i--) {
		history[i] = history[i - 1];
	}

	history[0] = hist;
	history0.timeAdded = millis();

	errorHandler.setInfo(F("!03,add histEntry driveDirection: %s \r\n"), enuDriveDirectionString[history0.driveDirection]);

	srvMotor.startDistanceMeasurement();

	if (flagShowHistory == true) {
		printHistoryEntry(0);
	}

}

THistory Blackboard::getInitialisedHistoryEntry(){
	THistory hist;
	hist.driveDirection = DD_NONE;		// [0]  Enthaelt Fahrtrichtung
	hist.distanceSoll = 0;
	hist.distanceIst = 0;				// [0] Enthaelt die gerade gefahrene distanz/winkeldistanz
	hist.coilFirstOutside = CO_NONE;   // [0] which coil was first outside jet
	hist.coilOutsideAfterOverrun = CO_NONE;
	hist.restored = false;					// wurde die node restored
	hist.timeAdded = 0;			// zeit, wann node erstellt wurde in ms
	hist.bumperActivated = BUM_NONE;		// Was bumper activated?
	hist.cruiseSpeed = 0;
	hist.coilsOutsideAngle = 0;
	return hist;
}


void Blackboard::printHistoryEntry(int x) {
	errorHandler.setInfo(F("============================\r\n"));
	errorHandler.setInfo(F("!05,Idx:             %d \r\n"), x);
	errorHandler.setInfo(F("!05,driveDirection   %s\r\n"), enuDriveDirectionString[history[x].driveDirection]);
	errorHandler.setInfo(F("!05,coilFirstOutside %s\r\n"), enuFlagCoilsOutsideString[history[x].coilFirstOutside]);
	errorHandler.setInfo(F("!05,coilOutAfterOver %s\r\n"), enuFlagCoilsOutsideString[history[x].coilOutsideAfterOverrun]);
	errorHandler.setInfo(F("!05,bumperActivated  %s\r\n"), enuFlagBumperActivatedString[history[x].bumperActivated]);
	errorHandler.setInfo(F("!05,timeAdded        %d \r\n"), history[x].timeAdded);
	errorHandler.setInfo(F("---\r\n"));
	errorHandler.setInfo(F("!05,distanceSoll   %f\r\n"), history[x].distanceSoll);
	errorHandler.setInfo(F("!05,distanceIst   %f\r\n"), history[x].distanceIst);
	errorHandler.setInfo(F("!05,cruiseSpeed   %d\r\n"), history[x].cruiseSpeed);
	errorHandler.setInfo(F("!05,coilsOutsideAngle   %f\r\n"), history[x].coilsOutsideAngle);
	errorHandler.setInfo(F("!05,restored         %d \r\n"), history[x].restored);
}

void Blackboard::historyUpdateDistance() {

	if (history0.driveDirection == DD_ROTATECC || history0.driveDirection == DD_ROTATECW) {
		history0.distanceIst = srvMotor.getAngleRotatedAngleDeg();
	}
	else {
		history0.distanceIst = srvMotor.getDistanceInCM();
	}
}

void Blackboard::deleteLastHistoryEntry() {

	//history0.restored = true;

	errorHandler.setInfo(F("!03,delete deleteLastHistoryEntry driveDirection: %s \r\n"), enuDriveDirectionString[history0.driveDirection]);

	//Delete History entry [0] while shifting to left
	for (int i = 0; i < HISTROY_BUFSIZE - 1; i++) {
		history[i] = history[i + 1];
	}
	history[HISTROY_BUFSIZE - 1] = getInitialisedHistoryEntry();
}

enuDriveDirection Blackboard::historyGetLastRotateDirection() {
	int i;
	enuDriveDirection dd = DD_ROTATECC;
	for (i = 0; i < HISTROY_BUFSIZE; i++) {
		if (history[i].driveDirection == DD_ROTATECC  || history[i].driveDirection == DD_ROTATECW) {
			dd = history[i].driveDirection;
			break;
		}
	}
	return dd;

}

int8_t Blackboard::histGetThreeLastForwardDistances(float& a, float& b, float& c) {

	int i, j, h;
	bool aFound, bFound, cFound;
	j = HISTROY_BUFSIZE;
	h = HISTROY_BUFSIZE;
	aFound = false;
	bFound = false;
	cFound = false;

	for (i = 0; i < HISTROY_BUFSIZE; i++) {
		if (history[i].driveDirection == DD_FORWARD) {
			a = history[i].distanceIst;
			aFound = true;
			j = i + 1;
			break;
		}
	}

	for (; j < HISTROY_BUFSIZE; j++) {
		if (history[j].driveDirection == DD_FORWARD) {
			b = history[j].distanceIst;
			bFound = true;
			h = j + 1;
			break;
		}
	}


	for (; h < HISTROY_BUFSIZE; h++) {
		if (history[h].driveDirection == DD_FORWARD) {
			c = history[h].distanceIst;
			cFound = true;
			break;
		}
	}

	if (flagShowRotateX) {
		errorHandler.setInfo(F("!5,histGTLFD result: %d forwDistA: %f forwDistB: %f forwDistC: %f\r\n"), aFound && bFound && cFound, a, b, c);
	}

	if (aFound && bFound && cFound) {
		return 3;
	}

	if (aFound && bFound) {
		return 2;
	}

	return 0;

}


void Blackboard::resetBB() {
	errorHandler.setInfo(F("Blackboard:reset\r\n"));


	flagEnableRestoreHistory = false;
	flagCruiseSpiral = false;

	//errorHandler.setInfo(F("bht->reset disable flagCruiseSpiral\r\n"));
	flagForceSmallRotAngle = 0;


	flagForceRotateDirection = FRD_NONE;

	//arcRotateXArc = 0;

	lastTimeSpiralStarted = millis() + 60000ul; // Erst eine Minute nach Reset warten, bevor Sprirale aktiviert werden kann.

	areaTargetDistanceInMeter = 2;

	flagGotoAreaXFirstCall = false;
	flagGoHome = false;

	flagBHTShowLastNode = CONF_bb_flagBHTShowLastNode;

	
	THistory hist = getInitialisedHistoryEntry();
	for (int i = 0; i < HISTROY_BUFSIZE; i++) {
		history[i] = hist;
	}

	randAngle = 0;
	flagForceAngle = false;
	
	flagShowRotateX = false;
	flagShowHistory = false;

	flagRotateAtPer = false;
	flagDriveCurve = false;
	
}