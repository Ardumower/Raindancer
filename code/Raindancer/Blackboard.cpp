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

#include "Blackboard.h"


const char* enuDriveDirectionString[] = { "DD_FORWARD",
										"DD_FORWARD_INSIDE",
										"DD_OVERRUN",
										"DD_REVERSE_ESC_OBST",
										"DD_REVERSE_INSIDE",
										"DD_ROTATECW",
										"DD_ROTATECC",
										"DD_REVERSE_LINE_FOLLOW",
										"DD_SPIRAL_CW",
										"DD_FEOROTATECC",
										"DD_FEOROTATECW",
										"DD_FREEBUMPER", // only used for history
										"DD_NONE",
										//"DD_ROTATE90CW",
										//"DD_ROTATE90CC",
										"UNKNOWN4",
										"UNKNOWN5",
										"UNKNOWN6"
};

const char* enuFlagEscabeObstacleConFlagString[] = { "FEO_NONE",
		"FEO_ROTCC",
		"FEO_ROTCW",
		"FEO_BACKINSIDE",
		"FEO_ROT",
		"UNKNOWN3",
		"UNKNOWN4",
		"UNKNOWN4",
		"UNKNOWN4",
		"UNKNOWN5"
};


const char* enuFlagForceRotateDirectionString[] = { "FRD_NONE",
	"FRD_ROTCC",
	"FRD_ROTCW",
	"UNKNOWN1",
	"UNKNOWN2",
	"UNKNOWN3",
	"UNKNOWN4",
	"UNKNOWN5"
};


const char* enuFlagCoilsOutsideString[] = { "CO_NONE",
"CO_RIGHT",
"CO_LEFT",
"CO_BOTH",
"UNKNOWN2",
"UNKNOWN3",
"UNKNOWN4",
"UNKNOWN5"
};


void Blackboard::setBehaviour(enuBehaviour b)
{

	// save statistic
#if CONF_DISABLE_EEPROM_SERVICE == false
	if (flagEnableMowing == true) { //only save if in mowing mode
		errorHandler.setInfo("!04,Write Stats to EEPROM\r\n");

		float mowtime = eeprom.readFloat(EEPADR_MOWTIME);
		unsigned long time = millis() - timeInMowBehaviour;
		double hours = (double)time / 3600000.0;
		mowtime += hours;
		eeprom.writeFloat(EEPADR_MOWTIME, mowtime);


		float mowway = eeprom.readFloat(EEPADR_MOWDIRVENWAY);
		float ticks = (motor.L->myEncoder->getAbsTicksCounter() + motor.R->myEncoder->getAbsTicksCounter()) / 2.0f;
		mowway += motor.getMForCounts(ticks);
		eeprom.writeFloat(EEPADR_MOWDIRVENWAY, mowway);
		
		int32_t rotations = eeprom.read32t(EEPADR_ROTATIONCOUNT);
		rotations = rotations + numberOfRotations;
		eeprom.write32t(EEPADR_ROTATIONCOUNT, rotations);

	}
#endif

	flagEnableMowing = false;
	flagEnablePerimetertracking = false;
	flagEnableCharging = false;
	flagEnableGotoAreaX = false;
	flagEnableFindPerimeter = false;
	flagEnableRestoreHistory = false;
	motor.enableDefaultRamping();

	switch (b) {
	case BH_GOTOAREA:
		resetBB();  //Reset BB because here starts a new mow sequence
		flagEnableGotoAreaX = true;
		flagGotoAreaXFirstCall = true;
		//rangeSensor.enabled = false;
		chargeSystem.deactivateRelay();  //Muss hier auch abgeschaltet werden, falls user dieses ueber das UI einschaltet.
		//motor.mowMotStop(); // Will be started in BH_MOW behaviour tree.
		motor.stopAllMotors(); // Nur zur Sicherheit, falls diese gerade laufen.
		motor.startDistanceMeasurementAreax();
		errorHandler.setInfo("!04,SET BEHAV -> BH_GOTOAREA\r\n");
		break;
	case BH_CHARGING:
		flagEnableCharging = true;
		//rangeSensor.enabled = false;
		motor.mowMotStop();
		errorHandler.setInfo("!04,SET BEHAV -> BH_CHARGING\r\n");
		break;
	case BH_PERITRACK:
		flagEnablePerimetertracking = true;
		//rangeSensor.enabled = false;
		chargeSystem.deactivateRelay();
		//motor.mowMotStop();
		errorHandler.setInfo("!04,SET BEHAV -> BH_PERITRACK\r\n");
		break;
	case BH_FINDPERIMETER:
		flagEnableFindPerimeter = true;
		//rangeSensor.enabled = true;
		chargeSystem.deactivateRelay();
		//motor.mowMotStop();
		errorHandler.setInfo("!04,SET BEHAV -> BH_FINDPERIMETER\r\n");
		break;
	case BH_MOW:
		resetBB(); //Reset BB because here starts a new mow sequence
		myRandom(0, 5000, true); // init seed value
		flagEnableMowing = true;
		//rangeSensor.enabled = true;
		chargeSystem.deactivateRelay();
		// reset statistic counter
		motor.resetEncoderCounter();
		numberOfRotations = 0;
		timeInMowBehaviour = millis();
		//motor.startDistanceMeasurement();
		errorHandler.setInfo("!04,SET BEHAV -> BH_MOW\r\n");
		break;
	case BH_RESTOREHISTORY:
		flagEnableRestoreHistory = true;
		//rangeSensor.enabled = true;
		chargeSystem.deactivateRelay();
		//motor.resetEncoderCounter();
		//motor.startDistanceMeasurement();
		errorHandler.setInfo("!04,SET BEHAV -> BH_RESTOREHISTORY\r\n");
		break;
	case BH_NONE:
		// Don't reset BB here because if you change in manual mode, you will probaly check the BB variables like history or so.
		//rangeSensor.enabled = false;
		chargeSystem.deactivateRelay();
		motor.mowMotStop();
		errorHandler.setInfo("!04,SET BEHAV -> BH_NONE\r\n");
		break;
	default:
		errorHandler.setError("!04,setBehaviour unbekanntes Behaviour\r\n");
	}
}

//void Blackboard::addHistoryEntry(THistory &h) {

void Blackboard::addHistoryEntry(enuDriveDirection _driveDirection, float  _distanceDriven , float _rotAngleSoll, float _rotAngleIst, 
	                             enuFlagForceRotateDirection _flagForceRotDirection, enuFlagCoilsOutside   _coilFirstOutside) {
	//Shift History for new entry
	for (int i = HISTROY_BUFSIZE - 1; i > 0; i--) {
		history[i] = history[i - 1];
	}

	history[0].driveDirection = _driveDirection;
	history[0].distanceDriven = _distanceDriven;
	history[0].rotAngleSoll = _rotAngleSoll;
	history[0].rotAngleIst = _rotAngleIst;
	history[0].flagForceRotDirection = _flagForceRotDirection;
	history[0].coilFirstOutside = _coilFirstOutside;

	history[0].restored = false;
	history[0].timeAdded = millis();
	
	motor.startDistanceMeasurement();


	if (flagShowHistory == true) {

		errorHandler.setInfo(F("============================\r\n"));
		sprintf(errorHandler.msg, "!05,driveDirection %s %s %s %s\r\n", enuDriveDirectionString[history[0].driveDirection], enuDriveDirectionString[history[1].driveDirection], enuDriveDirectionString[history[2].driveDirection], enuDriveDirectionString[history[3].driveDirection]);
		errorHandler.setInfo();

		sprintf(errorHandler.msg, "!05,coilFirstOut %s %s %s %s\r\n", enuFlagCoilsOutsideString[history[0].coilFirstOutside], enuFlagCoilsOutsideString[history[1].coilFirstOutside], enuFlagCoilsOutsideString[history[2].coilFirstOutside], enuFlagCoilsOutsideString[history[3].coilFirstOutside]);
		errorHandler.setInfo();

		sprintf(errorHandler.msg, "!05,rotAngleSoll %3.2f %3.2f %3.2f %3.2f\r\n", history[0].rotAngleSoll, history[1].rotAngleSoll, history[2].rotAngleSoll, history[3].rotAngleSoll);
		errorHandler.setInfo();

		sprintf(errorHandler.msg, "!05,flagForceRotDir %s %s %s %s\r\n", enuFlagForceRotateDirectionString[history[0].flagForceRotDirection], enuFlagForceRotateDirectionString[history[1].flagForceRotDirection], enuFlagForceRotateDirectionString[history[2].flagForceRotDirection], enuFlagForceRotateDirectionString[history[3].flagForceRotDirection]);
		errorHandler.setInfo();

		errorHandler.setInfo(F("---\r\n"));

		sprintf(errorHandler.msg, "!05,distanceDriven %3.2f %3.2f %3.2f %3.2f\r\n", history[0].distanceDriven, history[1].distanceDriven, history[2].distanceDriven, history[3].distanceDriven);
		errorHandler.setInfo();

		sprintf(errorHandler.msg, "!05,rotAngleIst %3.2f %3.2f %3.2f %3.2f\r\n", history[0].rotAngleIst, history[1].rotAngleIst, history[2].rotAngleIst, history[3].rotAngleIst);
		errorHandler.setInfo();

		sprintf(errorHandler.msg, "!05,restored  %d %d %d %d\r\n", history[0].restored, history[1].restored, history[2].restored, history[3].restored);
		errorHandler.setInfo();

		sprintf(errorHandler.msg, "!05,timeAdded %lu %lu %lu %lu\r\n", history[0].timeAdded, history[1].timeAdded, history[2].timeAdded, history[3].timeAdded);
		errorHandler.setInfo();
	}

}


void Blackboard::markLastHistoryEntryAsRestored() {

	history[0].restored = true;
	/*
	//Delete History entry [0] while shifting to left
	for (int i = 0 ; i < HISTROY_BUFSIZE-1; i++) {
		history[i] = history[i+1];
	}

	history[HISTROY_BUFSIZE - 1].distanceDriven = 300; // [0] Enthaelt die gerade gefahrene distanz von der letzten rotation bis jetzt. Jedes mal nachdem rotiert wurde, wird distanzmessung neu gestartet.
	history[HISTROY_BUFSIZE - 1].coilFirstOutside = CO_NONE; // [0] which coil was first outside jet
	history[HISTROY_BUFSIZE - 1].driveDirection = DD_NONE;; // [0]  Enthaelt rotationsrichtung der aktuellen Drehung.
	history[HISTROY_BUFSIZE - 1].rotAngleSoll = 0; // [0] Sollwinkel der aktuellen Drehung
	history[HISTROY_BUFSIZE - 1].rotAngleIst = 0;
	history[HISTROY_BUFSIZE - 1].flagForceRotDirection = FRD_NONE;
	*/
}


bool Blackboard::histGetTwoLastForwardDistances(float& a, float& b) {
	
	int i,j;
	bool aFound, bFound;
	j = HISTROY_BUFSIZE;
	aFound = false;
	bFound = false;

	for (i = 0; i < HISTROY_BUFSIZE; i++) {
		if (history[i].driveDirection == DD_FORWARD) {
			a = history[i].distanceDriven;
			aFound = true;
			j = i + 1;
			break;
		}
	}

	for (; j < HISTROY_BUFSIZE; j++) {
		if (history[j].driveDirection == DD_FORWARD) {
			b = history[j].distanceDriven;
			bFound = true;
			break;
		}
	}

	if (flagShowRotateX) {
		errorHandler.setInfo(F("!5,histGTLFD result: %d forwDistA: %f forwDistB: %f\r\n"), aFound && bFound, a, b);
	}

	if (aFound && bFound) {
		return true;
	}

	return false;

}

void Blackboard::resetBB()
{
	errorHandler.setInfo(F("Blackboard:reset\r\n"));
	// Init black board variables
	lastNodeLastRun = NULL;
	lastNodeCurrentRun = NULL;

	cruiseSpeed = 0;
	timeCruiseSpeedSet = 0;

	flagBumperInsidePerActivated = false;
	flagBumperOutsidePerActivated = false;
	flagCruiseSpiral = false;

	//errorHandler.setInfo(F("bht->reset disable flagCruiseSpiral\r\n"));

	flagCoilFirstOutside = CO_NONE;
	flagCoilFirstOutsideLatched = CO_NONE;
	//flagCoilOutsideAfterOverrun  = CO_BOTH;
	flagForceSmallRotAngle = 0;

	flagEscabeObstacleConFlag = FEO_NONE;

	flagForceRotateDirection = FRD_NONE;

	arcRotateXArc = 0;

	lastTimeSpiralStarted = millis() + 60000ul; // Erst eine Minute nach Reset warten, bevor Sprirale aktiviert werden kann.

	areaTargetDistanceInMeter = 2;

	flagGotoAreaXFirstCall = false;
	flagGoHome = false;

	flagBHTShowLastNode = CONF_bb_flagBHTShowLastNode;

	if (perimeterSensoren.isLeftOutside() || perimeterSensoren.isRightOutside()) {
		driveDirection = DD_REVERSE_INSIDE;
	}
	else {
		driveDirection = DD_FORWARD;
	}

	for (int i = 0; i < HISTROY_BUFSIZE; i++) {
		history[i].distanceDriven = 300; // [0] Enthaelt die gerade gefahrene distanz von der letzten rotation bis jetzt. Jedes mal nachdem rotiert wurde, wird distanzmessung neu gestartet.
		history[i].coilFirstOutside = CO_NONE; // [0] which coil was first outside jet
		history[i].driveDirection = DD_NONE;; // [0]  Enthaelt rotationsrichtung der aktuellen Drehung.
		history[i].rotAngleSoll = 0; // [0] Sollwinkel der aktuellen Drehung
		history[i].rotAngleIst = 0;
		history[i].flagForceRotDirection = FRD_NONE;
		history[i].restored = false;
	}

	numberToRestoreHist = 0;
	randAngle = 0;
	flagForceAngle = false;
	flagDeactivateRotInside = false;;

	flagShowRotateX = false;
	flagShowHistory = false;

	flagRotateAtPer = false;
	flagDriveCurve = false;
	flagEnableSecondReverse = false;
}