
#include "config.h"
#if CONF_USE_ADVANCED_PERIMETER_SERVICE ==  true
//###########################################################
//###########################################################
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

//#include <algorithm>   //for min/max
#include "perimeter.h"
#include "helpers.h"
#include "hardware.h"
#include "errorhandler.h"
#include "config.h"
#include "gps.h"

//GPS Service
extern Tgps gps;

#define I2C_MT_SHORT_RESULT 0
#define I2C_MT_AMPLITUDE_RESULT 1
#define I2C_MT_DETAIL_RESULT 2
#define I2C_MT_AGC_RESULT 3

#define BYTE_TO_BINARY_PATTERN "Decoded L: %c R: %c VL: %c VR: %c Back: %c Counter: %c%c%c\r\n"
#define BYTE_TO_BINARY(byte)           \
      (byte & 0x80 ? '1' : '0'),       \
            (byte & 0x40 ? '1' : '0'), \
            (byte & 0x20 ? '1' : '0'), \
            (byte & 0x10 ? '1' : '0'), \
            (byte & 0x08 ? '1' : '0'), \
            (byte & 0x04 ? '1' : '0'), \
            (byte & 0x02 ? '1' : '0'), \
            (byte & 0x01 ? '1' : '0')



union uFloat {
	uint8_t uBytes[4];
	float sFloat;
};

union uInt32 {
	uint8_t uBytes[4];
	int32_t sIn32t;
};

union uInt16 {
	uint8_t uBytes[2];
	int16_t sIn16t;
};

union uInt8 {
	uint8_t uBytes[1];
	int8_t sInt8;
};

void TPerimeterThread::setup() {

	magnetudeL = 0;       // maximum des korrelationssignals. Kann auch negative sein. Amplitude von kleiner +-40 ist rauschen
	magnetudeL0 = 0;
	lastTimeSignalReceivedL = 0;

	magnetudeR = 0;
	magnetudeR0 = 0;
	lastTimeSignalReceivedR = 0;
	magMax = 0;

	showValuesOnConsole = false;


	signalCounterL = 0;    // 2 outside >=  wert  >=-2 inside
	signalCounterR = 0;

	packetCounter = 0;
	lastPacketCounter = 0;
	shortResult = 0;
	state = 0;
	testcounter = 0;
}


void TPerimeterThread::CaluculateInsideOutsideL(int32_t magl) {

	if (CONF_LEFT_COIL_INVERSE) {
		magl *= -1;
	}


	if (magl == 0) {
		magnetudeL0 = magl;   //needed for TlineFollow
	}
	else {
		magnetudeL = magl;
		magnetudeL0 = magl;
	}

	//----------------------------------------
	// Evaluate left
	//----------------------------------------

	// ** inside mag positive**
	if (magl > 0) {

		signalCounterL = min(signalCounterL + 1, 3);
		lastTimeSignalReceivedL = millis();


	}

	// ** outside mag negative**
	else if (magl < 0) {
		signalCounterL = max(signalCounterL - 1, -3);
		lastTimeSignalReceivedL = millis();
	}

	// ** lost ** _magnetudeL.sIn16t == 0
	else {
		//signalCounterL = max(signalCounterL - 1, -3);
	}

	// Overwrite values when inside GPS polygon
	if (CONF_USE_GPS_POLYGON) // Check if the gps signal shows, that robot is inside the defined gps polygon.
	{
		if (gps.flagInsidePolygon && abs(magnetudeL) < CONF_PER_THRESHOLD_IGNORE_GPS) // only check if amplitude is lower than threshold
		{
			signalCounterL = 3;
			lastTimeSignalReceivedL = millis();
		}
	}
}

void TPerimeterThread::CaluculateInsideOutsideR(int32_t magr) {
	//----------------------------------------
	// Evaluate right
	//----------------------------------------


	if (CONF_RIGHT_COIL_INVERSE) {
		magr *= -1;
	}


	if (magr == 0) {
		magnetudeR0 = magr;
	}
	else {
		magnetudeR = magr;
		magnetudeR0 = magr;
	}


	if (magr > 0) {

		signalCounterR = min(signalCounterR + 1, 3);
		lastTimeSignalReceivedR = millis();

	}
	else if (magr < 0) {
		signalCounterR = max(signalCounterR - 1, -3);
		lastTimeSignalReceivedR = millis();

	}
	else {
		//signalCounterR = max(signalCounterR - 1, -3);
	}

	// Overwrite values when inside GPS polygon
	if (CONF_USE_GPS_POLYGON) // Check if the gps signal shows, that robot is inside the defined gps polygon.
	{
		if (gps.flagInsidePolygon && abs(magnetudeR) < CONF_PER_THRESHOLD_IGNORE_GPS) // only check if amplitude is lower than threshold
		{
			signalCounterR = 3;
			lastTimeSignalReceivedR = millis();
		}
	}

}

/*
void TPerimeterThread::UpdateState(EPerReceiveState t) {

	switch (t) {
	case SPR_WAIT_COILL_SAMPLE_COMPLETED:
		if (aiCoilLeft.isConvComplete()) {
			aiCoilRight.restartConv();
			SetState(SPR_COILL_CALCULATE);
		}
		break;

	case SPR_COILL_CALCULATE:
		coilL.run();
		if (coilL.state == 0) {
			if (coilL.isSignalValid()) {
				CaluculateInsideOutsideL(coilL.magnetude);
			}
			else {
				CaluculateInsideOutsideL(0);
			}
			SetState(SPR_WAIT_COILR_SAMPLE_COMPLETED);
			//errorHandler.setInfoNoLog(F("CompL %lu\r\n"), millis());

		}
		break;

	case SPR_WAIT_COILR_SAMPLE_COMPLETED:
		if (aiCoilRight.isConvComplete()) {
			//errorHandler.setInfoNoLog(F("ResL %lu\r\n"), millis());
			aiCoilLeft.restartConv();
			SetState(SPR_COILR_CALCULATE);
		}
		break;

	case SPR_COILR_CALCULATE:
		coilR.run();
		if (coilR.state == 0) {
			if (coilR.isSignalValid()) {
				CaluculateInsideOutsideR(coilR.magnetude);
			}
			else {
				CaluculateInsideOutsideR(0);
			}
			SetState(SPR_WAIT_COILL_SAMPLE_COMPLETED);
			if (showValuesOnConsole) {
				//sprintf(errorHandler.msg, "!03,ML: %d MR: %d  CL:%d CR: %d\r\n", magnetudeL, magnetudeR, signalCounterL, signalCounterR);
				//sprintf(errorHandler.msg, "!03,ML: %d MR: %d magMax:%d magMedL: %d magMedR: %d\r\n", magnetudeL, magnetudeR, magMax, (int)curMaxL,  (int)curMaxR);
				sprintf(errorHandler.msg, "!03,ML: %d/%d MR: %d/%d magMax:%d magMedL%%: %d magMedR%%: %d\r\n", magnetudeL, signalCounterL, magnetudeR, signalCounterR, magMax, (int)curMaxL * 100 / magMax, (int)curMaxR * 100 / magMax);
				errorHandler.setInfoNoLog();
			}
		}

		break;

	default:
		//TODO invalid state - reset, perhaps?
		break;
	}
}
*/

void TPerimeterThread::showConfig() {
	errorHandler.setInfoNoLog(F("!03,Advanced Perimeter Sensor Config\r\n"));
	errorHandler.setInfoNoLog(F("!03,enabled: %lu\r\n"), enabled);
	errorHandler.setInfoNoLog(F("!03,interval: %lu\r\n"), interval);
}

void TPerimeterThread::run() {
	static int coil = 0;
	int ret;
	union uInt16 convert;

	runned();
	if (CONF_DISABLE_PERIMETER_SERVICE) {
		magnetudeL = 1000;
		magnetudeL0 = 1000;
		magnetudeR = 1000;
		magnetudeR0 = 1000;
		return;
	}


	switch (state) {
	case 0:

		if (i2cAPR.read8Only(1, &shortResult, 1) != 1) {
			errorHandler.setInfo(F("APR comm error\r\n"));
			return;
		}
		errorHandler.setInfo(F("Short result: %u\r\n"), (unsigned int)shortResult);
		errorHandler.setInfo(F(BYTE_TO_BINARY_PATTERN), BYTE_TO_BINARY(shortResult));

		packetCounter = shortResult & 0x7;;
		if (lastPacketCounter != packetCounter) {
			lastPacketCounter = packetCounter;
			state = 1;
			interval = 1;
		}
		//interval = 100; 
		break;
	case 1:
		ret = i2cAPR.read8Only(10, &RxBuf[0], 1);

		RxValues.result = RxBuf[0];
		RxValues.potL = RxBuf[1];
		RxValues.potR = RxBuf[2];

		convert.uBytes[0] = RxBuf[3];
		convert.uBytes[1] = RxBuf[4];
		RxValues.L = convert.sIn16t;
		convert.uBytes[0] = RxBuf[5];
		convert.uBytes[1] = RxBuf[6];
		RxValues.R = convert.sIn16t;

		RxValues.ratioL = RxBuf[7];
		RxValues.ratioR = RxBuf[8];
		RxValues.resetCnt = RxBuf[9];

		errorHandler.setInfo(F("L: %d R: %d result: %u PotL: %u PotR: %u RatioL: %u RatioR: %u I2Creset: %u\r\n"),
			RxValues.L, RxValues.R,
			(unsigned int)RxValues.result,
			(unsigned int)RxValues.potL, (unsigned int)RxValues.potR,
			(unsigned int)RxValues.ratioL, (unsigned int)RxValues.ratioR,
			(unsigned int)RxValues.resetCnt);

		state = 0;
		interval = 10;
		testcounter++;

		if (testcounter == 40) {
			testcounter = 0;
			state = 2;
		}
		break;

	case 2:
		if (coil == 0) {
			coil = 1;
			// 0xB will be send RxBuf is only a dummy write with len 0
			ret = i2cAPR.write8(0xB, 0, &RxBuf[0]);
		}
		else {
			coil = 0;
			ret = i2cAPR.write8(0xF, 0, &RxBuf[0]);
		}
		state = 0;
		interval = 10;

		break;
	default:
		interval = 10;
		state = 0;
		break;

	}


}

// ------------------------------------------------------------------------------------------
// Folgende Funktionen werden von bMow aufgerufen um Max des Perimeters zu bestimmen
// ------------------------------------------------------------------------------------------
bool TPerimeterThread::isNearPerimeter() {

	if (signalCounterL <= 0 || signalCounterR <= 0) {
		return true;
	}

	return false;
}




bool TPerimeterThread::isLeftInside() {
	// Low signal, use filtered value for increased reliability
	return (signalCounterL > 0);
}


bool TPerimeterThread::isRightInside() {

	// Low signal, use filtered value for increased reliability
	return (signalCounterR > 0);
}


bool TPerimeterThread::isLeftOutside() {

	// Low signal, use filtered value for increased reliability
	return (signalCounterL <= 0);

}

bool TPerimeterThread::isRightOutside() {

	// Low signal, use filtered value for increased reliability
	return (signalCounterR <= 0);
}







//###########################################################
#endif //#if CONF_USE_ADVANCED_PERIMETER_SERVICE ==  true



