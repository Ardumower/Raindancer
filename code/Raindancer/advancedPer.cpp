
#include "config.h"
#if CONF_USE_ADVANCED_PERIMETER_SERVICE ==  true
//###########################################################
//###########################################################
/*
Robotic Lawn Mower
Copyright (c) 2019 by Kai Würtz

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
#include "helpers.h"

//GPS Service
extern Tgps gps;

#define I2C_POLL_INTERVALL 5 //ms
#define I2C_MT_SHORT_RESULT 0
#define I2C_MT_AMPLITUDE_RESULT 1
#define I2C_MT_DETAIL_RESULT 2
#define I2C_MT_AGC_RESULT 3

#define BYTE_TO_BINARY_PATTERN "!03,Result iL: %c iR: %c vL: %c vR: %c back: %c np: %c counter: %c%c\r\n"
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

	magnetudeL0 = 0;
	lastTimeSignalReceivedL = 0;

	magnetudeR0 = 0;
	lastTimeSignalReceivedR = 0;

	showValuesOnConsole = false;

	magMax = 0;

	signalCounterL = 0;    // 2 outside >=  wert  >=-2 inside
	signalCounterR = 0;

	lastPacketCounter = 0;
	state = 0;
	testcounter = 0;

	// initiate RxShortResults
	DecodeResults(0);

}


void TPerimeterThread::CaluculateInsideOutsideL(void) {

	// Inverse coil 
	if (CONF_LEFT_COIL_INVERSE) {
		if (RxShortResults.insideL == 1) {
			RxShortResults.insideL = 0;
		}
		else {
			RxShortResults.insideL = 1;
		}
	}


	// Set magnitude for linefollowing
	if (RxShortResults.validL == 0) {
		magnetudeL0 = 0;   //needed for TlineFollow
	}
	else {
		if (RxShortResults.insideL == 1) {
			magnetudeL0 = 1;
		}
		else {
			magnetudeL0 = -1;
		}
	}

	//----------------------------------------
	// Evaluate left
	//----------------------------------------

	if (RxShortResults.validL > 0) {
		// ** inside mag positive**
		if (RxShortResults.insideL == 1) {
			signalCounterL = min(signalCounterL + 1, 2);
			lastTimeSignalReceivedL = millis();
		}
		// ** outside mag negative**
		else if (RxShortResults.insideL == 0) {
			signalCounterL = max(signalCounterL - 1, -1);
			lastTimeSignalReceivedL = millis();
		}
	}

	// Overwrite values when inside GPS polygon
	if (CONF_USE_GPS_POLYGON) // Check if the gps signal shows, that robot is inside the defined gps polygon.
	{
		if (gps.flagInsidePolygon && !RxShortResults.validL) // only check if amplitude is lower than threshold
		{
			signalCounterL = 3;
			lastTimeSignalReceivedL = millis();
		}
	}
}

void TPerimeterThread::CaluculateInsideOutsideR(void) {
	//----------------------------------------
	// Evaluate right
	//----------------------------------------

	if (CONF_RIGHT_COIL_INVERSE) {
		if (RxShortResults.insideR == 1) {
			RxShortResults.insideR = 0;
		}
		else {
			RxShortResults.insideR = 1;
		}
	}


	// Set magnitude for linefollowing
	if (RxShortResults.validR == 0) {
		magnetudeR0 = 0;   //needed for TlineFollow
	}
	else {
		if (RxShortResults.insideR == 1) {
			magnetudeR0 = 1;
		}
		else {
			magnetudeR0 = -1;
		}
	}


	//----------------------------------------
	// Evaluate Right
	//----------------------------------------


	if (RxShortResults.validR > 0) {
		// ** inside mag positive**
		if (RxShortResults.insideR == 1) {
			signalCounterR = min(signalCounterR + 1, 2);
			lastTimeSignalReceivedR = millis();
		}

		// ** outside mag negative**
		else if (RxShortResults.insideR == 0) {
			signalCounterR = max(signalCounterR - 1, -1);
			lastTimeSignalReceivedR = millis();
		}
	}

	// Overwrite values when inside GPS polygon
	if (CONF_USE_GPS_POLYGON) // Check if the gps signal shows, that robot is inside the defined gps polygon.
	{
		if (gps.flagInsidePolygon && !RxShortResults.validR) // only check if signal not valid
		{
			signalCounterR = 3;
			lastTimeSignalReceivedR = millis();
		}
	}

}


void TPerimeterThread::showConfig() {
	errorHandler.setInfoNoLog(F("!03,Advanced Perimeter Sensor Config\r\n"));
	errorHandler.setInfoNoLog(F("!03,enabled: %lu\r\n"), enabled);
	errorHandler.setInfoNoLog(F("!03,interval: %lu\r\n"), interval);
}


void TPerimeterThread::DecodeResults(uint8_t shortResult) {
	// ---- Set short result byte used by I2C ----
	// Bits: 8		 7	     6	  5	     4	      3			2-1
	// Bits: L in/out, R in/out, L valid, R valid, F or B coil, near perimeter, counter
	RxShortResults.result = shortResult;
	RxShortResults.insideL = BitTest(shortResult, 7);
	RxShortResults.insideR = BitTest(shortResult, 6);
	RxShortResults.validL = BitTest(shortResult, 5);
	RxShortResults.validR = BitTest(shortResult, 4);
	RxShortResults.backCoilActive = BitTest(shortResult, 3);
	RxShortResults.nearPerimeter = BitTest(shortResult, 2);
	RxShortResults.packetCounter = shortResult & 0x03;
}

void TPerimeterThread::run() {
	static int coil = 0;
	uint8_t shortResult = 0;
	union uInt16 convert;
	uint8_t packetCounter = 0;
	uint8_t temp = 0;

	runned();

	if (CONF_DISABLE_PERIMETER_SERVICE) {
		magnetudeL0 = 1000;
		magnetudeR0 = 1000;
		DecodeResults(0xF0);
		interval = 100;
		lastTimeSignalReceivedL = millis();
		lastTimeSignalReceivedR = millis();
		signalCounterL = 2;
		signalCounterR = 2;
		return;
	}


	switch (state) {
	case 0: // poll APR if new data is available

		if (i2cAPR.read8Only(1, &shortResult, 1) != 1) {
			errorHandler.setInfo(F("!03,APR 0 comm error errorCountert:%d time %lu\r\n"), i2cAPR.i2cErrorcounter, millis());
			return;
		}
		/*
		delay(10);
		i2cAPR.write8(0xB, 0, &RxBuf[0]);
		*/
		/*
		if (showValuesOnConsole) {
			errorHandler.setInfo(F(BYTE_TO_BINARY_PATTERN), BYTE_TO_BINARY(shortResult));
		}
		*/

		// check if packetcounter changed. If yes, new data are available.
		packetCounter = shortResult & 0x3;
		if (lastPacketCounter != packetCounter) {
			lastPacketCounter = packetCounter;

			RxValues.result = shortResult;
			DecodeResults(RxValues.result);
			CaluculateInsideOutsideL();
			CaluculateInsideOutsideR();

			if (showValuesOnConsole) {
				//errorHandler.setInfo(F(BYTE_TO_BINARY_PATTERN), BYTE_TO_BINARY(shortResult));
				errorHandler.setInfo(F("!03,decode result: %d iL: %d iR: %d vL: %d vR: %d back: %d np: %d counter: %d\r\n"),
					RxShortResults.result, RxShortResults.insideL, RxShortResults.insideR, RxShortResults.validL, RxShortResults.validR,
					RxShortResults.backCoilActive, RxShortResults.nearPerimeter, RxShortResults.packetCounter);
				state = 1;
				interval = 1;
			}
		}
		break;
	case 1:
		if (i2cAPR.read8Only(9, &RxBuf[0], 1) != 9) {
			errorHandler.setInfo(F("!03,APR 1 comm error errorCountert:%d\r\n"), i2cAPR.i2cErrorcounter);
			state = 0;
			interval = I2C_POLL_INTERVALL;
			return;
		}

		//RxValues.result = RxBuf[0];
		convert.uBytes[0] = RxBuf[1];
		convert.uBytes[1] = RxBuf[2];
		RxValues.magnitudeL = convert.sIn16t;

		convert.uBytes[0] = RxBuf[3];
		convert.uBytes[1] = RxBuf[4];
		RxValues.magnitudeR = convert.sIn16t;

		RxValues.ratioL = RxBuf[5];
		RxValues.ratioR = RxBuf[6];
		RxValues.resetCnt = RxBuf[7];

		temp = RxBuf[8];

		if (temp & 0x1) {
			RxValues.AMPOverdriveDetectedL = 1;
		}
		else {
			RxValues.AMPOverdriveDetectedL = 0;
		}

		if (temp & 0x2) {
			RxValues.AMPOverdriveDetectedR = 1;
		}
		else {
			RxValues.AMPOverdriveDetectedR = 0;
		}
		

		errorHandler.setInfo(F("!03,aL: %d/%d aR: %d/%d ratioL: %u ratioR: %u result: %u I2Creset: %u oL: %d oR: %d \r\n"),
			RxValues.magnitudeL, signalCounterL, RxValues.magnitudeR, signalCounterR,
			(unsigned int)RxValues.ratioL, (unsigned int)RxValues.ratioR,
			(unsigned int)RxValues.result, (unsigned int)RxValues.resetCnt,
			(unsigned int)RxValues.AMPOverdriveDetectedL, (unsigned int)RxValues.AMPOverdriveDetectedR);

		state = 0;
		interval = I2C_POLL_INTERVALL;


		//test for coil switch
		/*
		texstcounter++;

		if (testcounter == 40) {
			testcounter = 0;
			state = 2;
		}
		*/
		break;

	case 2: // Test coil switch front to back and vice versa
		if (coil == 0) {
			coil = 1;
			// 0xB will be send RxBuf is only a dummy write with len 0
			i2cAPR.write8(0xB, 0, &RxBuf[0]);
		}
		else {
			coil = 0;
			i2cAPR.write8(0xF, 0, &RxBuf[0]);
		}
		state = 0;
		interval = 10;

		break;

	default:
		state = 0;
		interval = I2C_POLL_INTERVALL;
		break;
	}
}

// ------------------------------------------------------------------------------------------
// Folgende Funktionen werden von bMow aufgerufen um Max des Perimeters zu bestimmen
// ------------------------------------------------------------------------------------------
bool TPerimeterThread::isNearPerimeter() {
	//return false;

	if (RxShortResults.nearPerimeter) {
		//sprintf(errorHandler.msg, "!03,NearPerimeter magMedL: %d magMedR: %d\r\n", (int)curMaxL, (int)curMaxR);
		//errorHandler.setInfoNoLog();
		return true;
	}

	if (signalCounterL <= 0 || signalCounterR <= 0) {
		return true;
	}

	return false;
}


bool TPerimeterThread::isLeftInside() {
	// use filtered value for increased reliability
	return (signalCounterL > 0);
}

bool TPerimeterThread::isRightInside() {
	// use filtered value for increased reliability
	return (signalCounterR > 0);
}

bool TPerimeterThread::isLeftOutside() {
	// use filtered value for increased reliability
	return (signalCounterL < 1);

}

bool TPerimeterThread::isRightOutside() {
	// use filtered value for increased reliability
	return (signalCounterR < 1);
}







//###########################################################
#endif //#if CONF_USE_ADVANCED_PERIMETER_SERVICE ==  true



