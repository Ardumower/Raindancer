
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
#include "helpers.h"

//GPS Service
extern Tgps gps;

#define I2C_MT_SHORT_RESULT 0
#define I2C_MT_AMPLITUDE_RESULT 1
#define I2C_MT_DETAIL_RESULT 2
#define I2C_MT_AGC_RESULT 3

#define BYTE_TO_BINARY_PATTERN "Result iL: %c iR: %c vL: %c vR: %c back: %c counter: %c%c%c\r\n"
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

	// ** inside mag positive**
	if (RxShortResults.validL > 0) {
		if (RxShortResults.insideL == 1) {

			signalCounterL = min(signalCounterL + 1, 3);
			lastTimeSignalReceivedL = millis();
		}

		// ** outside mag negative**
		else if (RxShortResults.insideL == 0) {
			signalCounterL = max(signalCounterL - 1, -2);
			lastTimeSignalReceivedL = millis();
		}
	}

	// Overwrite values when inside GPS polygon
	if (CONF_USE_GPS_POLYGON) // Check if the gps signal shows, that robot is inside the defined gps polygon.
	{
		if (gps.flagInsidePolygon && !RxShortResults.validL < CONF_PER_THRESHOLD_IGNORE_GPS) // only check if amplitude is lower than threshold
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

	// ** inside mag positive**
	if (RxShortResults.validR > 0) {
		if (RxShortResults.insideR == 1) {
			signalCounterR = min(signalCounterR + 1, 3);
			lastTimeSignalReceivedR = millis();
		}

		// ** outside mag negative**
		else if (RxShortResults.insideR == 0) {
			signalCounterR = max(signalCounterR - 1, -2);
			lastTimeSignalReceivedR = millis();
		}
	}

	// Overwrite values when inside GPS polygon
	if (CONF_USE_GPS_POLYGON) // Check if the gps signal shows, that robot is inside the defined gps polygon.
	{
		if (gps.flagInsidePolygon && !RxShortResults.validR < CONF_PER_THRESHOLD_IGNORE_GPS) // only check if signal not valid
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
	// Bits: 7		 6	     5	  4	     3	      2-0
	// Bits: L in/out, R in/out, L valid, R valid, F or B coil, counter
	RxShortResults.result = shortResult;
	RxShortResults.insideL = BitTest(shortResult, 7);
	RxShortResults.insideR = BitTest(shortResult, 6);
	RxShortResults.validL = BitTest(shortResult, 5);
	RxShortResults.validR = BitTest(shortResult, 4);
	RxShortResults.backCoilActive = BitTest(shortResult, 3);
	RxShortResults.packetCounter = shortResult & 0x07;
}
void TPerimeterThread::run() {
	static int coil = 0;
	uint8_t shortResult = 0;
	union uInt16 convert;
	uint8_t packetCounter = 0;

	runned();
	if (CONF_DISABLE_PERIMETER_SERVICE) {
		magnetudeL0 = 1000;
		magnetudeR0 = 1000;
		DecodeResults(0xF0);
		RxValues.potL = 0;
		RxValues.potR = 0;
		return;
	}


	switch (state) {
	case 0: // poll APR if new data is available

		if (i2cAPR.read8Only(1, &shortResult, 1) != 1) {
			errorHandler.setInfo(F("!03,APR 0 comm error errorCountert:%d\r\n"), i2cAPR.i2cErrorcounter);
			return;
		}

		if (showValuesOnConsole) {
			//errorHandler.setInfo(F(BYTE_TO_BINARY_PATTERN), BYTE_TO_BINARY(shortResult));
		}

		// check if packetcounter changed. If yes, new data are available.
		packetCounter = shortResult & 0x7;
		if (lastPacketCounter != packetCounter) {
			lastPacketCounter = packetCounter;
			state = 1;
			interval = 1;
		}
		break;
	case 1:
		// Read Pot values to check if near perimeter
		if (i2cAPR.read8Only(3, &RxBuf[0], 1) != 3) {
			errorHandler.setInfo(F("!03,APR 1 comm error errorCountert:%d\r\n"), i2cAPR.i2cErrorcounter);
			state = 0;
			interval = 10;
			return;
		}

		RxValues.result = RxBuf[0];
		RxValues.potL = RxBuf[1];
		RxValues.potR = RxBuf[2];

		DecodeResults(RxValues.result);
		CaluculateInsideOutsideL();
		CaluculateInsideOutsideR();


		if (showValuesOnConsole) {
			state = 3; // go to state 3 to read and show extended values
			interval = 1;
			//state = 0;
			//interval = 10;
		}
		else {
			state = 0;
			interval = 10;
		}


		/* test for coil switch
		testcounter++;

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
	case 3:
		if (i2cAPR.read8Only(10, &RxBuf[0], 1) != 10) {
			errorHandler.setInfo(F("!03,APR 3 comm error errorCountert:%d\r\n"), i2cAPR.i2cErrorcounter);
			state = 0;
			interval = 10;
			return;
		}

		//RxValues.result = RxBuf[0];
		//RxValues.potL = RxBuf[1];
		//RxValues.potR = RxBuf[2];
		
		convert.uBytes[0] = RxBuf[3];
		convert.uBytes[1] = RxBuf[4];
		RxValues.L = convert.sIn16t;

		convert.uBytes[0] = RxBuf[5];
		convert.uBytes[1] = RxBuf[6];
		RxValues.R = convert.sIn16t;

		RxValues.ratioL = RxBuf[7];
		RxValues.ratioR = RxBuf[8];
		RxValues.resetCnt = RxBuf[9];

		errorHandler.setInfo(F("Decode result: %d iL: %d iR: %d vL: %d vR: %d back: %d counter: %d\r\n"),
			RxShortResults.result, RxShortResults.insideL, RxShortResults.insideR, RxShortResults.validL, RxShortResults.validR,
			RxShortResults.backCoilActive, RxShortResults.packetCounter);

		errorHandler.setInfo(F("!03,aL: %d/%d aR: %d/%d  potL: %u potR: %u ratioL: %u ratioR: %u result: %u I2Creset: %u\r\n"),
			RxValues.L, signalCounterL, RxValues.R, signalCounterR,
			(unsigned int)RxValues.potL, (unsigned int)RxValues.potR,
			(unsigned int)RxValues.ratioL, (unsigned int)RxValues.ratioR,
			(unsigned int)RxValues.result, (unsigned int)RxValues.resetCnt);

		state = 0;
		interval = 10;
		break;
	default:
		state = 0;
		interval = 10;
		state = 0;
		break;
	}
}

// ------------------------------------------------------------------------------------------
// Folgende Funktionen werden von bMow aufgerufen um Max des Perimeters zu bestimmen
// ------------------------------------------------------------------------------------------
bool TPerimeterThread::isNearPerimeter() {
	if (RxValues.potL > 0 || RxValues.potR > 0) {
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
	return (signalCounterL <= 0);

}

bool TPerimeterThread::isRightOutside() {
	// use filtered value for increased reliability
	return (signalCounterR <= 0);
}







//###########################################################
#endif //#if CONF_USE_ADVANCED_PERIMETER_SERVICE ==  true



