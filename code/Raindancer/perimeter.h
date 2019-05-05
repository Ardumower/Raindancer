

/*
Robotic Lawn Mower
Copyright (c) 2017 by Kai WÃ¼rtz

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


#ifndef PERIMETER_H
#define PERIMETER_H
// EmpfÃ¤ngt die Perimeterdaten Ã¼ber die SerDueMot read Schnittstelle vom due.

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

//#include "BufferedSerial.h"
#include "Thread.h"
#include "helpers.h"
#include "PerimeterCoil.h"
#include "RunningMedian.h"
#include "config.h"


enum EPerSignal {
	SIGNAL_INSIDE = -1, // optional, -1 is the initial state of the fsm
	SIGNAL_NA = 0,
	SIGNAL_OUTSIDE = 1
};


//###########################################################
// INCLUDE FOR PERIMETER RECEIVER CALCULATED BY DUE
#if CONF_USE_ADVANCED_PERIMETER_SERVICE ==  false
//###########################################################

enum EPerReceiveState {
	SPR_OFF = -1, // optional, -1 is the initial state of the fsm
	SPR_WAIT_COILL_SAMPLE_COMPLETED,
	SPR_WAIT_COILR_SAMPLE_COMPLETED,
	SPR_COILL_CALCULATE,
	SPR_COILR_CALCULATE

};

class TPerimeterThread : public Thread, public FSM<EPerReceiveState> {
private:

	PerimeterCoil coilL;
	PerimeterCoil coilR;

	// Achtung, Werte sind positive fÃ¼r inside!!!
	int magnetudeL;  // nimmt nur beim start 0 an. danach wird immer der letzte wert gelatched, wenn signal verloren
	int magnetudeR;
	int magnetudeL0; // nimmt zusätzlich 0 an wenn signal ungültig. Wird für Linefollowing verwendet verwendet.

	int16_t signalCounterL;    // 2 outside >=  wert  >=-2 inside
	int16_t signalCounterR;

	virtual void UpdateState(EPerReceiveState t);

	// Value negative for inside
	int32_t _magnetudeL;
	int32_t _magnetudeR;
	int32_t curMaxL, curMaxR;
	FastRunningMedian<int32_t, 16, 0> medianMagL;
	FastRunningMedian<int32_t, 16, 0> medianMagR;

	// Werte hier positiv fÃ¼r inside
	void CaluculateInsideOutsideL(int32_t magl);
	void CaluculateInsideOutsideR(int32_t magr);

public:

	void showADCWithoutOffset(uint8_t coil, bool bValue) {
		if (coil == 'L') {
			coilL.showADCWithoutOffset = bValue;
		}
		else {
			coilR.showADCWithoutOffset = bValue;
		}
	}

	void showCorrelation(uint8_t coil, bool bValue) {
		if (coil == 'L') {
			coilL.showCorrelation = bValue;
		}
		else {
			coilR.showCorrelation = bValue;
		}
	}

	void showCorrelationSQ(uint8_t coil, bool bValue) {
		if (coil == 'L') {
			coilL.showCorrelationSQ = bValue;
		}
		else {
			coilR.showCorrelationSQ = bValue;
		}
	}

	void showPSNRFunction(uint8_t coil, bool bValue) {
		if (coil == 'L') {
			coilL.showPSNRFunction = bValue;
		}
		else {
			coilR.showPSNRFunction = bValue;
		}
	}

	void showValuesResults(uint8_t coil, bool bValue) {
		if (coil == 'L') {
			coilL.showValuesResults = bValue;
		}
		else {
			coilR.showValuesResults = bValue;
		}
	}

	void showMatchedFilter(uint8_t coil, bool bValue) {
		if (coil == 'L') {
			coilL.showMatchedFilter = bValue;
		}
		else {
			coilR.showMatchedFilter = bValue;
		}
	}

	int magnetudeR0;

	unsigned long lastTimeSignalReceivedL;
	unsigned long lastTimeSignalReceivedR;

	int magMax;


	bool showValuesOnConsole;

	bool isNearPerimeter();

	bool isLeftInside();
	bool isRightInside();
	//bool isRightInsideMag();
	bool isLeftOutside();
	bool isRightOutside();
	//bool isRightOutsideMag();

	void setup();
	virtual void run(void);
	void showConfig();
};



//###########################################################
#endif //#if CONF_USE_ADVANCED_PERIMETER_SERVICE ==  false



//###########################################################
// INCLUDE FOR PERIMETER RECEIVER CALCULATED BY DUE
#if CONF_USE_ADVANCED_PERIMETER_SERVICE ==  true
//###########################################################


#define I2C_RECEICEBUFFER 15 //10
#define I2C_TXBUFFERSIZE 1

class TPerimeterThread : public Thread {
private:


	// Achtung, Werte sind positive fÃ¼r inside!!!
	int magnetudeL0; // nimmt zusätzlich 0 an wenn signal ungültig. Wird für Linefollowing verwendet verwendet.

	int16_t signalCounterL;    // 2 outside >=  wert  >=-2 inside
	int16_t signalCounterR;


	// Werte hier positiv fÃ¼r inside
	void CaluculateInsideOutsideL(void);
	void CaluculateInsideOutsideR(void);

	void DecodeResults(uint8_t shortResult);

	

	uint8_t lastPacketCounter = 0;


	struct {
		uint8_t result;
		uint8_t potL;
		uint8_t potR;
		uint8_t ratioL;
		uint8_t ratioR;
		uint8_t resetCnt;
		int16_t L;
		int16_t R;;
	} RxValues;

	struct {
		uint8_t result;
		uint8_t insideL;
		uint8_t insideR;
		uint8_t validL;
		uint8_t validR;
		uint8_t backCoilActive;
		int16_t packetCounter;
	} RxShortResults;

	uint8_t RxBuf[I2C_RECEICEBUFFER];

	uint8_t TxBuffer[I2C_TXBUFFERSIZE];

	int state;
	int testcounter;

	

public:

	void showADCWithoutOffset(uint8_t coil, bool bValue) {
	}

	void showCorrelation(uint8_t coil, bool bValue) {
	}

	void showCorrelationSQ(uint8_t coil, bool bValue) {
	}

	void showPSNRFunction(uint8_t coil, bool bValue) {
	}

	void showValuesResults(uint8_t coil, bool bValue) {
	}

	void showMatchedFilter(uint8_t coil, bool bValue) {
	}

	int magnetudeR0;

	unsigned long lastTimeSignalReceivedL;
	unsigned long lastTimeSignalReceivedR;

	int magMax; // not used for APR. Only dummy to comply with the interface

	bool showValuesOnConsole;

	bool isNearPerimeter();

	bool isLeftInside();
	bool isRightInside();

	bool isLeftOutside();
	bool isRightOutside();

	void setup();
	virtual void run(void);
	void showConfig();
};


//###########################################################
#endif //#if CONF_USE_ADVANCED_PERIMETER_SERVICE ==  true



//###########################################################
//###########################################################
#endif // #ifndef PERIMETER_H