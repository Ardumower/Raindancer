/*
Robotic Lawn Mower
Copyright (c) 2017 by Kai WÃ¼rtz



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

//#include <algorithm>   //for min/max
#include "perimeter.h"
#include "helpers.h"
#include "hardware.h"
#include "errorhandler.h"
#include "config.h"
#include "gps.h"

//GPS Service
extern Tgps srvGps;

void TPerimeterThread::setup() {

	magnetudeL = 0;       // maximum des korrelationssignals. Kann auch negative sein. Amplitude von kleiner +-40 ist rauschen
	magnetudeL0 = 0;
	lastTimeSignalReceivedL = 0;

	magnetudeR = 0;
	magnetudeR0 = 0;
	lastTimeSignalReceivedR = 0;
	magMax = 0;

	showValuesOnConsole = false;
	arcToPerimeter = 0;

	signalCounterL = 0;    // 2 outside >=  wert  >=-2 inside
	signalCounterR = 0;
	signalCounterLFast = 0;
	signalCounterRFast = 0;

	// Set ADC Sample Address
	//coilL.showValues = true;
	//coilL.showValuesResults = true;
	coilL.setup(aiCoilLeft.getSamples());
	coilR.setup(aiCoilRight.getSamples());


	//magLineFollowMax =  32100;
	//magLineFollow = 0;
	//magLineFollowMin = -32100;


	count = 0;

	SetState(SPR_WAIT_COILL_SAMPLE_COMPLETED);
}


void TPerimeterThread::CaluculateInsideOutsideL(int32_t magl) {
	int32_t average = 0;

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

		signalCounterLFast = min(signalCounterLFast + 1, 2);
		signalCounterL = min(signalCounterL + 1, 3);
		lastTimeSignalReceivedL = millis();

		// Determine Maximum amplitude
		medianMagL.addValue((int32_t)magnetudeL);
		curMaxL = medianMagL.getHighest(); // current max
		average = medianMagL.getAverage(8);
		if (average > magMax) {
			magMax = average;
		}

	}

	// ** outside mag negative**
	else if (magl < 0) {
		signalCounterLFast = max(signalCounterLFast - 1, -2);
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
		if (srvGps.flagInsidePolygon && abs(magnetudeL) < CONF_PER_THRESHOLD_IGNORE_GPS) // only check if amplitude is lower than threshold
		{
			signalCounterLFast = 2;
			signalCounterL = 3;
			lastTimeSignalReceivedL = millis();
		}
	}
}

void TPerimeterThread::CaluculateInsideOutsideR(int32_t magr) {
	//----------------------------------------
	// Evaluate right
	//----------------------------------------

	int32_t average = 0;

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

		signalCounterRFast = min(signalCounterRFast + 1, 2);
		signalCounterR = min(signalCounterR + 1, 3);
		lastTimeSignalReceivedR = millis();

		// Determine Maximum amplitude
		medianMagR.addValue((int32_t)magnetudeR);
		curMaxR = medianMagR.getHighest(); // current max
		average = medianMagR.getAverage(8);
		if (average > magMax) {
			magMax = average;
		}

	}
	else if (magr < 0) {
		signalCounterRFast = max(signalCounterRFast - 1, -2);
		signalCounterR = max(signalCounterR - 1, -3);
		lastTimeSignalReceivedR = millis();

	}
	else {
		//signalCounterR = max(signalCounterR - 1, -3);
	}

	// Overwrite values when inside GPS polygon
	if (CONF_USE_GPS_POLYGON) // Check if the srvGps signal shows, that robot is inside the defined gps polygon.
	{
		if (srvGps.flagInsidePolygon && abs(magnetudeR) < CONF_PER_THRESHOLD_IGNORE_GPS) // only check if amplitude is lower than threshold
		{
			signalCounterRFast = 2;
			signalCounterR = 3;
			lastTimeSignalReceivedR = millis();
		}
	}

}


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
			//errorHandler.setInfo(F("CompL %lu\r\n"), millis());

		}
		break;

	case SPR_WAIT_COILR_SAMPLE_COMPLETED:
		if (aiCoilRight.isConvComplete()) {
			//errorHandler.setInfo(F("ResL %lu\r\n"), millis());
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
				sprintf(errorHandler.msg, "!03,ML: %d/%d/%d MR: %d/%d/%d magMax:%d magMedL%%: %d magMedR%%: %d\r\n", magnetudeL, signalCounterL, signalCounterLFast, magnetudeR, signalCounterR, signalCounterRFast, magMax, (int)curMaxL * 100 / magMax, (int)curMaxR * 100 / magMax);
				errorHandler.setInfo();
			}
		}

		break;

	default:
		//TODO invalid state - reset, perhaps?
		break;
	}
}


void TPerimeterThread::showConfig() {
	errorHandler.setInfo(F("!03,Perimeter Sensor Config\r\n"));
	errorHandler.setInfo(F("!03,enabled: %d\r\n"), IsRunning());
	errorHandler.setInfo(F("!03,interval: %lu\r\n"), interval);
}

bool TPerimeterThread::Run() {

	if (CONF_DISABLE_PERIMETER_SERVICE) {
		magnetudeL = 1000;
		magnetudeL0 = 1000;
		magnetudeR = 1000;
		magnetudeR0 = 1000;
		return true;
	}
	LoopFSM();
	return true;
}


// ------------------------------------------------------------------------------------------
// Folgende Funktionen werden von bMow aufgerufen um Max des Perimeters zu bestimmen
// ------------------------------------------------------------------------------------------
bool TPerimeterThread::isNearPerimeter() {
	//return false;
	if (magMax == 0) { //if we have not measured a magnitude always return near in order to drive low speed then
		return true;
	}

	long thresholdUpper = (magMax * CONF_NEAR_PER_UPPER_THRESHOLD) / 100L; ; //(magMax * 80L) / 100L; //95% vom Maximalwert ist untere schwelle fuer bestimmung ob nah am perimeter wire
	long thresholdLower = (magMax * CONF_NEAR_PER_LOWER_THRESHOLD) / 100L; ; //(magMax * 80L) / 100L; //95% vom Maximalwert ist untere schwelle fuer bestimmung ob nah am perimeter wire



	if (curMaxL >= thresholdUpper && curMaxR >= thresholdLower) {
		//sprintf(errorHandler.msg, "!03,NearPerimeter magMedL: %d magMedR: %d\r\n", (int)curMaxL, (int)curMaxR);
		//errorHandler.setInfo();
		return true;
	}

	if (curMaxL >= thresholdLower && curMaxR >= thresholdUpper) {
		//sprintf(errorHandler.msg, "!03,NearPerimeter magMedL: %d magMedR: %d\r\n", (int)curMaxL, (int)curMaxR);
		//errorHandler.setInfo();
		return true;
	}

	if (signalCounterL <= 0 || signalCounterR <= 0) {
		return true;
	}

	return false;
}


// ------------------------------------------------------------------------------------------



bool TPerimeterThread::isLeftInside() {

	if (abs(magnetudeL) > CONF_PER_USE_COUNTER_THRESHOLD) {
		// Large signal, the in/out detection is reliable.
		// Using mag yields very fast in/out transition reporting.
		//return (magnetudeL > 0);
		return (signalCounterLFast > 0);
	}
	else {
		// Low signal, use filtered value for increased reliability
		return (signalCounterL > 0);
	}
}

/*
bool TPerimeterThread::isRightInsideMag()
{
    if (magnetudeR > 0)
	  return true;
    return false;
}
*/

bool TPerimeterThread::isRightInside() {

	if (abs(magnetudeR) > CONF_PER_USE_COUNTER_THRESHOLD) {
		// Large signal, the in/out detection is reliable.
		// Using mag yields very fast in/out transition reporting.
		//return (magnetudeR > 0);
		return (signalCounterRFast > 0);
	}
	else {
		// Low signal, use filtered value for increased reliability
		return (signalCounterR > 0);
	}
}


bool TPerimeterThread::isLeftOutside() {

	if (abs(magnetudeL) > CONF_PER_USE_COUNTER_THRESHOLD) {
		// Large signal, the in/out detection is reliable.
		// Using mag yields very fast in/out transition reporting.
		//return (magnetudeL <= 0);
		return (signalCounterLFast <= 0);
	}
	else {
		// Low signal, use filtered value for increased reliability
		return (signalCounterL <= 0);
	}

}

bool TPerimeterThread::isRightOutside() {

	if (abs(magnetudeR) > CONF_PER_USE_COUNTER_THRESHOLD) {
		// Large signal, the in/out detection is reliable.
		// Using mag yields very fast in/out transition reporting.
		//return (magnetudeR <= 0);
		return (signalCounterRFast <= 0);
	}
	else {
		// Low signal, use filtered value for increased reliability
		return (signalCounterR <= 0);
	}
}

/*
bool TPerimeterThread::isRightOutsideMag()
{
    if (magnetudeR <= 0)
	  return true;
    return false;
}
*/



