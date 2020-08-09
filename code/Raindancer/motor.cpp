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

#include "motor.h"
#include "hardware.h"
#include "errorhandler.h"
#include "config.h"

// perimeter only used for motorPerOverrunTest
#include "perimeter.h"
extern TPerimeterThread srvPerSensoren;


//#define ENCODER_TEST 1   // use this to verify that your encoders are working


/*********************************************************************************************
Klassenfunktionen.
********************************************************************************************/

void TMotorInterface::setup(TMowClosedLoopControlThread* _M, TClosedLoopControlThread* _LEFT, TClosedLoopControlThread* _R, TPositionControl* _pcL, TPositionControl* _pcR) {
	L = _LEFT;
	R = _R;
	pcL = _pcL;
	pcR = _pcR;
	M = _M;

	flagShowDistance = false;
	flagMotorStepSpeed = false;
	flagMotorFSB = false;
	flagMotorPFSB = false;
	flagMotorPerOverrun = false;
	lastrunTest = 0;
	stateTest = 0;

}


void TMotorInterface::run() {
	// Wird alle 100ms aufgerufen
	runned();
	/*
	pcL->run();
	L->run();
	pcR->run();
	R->run();
	*/


	// Test functions start
	if (flagMotorStepSpeed) {
		motorStepSpeed();
	}
	else if (flagMotorFSB) {
		testForwardStopBackward();
	}
	else if (flagMotorPFSB) {
		testPosForwardStopBackward();
	}
	else if (flagMotorPerOverrun) {
		motorPerOverrunTest();
	}
}


void TMotorInterface::stopAllMotors() {
	//stopPositioning();
	stopCLC();
	mowMotStop();
}

void TMotorInterface::hardStop() {
	stopCLC();
	/*
	L->hardStop();
	R->hardStop();
	pcL->reset();
	pcR->reset();
	*/
}

void TMotorInterface::stopCLC() {
	L->stop();
	R->stop();
	pcL->reset();
	pcR->reset();
}


void TMotorInterface::stopPC() {
	pcL->stop();
	pcR->stop();
}


void TMotorInterface::stopPCAtPerimeter() {
	pcL->stopAtPerimeter();
	pcR->stopAtPerimeter();
}

void TMotorInterface::mowMotStart() {
	M->forward();
}

void TMotorInterface::mowMotStop() {
	M->stop();
}

bool TMotorInterface::isMowMotRunning() {
	return M->isRunning();
}


void TMotorInterface::setSpeedCLC(long  speed) {
	L->setSpeed(speed);
	R->setSpeed(speed);
}

void TMotorInterface::changeSpeedPC(long  speed) //-100% to +100%
{
	pcL->changeSpeed(speed);
	pcR->changeSpeed(speed);
}

void TMotorInterface::enableDefaultRamping() {
	L->enableDefaultRamping();
	R->enableDefaultRamping();
}



void TMotorInterface::enablePerTrackRamping() {
	L->enablePerTrackRamping();
	R->enablePerTrackRamping();
}

void TMotorInterface::enableFastStopRamping() {
	L->enableFastStopRamping();
	R->enableFastStopRamping();
}

/*
void TMotorInterface::stopPositioning(){
pcL->stopPositioning();
pcR->stopPositioning();
}
*/

void TMotorInterface::rotateAngle(float _angle, long _speed) {
	pcL->rotateAngle(_angle, _speed);
	pcR->rotateAngle(_angle, _speed);
}

void TMotorInterface::rotateCM(float _cm, long _speed) {
	pcL->rotateCM(_cm, _speed);
	pcR->rotateCM(_cm, _speed);
}

void TMotorInterface::rotateCM(float _cmL, float _cmR, long _speedL, long _speedR) {
	pcL->rotateCM(_cmL, _speedL);
	pcR->rotateCM(_cmR, _speedR);
}

void TMotorInterface::turnTo(float _angle, long _speed) {
	//float weg = (_angle * PI * (GETTF(TF_DISTANCE_BETWEEN_WHEELS_CM)/2) ) /180;
	float weg = (_angle * PI * CONF_DISTANCE_BETWEEN_WHEELS_CM) / 360.0f;

	pcL->rotateCM(weg, _speed);
	pcR->rotateCM(-1.0f * weg, _speed);

}

bool  TMotorInterface::isPositionReached() {
	if (pcL->isPositionReached() && pcR->isPositionReached()) {
		return true;
	}
	return false;
}

bool  TMotorInterface::isCLCStopped() {
	if (L->isStopped() && R->isStopped()) {
		return true;
	}
	return false;
}

long TMotorInterface::getCountsForM(float x) {
	return pcL->getCountsForCM(x * 100);
}


long TMotorInterface::getMForCounts(float x) {
	return pcL->getCMForCounts(x) / 100;
}


void TMotorInterface::resetEncoderCounter() {
	L->resetEncoderCounter();
	R->resetEncoderCounter();;
}


long TMotorInterface::getEncoderTickCountsL() {
	return L->myEncoder->getTickCounter();
}
long TMotorInterface::getEncoderTickCountsR() {
	return R->myEncoder->getTickCounter();
}

long encCountsLCoilOut;
long encCountsRCoilOut;
unsigned long lastRunDistanceMeasurementCoilOut;

void TMotorInterface::startDistanceMeasurementCoilOut() {
	//errorHandler.setInfo(F("startDistanceMeasurementCoilOut\r\n"));
	//if (b) { // Beide coils outside
	encCountsLCoilOut = L->myEncoder->getTickCounter();
	encCountsRCoilOut = R->myEncoder->getTickCounter();
	lastRunDistanceMeasurementCoilOut = millis();
	coilOutMeasurementRunningL = true;
	coilOutMeasurementRunningR = true;
	/*	}

else { // Wenn nur eine coil outside, hohe distanz vorgeben
	encCountsLCoilOut = pcL->getCountsForCM(100);
	encCountsRCoilOut = 0;
	lastRunDistanceMeasurementCoilOut = millis();
	coilOutMeasurementRunningL = false;
	coilOutMeasurementRunningR = false;
}
*/
}




void TMotorInterface::stopDistanceMeasurementLCoilOut() {
	if (coilOutMeasurementRunningL) {
		//errorHandler.setInfo(F("STOP L\r\n"));
		encCountsLCoilOut = L->myEncoder->getTickCounter() - encCountsLCoilOut;
		coilOutMeasurementRunningL = false;
	}
}

void TMotorInterface::stopDistanceMeasurementRCoilOut() {
	if (coilOutMeasurementRunningR) {
		//errorHandler.setInfo(F("STOP R\r\n"));
		encCountsRCoilOut = R->myEncoder->getTickCounter() - encCountsRCoilOut;
		coilOutMeasurementRunningR = false;
	}
}


float TMotorInterface::getDistanceDiffInCMForCoilOut() {
	if (coilOutMeasurementRunningL || coilOutMeasurementRunningR) { //Eine Spule nicht drinnen gewesen
		if (flagShowDistance) {
			if (millis() - lastRunDistanceMeasurementCoilOut > 100) {
				lastRunDistanceMeasurementCoilOut = millis();
				sprintf(errorHandler.msg, "!03,DistCoilOutDiff=100 Coil out L:%d R:%d\r\n", coilOutMeasurementRunningL, coilOutMeasurementRunningR);
				errorHandler.setInfo();
			}
		}
		return 100.0f;
	}

	long encCounts = encCountsLCoilOut - encCountsRCoilOut;
	encCounts = abs(encCounts);

	float difference = pcL->getCMForCounts(encCounts);
	//difference = abs(difference);

	if (flagShowDistance) {
		if (millis() - lastRunDistanceMeasurementCoilOut > 100) {
			lastRunDistanceMeasurementCoilOut = millis();
			sprintf(errorHandler.msg, "!03,DistCoilOutDiff CM: %f encCountsDiff %ld\r\n", difference, encCounts);
			errorHandler.setInfo();
		}
	}

	return difference;
}


float TMotorInterface::getDistanceAngleCoilOut() {
	if (coilOutMeasurementRunningL || coilOutMeasurementRunningR) {
		if (flagShowDistance) {
			if (millis() - lastRunDistanceMeasurementCoilOut > 100) {
				lastRunDistanceMeasurementCoilOut = millis();
				sprintf(errorHandler.msg, "!03,DistCoilOutAngle=90 Coil out L:%d R:%d\r\n", coilOutMeasurementRunningL, coilOutMeasurementRunningR);
				errorHandler.setInfo();
			}
		}
		return 90.0f;
	}

	long encCounts = encCountsLCoilOut - encCountsRCoilOut;
	encCounts = abs(encCounts);

	float difference = pcL->getCMForCounts(encCounts);
	//difference = abs(difference);

	float angle = atan(difference / CONF_DISTANCE_BETWEEN_COILS_CM) * 180 / PI; // 14.0cm Abstand der Spulen

	if (flagShowDistance) {
		if (millis() - lastRunDistanceMeasurementCoilAngle > 100) {
			lastRunDistanceMeasurementCoilAngle = millis();
			sprintf(errorHandler.msg, "!03,DistCoilOutAngle: %f encCountsDiff %ld\r\n", angle, encCounts);
			errorHandler.setInfo();
		}
	}

	return angle;
}


void TMotorInterface::startDistanceMeasurement() {
	encCountsLDistTraveled = L->myEncoder->getTickCounter();
	encCountsRDistTraveled = R->myEncoder->getTickCounter();
	lastRunDistanceMeasurementTraveled = millis();
}

float TMotorInterface::getDistanceInCM() {
	long encCountsL = L->myEncoder->getTickCounter() - encCountsLDistTraveled;
	long encCountsR = R->myEncoder->getTickCounter() - encCountsRDistTraveled;

	//if (encCountsL <0) encCountsL = 0;
	//if (encCountsR <0) encCountsR = 0;

	long encCounts = encCountsL + encCountsR;
	encCounts /= 2;

	float drivenDistance = pcL->getCMForCounts(encCounts);
	//drivenDistance = abs(drivenDistance);

	if (flagShowDistance) {
		if (millis() - lastRunDistanceMeasurementTraveled > 100) {
			lastRunDistanceMeasurementTraveled = millis();
			sprintf(errorHandler.msg, "!03,DrivenDistance CM: %f encCounts %ld\r\n", drivenDistance, encCounts);
			errorHandler.setInfo();
		}
	}

	return drivenDistance;
}


float TMotorInterface::getAngleRotatedDistanceCM() {
	long encCountsL = L->myEncoder->getTickCounter() - encCountsLDistTraveled;
	long encCountsR = R->myEncoder->getTickCounter() - encCountsRDistTraveled;

	long encCounts = abs(encCountsL) + abs(encCountsR);
	encCounts /= 2;

	float drivenDistance = pcL->getCMForCounts(encCounts);
	//drivenDistance = abs(drivenDistance);

	if (flagShowDistance) {
		if (millis() - lastRunDistanceMeasurementTraveled > 100) {
			lastRunDistanceMeasurementTraveled = millis();
			float _angle = (drivenDistance * 360.f) / (PI * CONF_DISTANCE_BETWEEN_WHEELS_CM);
			sprintf(errorHandler.msg, "!03,Rotated Angle angle: %f CM: %f encCounts: %ld \r\n", _angle, drivenDistance, encCounts);
			errorHandler.setInfo();
		}
	}

	return drivenDistance;
}

float TMotorInterface::getAngleRotatedAngleDeg() {
	long encCountsL = L->myEncoder->getTickCounter() - encCountsLDistTraveled;
	long encCountsR = R->myEncoder->getTickCounter() - encCountsRDistTraveled;

	long encCounts = abs(encCountsL) + abs(encCountsR);
	encCounts /= 2;

	float drivenDistance = pcL->getCMForCounts(encCounts);
	//drivenDistance = abs(drivenDistance);
	float _angle = (drivenDistance * 360.f) / (PI * CONF_DISTANCE_BETWEEN_WHEELS_CM);

	// set negative if direction was CC
	if (encCountsL < 0) {
		_angle = -_angle;

	}

	if (flagShowDistance) {
		if (millis() - lastRunDistanceMeasurementTraveled > 100) {
			lastRunDistanceMeasurementTraveled = millis();

			sprintf(errorHandler.msg, "!03,Rotated Angle angle: %f CM: %f encCounts: %ld \r\n", _angle, drivenDistance, encCounts);
			errorHandler.setInfo();
		}
	}

	return _angle;
}


void TMotorInterface::startDistanceMeasurementSpiral() {
	encCountsLSpiral = L->myEncoder->getTickCounter();
	lastRunDistanceMeasurementSpiral = millis();
}


float TMotorInterface::getDistanceLInCMSpiral() {
	long encCounts = L->myEncoder->getTickCounter() - encCountsLSpiral;;

	encCounts = abs(encCounts);

	float drivenDistance = pcL->getCMForCounts(encCounts);
	drivenDistance = abs(drivenDistance);

	if (flagShowDistance) {
		if (millis() - lastRunDistanceMeasurementSpiral > 100) {
			lastRunDistanceMeasurementSpiral = millis();
			sprintf(errorHandler.msg, "!03,DistSpiralL CM: %f encCounts %ld\r\n", drivenDistance, encCounts);
			errorHandler.setInfo();
		}
	}

	return drivenDistance;
}



void TMotorInterface::startDistanceMeasurementAreax() {
	encCountsLAreaX = L->myEncoder->getTickCounter();
	encCountsRAreaX = R->myEncoder->getTickCounter();
	lastRunDistanceMeasurementAreaX = millis();
}

long TMotorInterface::getDistanceInMeterAreax() {
	long encCountsL = L->myEncoder->getTickCounter() - encCountsLAreaX;
	long encCountsR = R->myEncoder->getTickCounter() - encCountsRAreaX;

	encCountsL = abs(encCountsL);
	encCountsR = abs(encCountsR);


	long encCounts = encCountsL + encCountsR;
	encCounts /= 2;

	long drivenDistance = getMForCounts(encCounts);
	drivenDistance = abs(drivenDistance);

	if (flagShowDistance) {
		if (millis() - lastRunDistanceMeasurementAreaX > 500) {
			lastRunDistanceMeasurementAreaX = millis();
			sprintf(errorHandler.msg, "!03,DistAreax M: %ld encCounts %ld\r\n", drivenDistance, encCounts);
			errorHandler.setInfo();
		}
	}


	return drivenDistance;
}


void TMotorInterface::startDistanceMeasurementOverRun() {
	encCountsLOverRun = L->myEncoder->getTickCounter();
	encCountsROverRun = R->myEncoder->getTickCounter();
	lastRunDistanceMeasurementOverRun = millis();
}

float TMotorInterface::getDistanceInCMForOverRun() {
	long encCountsL = L->myEncoder->getTickCounter() - encCountsLOverRun;
	long encCountsR = R->myEncoder->getTickCounter() - encCountsROverRun;

	long encCounts = encCountsL + encCountsR;
	encCounts /= 2;

	float drivenDistance = pcL->getCMForCounts(encCounts);
	drivenDistance = abs(drivenDistance);

	if (flagShowDistance) {
		if (millis() - lastRunDistanceMeasurementOverRun > 100) {
			lastRunDistanceMeasurementOverRun = millis();
			sprintf(errorHandler.msg, "!03,DistOverRun CM: %f encCounts %ld\r\n", drivenDistance, encCounts);
			errorHandler.setInfo();
		}
	}

	return drivenDistance;
}



void TMotorInterface::startDistanceMeasurementTriangle() {
	encCountsLTriangle = L->myEncoder->getTickCounter();
	encCountsRTriangle = R->myEncoder->getTickCounter();
	lastRunDistanceMeasurementTriangle = millis();
}

float TMotorInterface::getDistanceInCMForTriangle() {
	long encCountsL = L->myEncoder->getTickCounter() - encCountsLTriangle;
	long encCountsR = R->myEncoder->getTickCounter() - encCountsRTriangle;

	long encCounts = encCountsL + encCountsR;
	encCounts /= 2;

	float drivenDistance = pcL->getCMForCounts(encCounts);

	if (flagShowDistance) {
		if (millis() - lastRunDistanceMeasurementOverRun > 100) {
			lastRunDistanceMeasurementOverRun = millis();
			sprintf(errorHandler.msg, "!03,DistOverRun CM: %f encCounts %ld\r\n", drivenDistance, encCounts);
			errorHandler.setInfo();
		}
	}

	return drivenDistance;
}


// Used for tuning the PID values
// Changes the speed between speedMinTest and speedMaxTest every 4 sec.
void TMotorInterface::motorStepSpeed() {

	if (stateTest == 0) {
		// Do nothing
	}
	else if (stateTest == 1) { // Start Test
		stateTest = 3;
		lastrunTest = millis() - 4001ul; // Next loop don't wait for 4000ms. Go imediatly to state 2
	}
	else 	if (stateTest == 99) { // Deactivate Test
		setSpeedCLC(0);
		L->hardStop();
		R->hardStop();
		stateTest = 0;
		flagMotorStepSpeed = false;
	}


	if ((millis() - lastrunTest) > 6000) {
		lastrunTest = millis();

		if (stateTest == 2) {
			setSpeedCLC(speedMinTest);
			stateTest = 3;
		}
		else if (stateTest == 3) {
			setSpeedCLC(speedMaxTest);
			stateTest = 2;
		}
	}
}

// Used for tuning the rampAccRPM and values
//Not used anymore
void TMotorInterface::testForwardStopBackward() {
	if (stateTest == 0) {
		// Do nothing
	}
	else if (stateTest == 1) { // Start Test
		stateTest = 5;
		lastrunTest = millis() - 4001ul; // Next loop don't wait for 4000ms. Go imediatly to state 2
	}
	else if (stateTest == 99) { // Deactivate Test
		setSpeedCLC(0);
		L->hardStop();
		R->hardStop();
		stateTest = 0;
		flagMotorFSB = false;
	}



	unsigned long now = millis();


	switch (stateTest) {
	case 3:
		//forward();
		if ((now - lastrunTest) > 4000) {
			stateTest = 4;
			stopCLC();
			errorHandler.setInfoNoLog(F("STOP\r\n"));
		}

		break;

	case 4:
		if (isCLCStopped()) {
			lastrunTest = now;
			setSpeedCLC(speedMinTest);
			stateTest = 5;
			errorHandler.setInfoNoLog(F("MIN\r\n"));
		}

		break;

	case 5:
		if ((now - lastrunTest) > 4000) {
			stateTest = 6;
			stopCLC();
			errorHandler.setInfoNoLog(F("STOP\r\n"));
		}
		break;

	case 6:
		if (isCLCStopped()) {
			lastrunTest = now;
			setSpeedCLC(speedMaxTest);
			stateTest = 3;
			errorHandler.setInfoNoLog(F("MAX\r\n"));
		}
		break;

	default:
		break;
	}

}



// Fährt vor und zurück nach Position. Test von TPositionControl
void TMotorInterface::testPosForwardStopBackward() {
	if (stateTest == 0) {
		// Do nothing
	}
	else if (stateTest == 1) { // Start Test
		stateTest = 3;
	}
	else if (stateTest == 99) { // Deactivate Test
		stopCLC();
		stateTest = 0;
		flagMotorPFSB = false;
	}

	switch (stateTest) {
	case 2:
		//errorHandler.setInfo(F("1\r"));
		if (isPositionReached()) {
			rotateAngle(-speedMinTest, speedMaxTest);
			stateTest = 3;
		}
		break;
	case 3:
		//errorHandler.setInfo(F("2\r"));
		if (isPositionReached()) {
			rotateAngle(speedMinTest, speedMaxTest);
			stateTest = 2;
		}
		break;

	default:
		break;
	}

}



void TMotorInterface::motorPerOverrunTest() {
	if (stateTest == 0) {
		// Do nothing
	}
	else if (stateTest == 1) { // Start Test
		stateTest = 2;
	}
	else if (stateTest == 99) { // Deactivate Test
		enableDefaultRamping();
		stopPC();
		stateTest = 0;
		flagMotorPerOverrun = false;
	}

	switch (stateTest) {
	case 2:
		setSpeedCLC(speedMinTest);
		stateTest = 3;
		break;
	case 3:
		if (srvPerSensoren.isLeftOutside() || srvPerSensoren.isRightOutside()) {
			stateTest = 99;
		}
		break;

	default:
		break;
	}


}
void TMotorInterface::showConfig() {
	errorHandler.setInfoNoLog(F("!03,enabled: %lu\r\n"), enabled);
	errorHandler.setInfoNoLog(F("!03,interval: %lu\r\n"), interval);
	errorHandler.setInfoNoLog(F("!03,speedMinTest %f\r\n"), speedMinTest);
	errorHandler.setInfoNoLog(F("!03,speedMaxTest %f\r\n"), speedMaxTest);
}
