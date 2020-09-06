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

#ifndef _MOTOR_h
#define _MOTOR_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "helpers.h"
#include "mowclosedloopcontrol.h"
#include "closedloopcontrol.h"
#include "positioncontrol.h"
#include "Protothread.h"



class TMotorInterface : public Protothread {
private:


	// Distance measurement for different functions

	long encCountsLDistTraveled;
	long encCountsRDistTraveled;
	unsigned long lastRunDistanceMeasurementTraveled;

	long encCountsLAreaX;
	long encCountsRAreaX;
	unsigned long lastRunDistanceMeasurementAreaX;

	bool coilOutMeasurementRunningL;
	bool coilOutMeasurementRunningR;
	long encCountsLCoilOut;
	long encCountsRCoilOut;
	unsigned long lastRunDistanceMeasurementCoilOut;
	unsigned long lastRunDistanceMeasurementCoilAngle;

	long encCountsLSpiral;
	unsigned long lastRunDistanceMeasurementSpiral;

	long encCountsLOverRun;
	long encCountsROverRun;
	unsigned long lastRunDistanceMeasurementOverRun;

	long encCountsLTriangle;
	long encCountsRTriangle;
	unsigned long lastRunDistanceMeasurementTriangle;

public:

	TClosedLoopControlThread* L;
	TClosedLoopControlThread* R;
	TMowClosedLoopControlThread* M;
	TPositionControl* pcL;
	TPositionControl* pcR;

	virtual bool Run();

	void stopAllMotors();

	// Mow motor
	void mowMotStart();
	void mowMotStop();
	bool isMowMotRunning();

	// drive motors
	void setSpeedCLC(long  speed); //-100% to +100%
	void changeSpeedPC(long  speed); //-100% to +100%
	void stopPC(); // stop drive motors with position control
	void stopPCAtPerimeter(); // stop drive motors with position control
	void stopCLC(); // stop drive motors direct with CLC
	void hardStop();
	bool isCLCStopped();

	void enableDefaultRamping();
	void enablePerTrackRamping();
	void enableFastStopRamping();


	// positioning routines
	void rotateAngle(float angle, long _speed);
	void rotateCM(float _cm, long _speed);
	void rotateCM(float _cmL, float _cmR, long _speedL, long _speedR);
	void turnTo(float angle, long _speed);
	//void stopPositioning(); // query isPositionReached() to determine if motors are stopped!!!
	bool isPositionReached();

	// Calculate Encodercounts to Meter
	long getCountsForM(float x);
	long getMForCounts(float x);


	void resetEncoderCounter();
	void startDistanceMeasurementCoilOut();
	void stopDistanceMeasurementLCoilOut();
	void stopDistanceMeasurementRCoilOut();
	float getDistanceDiffInCMForCoilOut();
	float getDistanceAngleCoilOut();

	void startDistanceMeasurementAreax();


	void startDistanceMeasurementSpiral();
	long getDistanceInMeterAreax();

	void startDistanceMeasurement();
	float getDistanceInCM();
	float getAngleRotatedDistanceCM();
	float getAngleRotatedAngleDeg();

	float getDistanceLInCMSpiral();

	void startDistanceMeasurementTriangle();
	float getDistanceInCMForTriangle();

	void startDistanceMeasurementOverRun();
	float getDistanceInCMForOverRun();


	long getEncoderTickCountsL();
	long getEncoderTickCountsR();

	bool flagShowDistance; // When GotoAreaX is activated, showing the distance can be switched on with this flag.

	void showConfig();

	void setup(TMowClosedLoopControlThread* _M, TClosedLoopControlThread* _LEFT, TClosedLoopControlThread* _R, TPositionControl* _pcL, TPositionControl* _pcR);


	// Variable Functions for testing
	//----------------
	unsigned long lastrunTest; // used for PID tune
	int stateTest;
	int speedMinTest, speedMaxTest; //speed for PID tune

	bool flagMotorStepSpeed; // accelerate and decelerate the motor form 80% to 60 %
	void motorStepSpeed();
	bool flagMotorFSB; // drive forward to 80% stop drive backward to -80% stop
	void testForwardStopBackward();

	bool flagMotorPFSB;
	void testPosForwardStopBackward();

	bool flagMotorPerOverrun;
	void motorPerOverrunTest();
};



#endif

