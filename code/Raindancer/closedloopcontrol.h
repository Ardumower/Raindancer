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


#ifndef _CLOSEDLOOPCONTROL_h
#define _CLOSEDLOOPCONTROL_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif



#include "Thread.h"
#include "helpers.h"
#include "Sabertooth.h"
#include "hardware.h"
#include "errorhandler.h"


class TClosedLoopControlThread : public Thread
{
private:
	bool flagHardstopActive;        // if hardStop() is called, set to true that calculatePDFF() sets output direct to 0
	unsigned long lastTimeSpeedSet; // every time speed is set, this is updated. motorstall will then first be checked 1 second after this time
	int lastSetSpeedPerc;           // save last set speed to remember in setSpeed, stop, .... This is to prevent to set same speed again and therfore init lastTimeSpeedSet again.

	//speed calcualtion in calculateCurSpeed()
	unsigned long lastTimeEncoderRead;
	long lastEncoderTickCounter;
	int motorStallCounter;
	bool firsttimeCurSpeed;
	float current_speed_mph;
	void calculateCurSpeed();       // reads encodere and calcualtes current_speed and check for encoder stall


	// PDFF input/output parameters
	float setpointRPM, current_speedRPM, pdffOutputPWM; // Pid setpoint, input und oputput
	float ITermRPM;
	float errorRPM;

	// PDFF
	unsigned long lastTimePDFF;
	bool firsttimePDFF;
	void calculatePDFF();
	void resetPDFF();


	// Variables Functions for testing
	unsigned long lastTickCounterShowEnc;
	unsigned long lastTimeEncoderReadShowEnc;
	

public:

	uint8_t motorNo;  // Motornumber welcher motor soll von sabertooh angesprochen werden
	CRotaryEncoder *myEncoder;

	float kfr;                 // * (kfr) proportional of feed forward
	float kp;                  // * (P)roportional Tuning Parameter
	float ki;                  // * (I)ntegral Tuning Parameter

	float setOutputToZeroAtRPm;      //If sollSpeedRPM should be 0, this is the threshold  where output is set to zero to improve agility. Positive value!
	float stopReachedThresholdAtRpm; //If the motor is running at this threshold, it is asssumed, that the motor is stopped 
    float stopNearReachedThresholdAtRpm; //If the motor is running at this threshold, it is asssumed, that the motor is just before stopping. Used for line follower to get a smoother motion. 
	void setup(uint8_t motorNumber, CRotaryEncoder *enc);

	virtual void run();  // Called at the loop rate 20ms to run the state machine and therfore the motor

	//Speed commands
	void setSpeed(int  speed);  // -100% bis 100% Motorgeschwindigkeit festlegen und sofort losfahren
	void stop(); // motor stoppen
	void hardStop();
	bool isStopped();
	bool isNearStopped();
	
	void enableDefaultRamping();
	void enablePerTrackRamping();
	void enableFastStopRamping();

	void resetEncoderCounter();
	void controlDirect(int speed); // set direct speed through sabertooth -255 to 255

	void showConfig();

	float calculateSpeedPercToRpm(int  speedPercentage); // Calculate speed from % to RPM
	float calculateSpeedRpmToPwm(float speedRPM); // Calculate speed from RPM to PWM
	float calculateSpeedPercToPwm(float  speedPercentage); // Calculate speed from % to PWM

	float getCurrentSpeedInPerc(); // get % of current speed
	float getSetpointSpeedInPerc();
	float getCurrentSpeedInRPM() { return current_speedRPM; }; // get % of current speed



	// Variables Functions for testing
	//----------------
	bool flagShowSpeed; // show speed on debug interface
	bool flagShowSetpointCurrSpeed;
	bool flagShowEncoder; // show encoder on debug interface
	bool flagShowEnableRamping;
	bool flagControldirect;



};



#endif

