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

#ifndef _MOWMOTOR_h
#define _MOWMOTOR_h

#include <inttypes.h>
#include "Thread.h"
#include "helpers.h"
#include "Sabertooth.h"


enum EMowMotorState
{
	STMM_OFF = -1, // optional, -1 is the initial state of the fsm
	STMM_FORWARD,
	STMM_BACKWARD,
	STMM_STOP
};

class TMowClosedLoopControlThread : public Thread, public FSM<EMowMotorState>
{
private:

	virtual void BeginState(EMowMotorState t);
	virtual void UpdateState(EMowMotorState t);

	uint8_t motorNo;  // Motornumber welcher motor soll von sabertooh angesprochen werden
	bool directionForward;
	int8_t resetCount;
public:

	bool motorDisabled; // Wird von mowmotorSensor gesetzt wenn Strom zu hoch.
	bool uiMotorDisabled; // User stopped Motor 
	bool flagShowSpeed;

	float speedCurr;
	int motorMowAccel;
	float speedLimit;

	void forward();
	void backward(); 
	void stop();
	bool isStopped();
	bool isRunning();

	void setup(uint8_t motorNumber);
	// Called at the loop rate 100ms to run the state machine and therfore the motor
	virtual void run();

	void controlDirect(int speed);
	void showConfig();
};


#endif


