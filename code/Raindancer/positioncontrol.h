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

Code used from LinuxCNC Project and adapted to own needs
https://github.com/LinuxCNC/linuxcnc/blob/master/src/emc/motion/simple_tp.c#start-of-
*******************************************************************
* Description: simple_tp.c
*   A simple single axis trajectory planner.  See simple_tp.h for API.
*
* Author: jmkasunich
* License: GPL Version 2
* Created on:
* System: Linux
*
* Copyright (c) 2004 All rights reserved.

The update() function does all the work.  If 'enable' is true, it
computes a new value of 'curr_pos', which moves toward 'pos_cmd'
while obeying the 'max_vel' and 'max_accel' limits.  It also sets
'active' if movement is in progress, and clears it when motion
stops at the commanded position.  The command or either of the
limits can be changed at any time.  If 'enable' is false, it
ramps the velocity to zero, then clears 'active' and sets
'pos_cmd' to match 'curr_pos', to avoid motion the next time it
is enabled.  'period' is the period between calls, in seconds.

*******************************************************************
*/


#ifndef _POSITIONCONTROL_h
#define _POSITIONCONTROL_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif



#include "Protothread.h"
#include "closedloopcontrol.h"
#include "CRotaryEncoder.h"



class TPositionControl : public Protothread
{

public:

	virtual bool Run();

	void setup(TClosedLoopControlThread *_motor, CRotaryEncoder *enc);

	void rotateAngle(float _angle, long _speedPercentage);
	void rotateCM(float _distanceCm, long _speedPercentage);
	void changeSpeed(long _speedPercentage);
	void stop();
	void stopAtPerimeter();

	void reset();
	bool isPositionReached(); // rotateCM or rotateAngle finished?

	void showConfig();

	// Wegberechnungsroutinen
	static long getCountsForCM(float x);
	static long getCountsForDegree(float x);
	static float getDegreeForCounts(float x);
	static float getCMForCounts(float x);

	bool flagShowResults;
    float stopCmBeforeTarget;


private:

	uint8_t state;

	float ta;

	TClosedLoopControlThread *motor;
	CRotaryEncoder *myEncoder;

	float pos_cmd;		/* position command */
	float max_vel;		/* velocity limit */
	float max_acc;		/* acceleration limit */
	int   enable;		/* if zero, motion stops ASAP */
	float curr_pos;    	/* current position */
	float curr_vel;	    /* current velocity */
	int   active;		/* non-zero if motion in progress */
	bool  forward, backward; /* drive direction. prevents algorithm from driving back in other direction after overshooting tiny_dp */

};

#endif

