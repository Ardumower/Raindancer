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

#ifndef _GLOBSL_h
#define _GLOBSL_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

extern long myRandom(long min, long max, bool init = false);

extern long mapl(long x, long in_min, long in_max, long out_min, long out_max);
extern float mapf(float x, float in_min, float in_max, float out_min, float out_max);

#define BitTest (var, bit) ((var & (1 << bit)) != 0) // Returns true/false if bit is set
#define GetBit  (var, bit) ((var & (1 << bit)) != 0)  // Returns true/false if bit is set
#define SetBit  (var, bit) (var |= (1 << bit))
#define FlipBit (var, bit) (var ^= (1 << bit))
#define ClearBit(var, bit) (var &= ~(1 << bit))

//#define PI (3.141592653589793238463f)
//determine the sign of a value  -1/0/1
//template <typename T> int sign(T val) {
//  return (T(0) < val) - (val < T(0));
//}


// determin the sign of a value -1/1. zero belongs to plus. 
template <typename T> int sign0plus(T val) {
	if (val >= T(0)) return 1;
	return -1;
}

// determin the sign of a value -1/1. zero belongs to minus. 
template <typename T> int sign0minus(T val) {
	if (val > T(0)) return 1;
	return -1;
}

/*******************************************************/
// STATE MACHINE
/*******************************************************/
// (c) Francois Guibert, www.frozax.com (@Frozax)
// http://www.frozax.com/blog/2012/10/simple-useful-finite-state-machine-fsm-c/
//
// The states are represented by an enum (easy to debug)
// To run the FSM, just call SetState and LoopFSM()
// Get the current state using GetState()
// Many transitions are based on timing. Therefore, I implemented a GetTimeInCurState() method
// Perform your specific actions in methods BeginState, EndState and UpdateState
//
//#pragma once

template<typename T>
class FSM
{
public:
	FSM() : _start_time_of_cur_state(millis()), _cur_state(-1)
	{
	}

	virtual void BeginState(T state) {}
	virtual void UpdateState(T state) {}
	virtual void EndState(T state) {}

	// Set state of statemachine. 
	// Set reenterState = true if you want to enter the same current state again.
	//   Then Endstate and Beginstate of the current state will be called also!
	void SetState(T state, bool reenterState = false)
	{
		if (_cur_state == state && !reenterState) return; //Don't set same state again

		EndState((T)_cur_state);
		_cur_state = state;
		_start_time_of_cur_state = millis();
		BeginState((T)_cur_state);
	}

	void LoopFSM()
	{
		if (_cur_state != -1)
		{
			UpdateState((T)_cur_state);
		}
	}

	unsigned long GetTimeInCurState() {
		return millis() - _start_time_of_cur_state;
	}

	T GetState() {
		return (T)_cur_state;
	}

	void SetStartTimeOfCurrentState(unsigned long time) {
		_start_time_of_cur_state = time;
	}
private:
	unsigned long _start_time_of_cur_state;
	int _cur_state;
};

#endif


