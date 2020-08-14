/*
Robotic Lawn Mower
Copyright (c) 2017 by Kai W�rtz

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



/**********************************************************************************************
Arduino PID Library - Version 1.1.1
by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com

This Library is licensed under a GPLv3 License

Changed for own needs kai Wuertz
**********************************************************************************************/

#include "pid.h"
#include <math.h>
#include "helpers.h"
#include "hardware.h"
#include "errorhandler.h"

/*Constructor (...)*********************************************************
The parameters specified here are those for for which we can't set up
reliable defaults, so we need to have the user set them.
***************************************************************************/
void PIDPOS::setup(float* Input, float* Output, float* Setpoint, float Kp, float Ki, float Kd) {

	myOutput = Output;
	myInput = Input;
	mySetpoint = Setpoint;
	inAuto = false;

	SetOutputLimits(-127, 127);               //default output limit corresponds to sabertooth
	SetTunings(Kp, Ki, Kd);


	Int_Improvement = false;
	First_Time = true;

	flagShowValues = false;
}



/* Compute() **********************************************************************
This, as they say, is where the magic happens.  this function should be called
every time "void loop()" executes. the function will decide for itself whether a new
pid Output needs to be computed.  returns true when the output is computed,
false when nothing has been done.
Duration on DUE for calculation 12ms

ACHTUNG abge�ndert. Muss in bestimmten Zeitabst�nden aufgerufen werden
**********************************************************************************/
bool PIDPOS::Compute() {
	if (!inAuto) return false;

	unsigned long now =  micros();
	unsigned long delta = now - lastTimeInMillis;
	float Ta = (float)(delta) / 1000000.0f;  // Ta in seconds

										   /*Compute all the working error variables*/
	float input = *myInput;
	float error = *mySetpoint - input;

	// First call initialize the variables for bumpless Transfer and return. If not using "return" in this if statement,
	// then  Ta has to be defined and set to the used intervall.
	if (First_Time) {  
		First_Time = false;
		ITerm = *myOutput;
		lastInput = *myInput;
		////lastError = error;
		lastTimeInMillis = now; // no Ta used. Get current time to calculate at return.
		if (ITerm > outMax) ITerm = outMax;
		else if (ITerm < outMin) ITerm = outMin;
		return false;
	}


	// --- calculate proportional value ---
	float PTerm = kp * error;

	// --- PID Int Improvement ---
	if (Int_Improvement) {
		if (sign0plus(error) != sign0plus(ITerm))  ITerm = 0.0f;  //sign0minus => 0 belongs to plus

	}

	// --- Calculate integrated value ---
	ITerm += (ki * error * Ta);

	if (ITerm > outMax) ITerm = outMax;
	else if (ITerm < outMin) ITerm = outMin;


	// --- calculate derivative value ---
	float DTerm =  ((input - lastInput) * kd) / Ta;
	////float DTerm = (error - lastError) * kd / Ta;


	/*Compute PID Output*/
	float output = PTerm + ITerm - DTerm;  // mind the minus sign for using input in DTerm!!!
	//float output = PTerm + ITerm + DTerm;

	if (output > outMax) output = outMax;
	else if (output < outMin) output = outMin;
	*myOutput = output;

	  	
     /*Remember some variables for next time*/
	lastInput = input;
	////lastError = error;
	lastTimeInMillis = now;

	//debug << "Duration in mikroSek: " << micros() - now << endl;

	if (flagShowValues) {
		errorHandler.setInfo(F("e: %f P: %f I: %f D: %f O: %f\r\n"), error, PTerm, ITerm, DTerm, output);
	}

	return true;


}


/* SetTunings(...)*************************************************************
This function allows the controller's dynamic performance to be adjusted.
it's called automatically from the constructor, but tunings can also
be adjusted on the fly during normal operation
******************************************************************************/
void PIDPOS::SetTunings(float Kp, float Ki, float Kd) {
	if (Kp < 0 || Ki < 0 || Kd < 0) return;
	kp = Kp;
	ki = Ki;
	kd = Kd;

}

void PIDPOS::SetKp(float Kp) {
	if (Kp < 0) return;
	kp = Kp;

}
void PIDPOS::SetKi(float Ki) {
	if (Ki < 0) return;
	ki = Ki;

}
void PIDPOS::SetKd(float Kd) {
	if (Kd < 0) return;
	kd = Kd;

}


/* SetOutputLimits(...)****************************************************
This function will be used far more often than SetInputLimits.  while
the input to the controller will generally be in the 0-1023 range (which is
the default already,)  the output will be a little different.  maybe they'll
be doing a time window and will need 0-8000 or something.  or maybe they'll
want to clamp it from 0-125.  who knows.  at any rate, that can all be done
here.
**************************************************************************/
void PIDPOS::SetOutputLimits(float Min, float Max) {
	if (Min >= Max) return;
	outMin = Min;
	outMax = Max;

	if (inAuto) {
		if (*myOutput > outMax) *myOutput = outMax;
		else if (*myOutput < outMin) *myOutput = outMin;

		if (ITerm > outMax) ITerm = outMax;
		else if (ITerm < outMin) ITerm = outMin;
	}
}

/* SetMode(...)****************************************************************
Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
when the transition from manual to auto occurs, the controller is
automatically initialized
******************************************************************************/
void PIDPOS::SetMode(int Mode) {
	bool newAuto = (Mode == AUTOMATIC);
	if (newAuto == !inAuto) {
		/*we just went from manual to auto*/
		Initialize();
	}
	inAuto = newAuto;
}

/* Initialize()****************************************************************
does all the things that need to happen to ensure a bumpless transfer
from manual to automatic mode.
******************************************************************************/
void PIDPOS::Initialize() {
	First_Time = true;
}


/* Status Functions*************************************************************
Just because you set the Kp=-1 doesn't mean it actually happened.  these
functions query the internal state of the PID.  they're here for display
purposes.  this are the functions the PID Front-end uses for example
******************************************************************************/
float PIDPOS::GetKp() {
	return  kp;
}
float PIDPOS::GetKi() {
	return  ki;
}
float PIDPOS::GetKd() {
	return  kd;
}

int PIDPOS::GetMode() {
	return  inAuto ? AUTOMATIC : MANUAL;
}

float PIDPOS::GetIterm() {
	return  ITerm;
}
