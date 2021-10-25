// MC33926.h

#ifndef _MC33926_h
#define _MC33926_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "hardware.h"
#include "CRotaryEncoder.h"


class MC33926Wheels
{
public:

	/*!
	Sets the power of the specified motor.
	\param motor The motor number, 1 or 2.
	\param power The power, between -255 and 255.
	*/
	MC33926Wheels(CRotaryEncoder& _encoder1, CRotaryEncoder& _encoder2);
	~MC33926Wheels();

	void motor(byte motor, int power);
	void resetFault(bool force);

private:
	int m_power1;
	int m_power2;
	CRotaryEncoder& encoder1;
	CRotaryEncoder& encoder2;

};


class MC33926Mow
{
public:

	/*!
	Sets the power of the specified motor.
	\param motor The motor number, 1 or 2.
	\param power The power, between -255 and 255.
	*/
	MC33926Mow();
	~MC33926Mow();

	void motor(byte motor, int power);
	void resetFault(bool force);

private:
	int m_power;
  bool disc_direction_positive;
};
#endif


