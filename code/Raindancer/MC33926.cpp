// MC33926 motor driver
// Check http://forum.pololu.com/viewtopic.php?f=15&t=5272#p25031 for explanations.
//(8-bit PWM=255, 10-bit PWM=1023)
// IN1 PinPWM         IN2 PinDir
// PWM                L     Forward
// nPWM               H     Reverse


#include "hardware.h"
#include  "errorhandler.h"


MC33926Wheels::MC33926Wheels(CRotaryEncoder& _encoder1, CRotaryEncoder& _encoder2): encoder1(_encoder1), encoder2(_encoder2) {
	m_power1 = 999;
	m_power2 = 999;
}

MC33926Wheels::~MC33926Wheels()
{
	
}

/*!
Sets the power of the specified motor.
\param motor The motor number, 1 or 2.
\param power The power, between -255 and 255.
*/
void MC33926Wheels::motor(byte motor, int power)
{

	//clamp to[-255, 255]
	power = power >  255 ? 255 : power;
	power = power < -255 ? -255 : power;

	if (motor == 1) {
		if (m_power1 == power) return;
		m_power1 = power;

		//power = (power * 51) / 80;

		if (power < 0) {
			doMotorLeftDir = HIGH;
			pwmMotorLeft.write(255 - ((byte)abs(power)));
			encoder1.directionIsBackward();
		}
		else {
			doMotorLeftDir = LOW;
			pwmMotorLeft.write((byte)power);
			encoder1.directionIsForward();
		}
	}
	else if (motor == 2) {
		if (m_power2 == power) return;
		m_power2 = power;

		//power = (power * 51) / 80;

		if (power < 0) {
			doMotorRightDir = HIGH;
			pwmMotorRight.write(255 - ((byte)abs(power)));
			encoder2.directionIsBackward();
		}
		else {
			doMotorRightDir = LOW;
			pwmMotorRight.write((byte)power);
			encoder2.directionIsForward();
		}

	}
}



void MC33926Wheels::resetFault(bool force) {
	if (diMotorLeftFault == LOW || force) {
		doMotorEnable = LOW;
		delay(1);
		doMotorEnable = HIGH;
		errorHandler.setInfo(F("Reset motor left fault\r\n"));
	}
	if (diMotorRightFault == LOW || force) {
		doMotorEnable = LOW;
		delay(1);
		doMotorEnable = HIGH;
		errorHandler.setInfo(F("Reset motor right fault\r\n"));
	}
}


MC33926Mow::MC33926Mow(){
	m_power = 999;
}

MC33926Mow::~MC33926Mow() {
}


void MC33926Mow::motor(byte motor, int power)
{

	//clamp to[-255, 255]
	power = power >  255 ? 255 : power;
	power = power < -255 ? -255 : power;


	if (m_power == power) return;
	m_power = power;

	//power = (power * 51) / 80;

	if (motor == 1) {
		if (power < 0) {
			doMotorMowDir = HIGH;
			pwmMotorMowPWM.write(255 - ((byte)abs(power)));
		}
		else {
			doMotorMowDir = LOW;
			pwmMotorMowPWM.write((byte)power);
		}
	}

}


void MC33926Mow::resetFault(bool force) {
	if (diMotorMowFault == LOW || force) {
		doMotorMowEnable = LOW;
		delay(1);
		doMotorMowEnable = HIGH;
		errorHandler.setInfo(F("Reset motor mow fault\r\n"));
	}
}

