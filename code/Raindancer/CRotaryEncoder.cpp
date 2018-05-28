// https://developer.mbed.org/users/Raabinator/code/CRotaryEncoder/
// Changed for own needs 2017 Kai Würtz
// 

#include "CRotaryEncoder.h"
#include "errorhandler.h"




CRotaryEncoder::CRotaryEncoder(DigitalIn &_pinA) : m_pinA(_pinA)
{

	m_ticks = 0;
	m_abs_ticks = 0;
	m_isReversed = false;
	m_direction_backward = false;
}

CRotaryEncoder::~CRotaryEncoder()
{

}

long   CRotaryEncoder::getTickCounter()
{
	return m_ticks;
};

void CRotaryEncoder::resetTickCounter()
{
	m_ticks = 0;
};


unsigned long CRotaryEncoder::getAbsTicksCounter()
{
	return m_abs_ticks;
};


void CRotaryEncoder::resetAbsTicksCounter()
{
	m_abs_ticks = 0;
};



void CRotaryEncoder::isReversed()
{
	m_isReversed = true;
	//errorHandler.setInfo(F("Encoder reverse\r\n"));
}

void CRotaryEncoder::isNotReversed()
{
	m_isReversed = false;
	//errorHandler.setInfo(F("Encoder not reverse\r\n"));
}

void CRotaryEncoder::directionIsForward()
{
	m_direction_backward = false;
}

void CRotaryEncoder::directionIsBackward()
{
	m_direction_backward = true;
}


void CRotaryEncoder::rise(void)
{
	int b;

	if (m_isReversed) {
		b = m_direction_backward ? +1 : -1;
	}
	else {
		b = m_direction_backward ? -1 : +1;
	}

	m_ticks += b;
	m_abs_ticks++;

}

