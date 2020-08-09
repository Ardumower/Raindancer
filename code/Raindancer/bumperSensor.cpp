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

#include "bumperSensor.h"
#include "motor.h"
#include "config.h"

// Motorensteuerung Interface. KEIN THREAD. Wird verwendet um clcX und pcX zu steuern.
extern TMotorInterface motor;

void   TbumperSensor::setup()
{
	errorHandler.setInfo(F("TbumperSensor::setup()\r\n"));
	_bumperLeftActivated = false;
	_bumperRightActivated = false;
	_bumperDuinoActivated = false;
	flagShowBumper = false;

};

bool TbumperSensor::isBumperActivated()
{
	return (_bumperLeftActivated || _bumperRightActivated || _bumperDuinoActivated);
}

// Never use the following functions to check if a bumper is activated. Therefore the function isBumperActivated() is created.
// isBumperActivatedLeft() isBumperActivatedRight only checks the left or right bumper but not the bumperduino 
// and is only used in TFreeBumper to set the variables bb.flagBumperActivatedLeft and bb.flagBumperActivatedRight
bool TbumperSensor::isBumperActivatedLeft()
{
	//errorHandler.setInfo(F("TbumperSensor::isBumperActivatedLeft(); %d\r\n"), _bumperLeftActivated);
	return (_bumperLeftActivated); 
}
bool TbumperSensor::isBumperActivatedRight()
{
	//errorHandler.setInfo(F("TbumperSensor::isBumperActivatedRight(); %d\r\n"), _bumperRightActivated);
	return (_bumperRightActivated); 
}

bool TbumperSensor::isBumperDuinoActivated() {
	//errorHandler.setInfo(F("TbumperSensor::isBumperDuinoActivated(); %d\r\n"), _bumperDuinoActivated);
	return (_bumperDuinoActivated);
}


// Handle the different possibilities for the bumper usage low active, high active, enabled
bool TbumperSensor::_checkBumperLeft()
{
	if (CONF_USE_LEFT_BUMPER) {
		return (CONF_LEFT_BUMPER_LOW_ACTIVE) ? (diBumperL == LOW) : (diBumperL == HIGH);
	}
	return false;
}

bool TbumperSensor::_checkBumperRight()
{
	if (CONF_USE_RIGHT_BUMPER) {
		return (CONF_RIGHT_BUMPER_LOW_ACTIVE) ? (diBumperR == LOW) : (diBumperR == HIGH);
	}
	return false;
}

void TbumperSensor::run()
{
	runned();

	// Orignal Ardumower Bumper
#if CONF_DISABLE_BUMPER_SERVICE == false
 //bber-----------------------------------------------------
	bool bl = _checkBumperLeft();
	bool br = _checkBumperRight();

	if ((bl == false) && _bumperLeftActivated) {
		if (flagShowBumper) {
			errorHandler.setInfo(F("!03,Bumper Left deactivated\r\n"));
		}
		_bumperLeftActivated = false;
		//errorHandler.setInfo("!bumperLeftActivated = false;\r\n");
	}

	if ((br == false) && _bumperRightActivated) {
		if (flagShowBumper) {
			errorHandler.setInfo(F("!03,Bumper Right deactivated\r\n"));
		}
		_bumperRightActivated = false;
		//errorHandler.setInfo("!_bumperRightActivated = false;\r\n");
	}

	if ((bl == true) && !_bumperLeftActivated) {
		if (flagShowBumper) {
			errorHandler.setInfo(F("!03,Bumper left activated\r\n"));
		}
		_bumperLeftActivated = true;
		//errorHandler.setInfo("_bumperLeftActivated = true;\r\n");
	}

	if ((br == true) && !_bumperRightActivated) {
		if (flagShowBumper) {
			errorHandler.setInfo(F("!03,Bumper right activated\r\n"));
		}
		_bumperRightActivated = true;
		//errorHandler.setInfo("_bumperRightActivated = true;\r\n");

	}

	//----------------------------------------------
#endif

#if CONF_DISABLE_BUMPERDUINO_SERVICE == false 
	//#pragma message ("CONF_DISABLE_BUMPERDUINO_SERVICE AKTIVE")


			// Low active
	if (diBumperSensor == HIGH && _bumperDuinoActivated) {
		if (flagShowBumper) {
			errorHandler.setInfo(F("!03,BumperDuino deactivated\r\n"));
		}
		_bumperDuinoActivated = false;
	}

	if (diBumperSensor == LOW && !_bumperDuinoActivated) {
		if (flagShowBumper) {
			errorHandler.setInfo(F("!03,BumperDuino activated\r\n"));
		}
		_bumperDuinoActivated = true;
		//motor.hardStop();
	}

	/*
	if (flagShowBumper) {
	errorHandler.setInfo("!03,Bumper %d\r\n", diBumperSensor.read());
	}
	*/
#endif
		}


void TbumperSensor::showConfig()
{
	errorHandler.setInfoNoLog(F("!03,Bumper Sensor Config\r\n"));
	errorHandler.setInfoNoLog(F("!03,enabled: %lu\r\n"), enabled);
	errorHandler.setInfoNoLog(F("!03,interval: %lu\r\n"), interval);
}



