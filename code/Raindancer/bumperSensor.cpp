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

#include "bumperSensor.h"
#include "motor.h"
#include "config.h"

// Motorensteuerung Interface. KEIN THREAD. Wird verwendet um clcX und pcX zu steuern.
extern TMotorInterface motor;

void   TbumperSensor::setup()
{

	_bumperActivated = false;
	_bumperDuinoActivated = false;
	flagShowBumper = false;

};

bool TbumperSensor::isBumperActivated()
{
	return (_bumperActivated || _bumperDuinoActivated);
}
//bber -----------------------------------------------
bool TbumperSensor::isBumperActivatedLeft()
{
  return (_bumperLeftActivated || _bumperDuinoActivated); // i leave the bumperduino but certainly not use
}
bool TbumperSensor::isBumperActivatedRight()
{
  return (_bumperRightActivated || _bumperDuinoActivated); // i leave the bumperduino but certainly not use
}
//-----------------------------------------------------


void TbumperSensor::run()
{
	runned();

	// Orignal Ardumower Bumper
#if CONF_DISABLE_BUMPER_SERVICE == false
 //bber-----------------------------------------------------
  if (!_bumperRightActivated && !_bumperLeftActivated && _bumperActivated) {
    if (flagShowBumper) {
      errorHandler.setInfo("!03,Bumper deactivated\r\n");
    }
    _bumperActivated = false;
  }

  if (( diBumperL == HIGH ) && _bumperLeftActivated) {
    if (flagShowBumper) {
      errorHandler.setInfo("!03,Bumper Left deactivated\r\n");
    }
    _bumperLeftActivated = false;
  }

  if (( diBumperR == HIGH ) && _bumperRightActivated) {
    if (flagShowBumper) {
      errorHandler.setInfo("!03,Bumper Right deactivated\r\n");
    }
    _bumperRightActivated = false;
  }

  if ((diBumperL == LOW) && !_bumperLeftActivated) {
    if (flagShowBumper) {
      errorHandler.setInfo("!03,Bumper left activated\r\n");
    }
    //bb.flagForceRotateDirection = FRD_CC;
    //bb.driveDirection = DD_FEOROTATECC;
    _bumperLeftActivated = true;
  }
  if ((diBumperR == LOW) && !_bumperRightActivated) {
    if (flagShowBumper) {
      errorHandler.setInfo("!03,Bumper right activated\r\n");
    }
    //bb.flagForceRotateDirection = FRD_CW;
    //bb.driveDirection = DD_FEOROTATECW;
    _bumperRightActivated = true;

  }

  if ((_bumperRightActivated || _bumperLeftActivated) && !_bumperActivated) {
    _bumperActivated = true;
    if (flagShowBumper) {
      errorHandler.setInfo("!03,Bumper activated\r\n");
    }
  }
  //----------------------------------------------
#endif

#if CONF_DISABLE_BUMPERDUINO_SERVICE == false 
	//#pragma message ("CONF_DISABLE_BUMPERDUINO_SERVICE AKTIVE")


			// Low active
	if (diBumperSensor == HIGH && _bumperDuinoActivated) {
		if (flagShowBumper) {
			errorHandler.setInfo("!03,BumperDuino deactivated\r\n");
		}
		_bumperDuinoActivated = false;
	}

	if (diBumperSensor == LOW && !_bumperDuinoActivated) {
		if (flagShowBumper) {
			errorHandler.setInfo("!03,BumperDuino activated\r\n");
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



