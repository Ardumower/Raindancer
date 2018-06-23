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

#include "rangeSensor.h"
#include "config.h"



void   TrangeSensor::setup()
{

	_rangeActivated  = false;
	flagShowRange = false;

};

bool TrangeSensor::isNearObstacle()
{
	return _rangeActivated;
}


void TrangeSensor::run()
{


	runned();

	if (CONF_DISABLE_RANGE_SERVICE) {
		_rangeActivated = false;
		return;
	}


	//if (flagShowRange) {
	//errorHandler.setInfo("!03,Range %d\r\n", diNearObsacleSensor.read());
	//}

	// Low active
	if (diNearObsacleSensor == HIGH && _rangeActivated == true) {
		_rangeActivated = false;
		if (flagShowRange) {
			errorHandler.setInfo(F("!03,Range deactivated\r\n"));
		}
	}

	if (diNearObsacleSensor == LOW && _rangeActivated == false) {
		_rangeActivated = true;
		if (flagShowRange) {
			errorHandler.setInfo(F("!03,Range activated\r\n"));
		}
	}


}


void TrangeSensor::showConfig()
{
	errorHandler.setInfoNoLog(F("!03,Range Sensor Config\r\n"));
	errorHandler.setInfoNoLog(F("!03,enabled: %lu\r\n"), enabled);
	errorHandler.setInfoNoLog(F("!03,interval: %lu\r\n"), interval);
}



