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

#include "rangeSensor.h"
#include "config.h"



void   TrangeSensor::setup() {

	_rangeActivated = false;
	flagShowRange = false;

};

bool TrangeSensor::isNearObstacle() {
	return _rangeActivated;
}


bool TrangeSensor::Run() {

	PT_BEGIN();
	while (1) {

		PT_YIELD_INTERVAL();

		if (CONF_DISABLE_RANGE_SERVICE) {
			_rangeActivated = false;
			PT_EXIT();
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
	PT_END();
}


void TrangeSensor::showConfig() {
	errorHandler.setInfo(F("!03,Range Sensor Config\r\n"));
	errorHandler.setInfo(F("!03,enabled: %d\r\n"), IsRunning());
	errorHandler.setInfo(F("!03,interval: %lu\r\n"), interval);
}



