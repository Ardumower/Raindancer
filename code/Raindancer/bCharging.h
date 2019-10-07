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

#ifndef BH_CHARGING_H
#define BH_CHARGING_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "BehaviourTree.h"
#include "EEPROM.h"
//bber1  add to know the kind of chassis and station type
#include "config.h"
//-----------


class TchargeRelayOn : public Action
{
private:

	uint8_t state;
	uint8_t connectAttempts;

public:

	TchargeRelayOn() {} //: firstTimeCall(false){}

	virtual void onInitialize(Blackboard& bb) {
		//firstTimeCall = true;
#if CONF_DISABLE_EEPROM_SERVICE == false
		int32_t count = bb.eeprom.read32t(EEPADR_CHARGINGCOUNT);
		count++;
		bb.eeprom.write32t(EEPADR_CHARGINGCOUNT, count);
#endif
		//bb.chargeSystem.activateRelay();

		state = 0;
		connectAttempts = 0;
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		// grind the contacts a little to get better connections
		switch (state)
		{
		case 0:
			if (getTimeInNode() > 1000) { // wait 1 sec. until contacts set
      //bber1
        if (CONF_PASS_THROUGH_CHARGING_STATION == true) {  //forward 4 cm 
          bb.motor.rotateCM(4, bb.CRUISE_SPEED_LOW);
          errorHandler.setInfo(F("TchargeRelayOn forward 4\r\n"));
        }
      //------------  
				state = 1;
				setTimeInNode(millis());
				connectAttempts++;
			}
			break;
		case 1:
			if (bb.motor.isPositionReached()) {
				bb.motor.rotateCM(-3, bb.CRUISE_SPEED_LOW);
				state = 2;
				setTimeInNode(millis());
				errorHandler.setInfo(F("TchargeRelayOn backward -3\r\n"));
			}
			if (getTimeInNode() > 10000) {
				errorHandler.setError(F("!03,TchargeRelayOn too long in state\r\n"));
			}
			break;
		case 2:
			if (bb.motor.isPositionReached()) {
        //bber1
        if (CONF_HEAD_CHARGING_STATION == true) bb.motor.rotateCM(3, bb.CRUISE_SPEED_LOW); // back in station for stoping station
        if (CONF_PASS_THROUGH_CHARGING_STATION == true) bb.motor.rotateCM(2, bb.CRUISE_SPEED_LOW);// normal forward for raindancer chassis
        //--------------------------
				state = 3;
				setTimeInNode(millis());
				errorHandler.setInfo(F("TchargeRelayOn forward 2\r\n"));
			}
			if (getTimeInNode() > 10000) {
				errorHandler.setError(F("!03,TchargeRelayOn too long in state\r\n"));
			}
		case 3:
			if (bb.motor.isPositionReached()) {
				state = 4;
				bb.chargeSystem.activateRelay();
				setTimeInNode(millis());
				errorHandler.setInfo(F("TchargeRelayOn get contact finished\r\n"));
			}
			if (getTimeInNode() > 10000) {
				errorHandler.setError(F("!03,TchargeRelayOn too long in state\r\n"));
			}
			break;
		case 4:

			if (getTimeInNode() > 10000) { // Check after 10 seconds, if current is flowing
				if (bb.chargeSystem.chargeCurrent > 0.001f) {
					state = 5;
					errorHandler.setInfo(F("TchargeRelayOn second check ok\r\n"));
				}
				else {
					errorHandler.setInfo(F("TchargeRelayOn attemp %c check failed\r\n"), connectAttempts);
					state = 0;
					bb.chargeSystem.deactivateRelay();
					setTimeInNode(millis());
	   			    if (connectAttempts > 2) {
						state = 5;
						errorHandler.setInfo(F("TchargeRelayOn not able to charge. Relay off!!!\r\n"));
						break;
					}
				}
			}
			break;
		default:
			break;
		}


		return BH_RUNNING;
	}

	virtual void onTerminate(NodeStatus status, Blackboard& bb) {
		bb.chargeSystem.deactivateRelay();
	}
};



#endif

