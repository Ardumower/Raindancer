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

#ifndef CHARGESYSTEM_H
#define CHARGESYSTEM_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "Thread.h"
#include "helpers.h"
#include "hardware.h"
#include "errorhandler.h"
#include "config.h"
#include "buzzer.h"

#define BATTERYFACTOR_CS  11.0f // ((100.0f+10.0f)/10.0f) ((R1+R2)/R2)  ADC voltage to charge voltage. 10.9 determined by measuring 
#define CURRENTFACTOR_CS  0.5f         // ADC voltage to current ampere

extern BuzzerClass Buzzer;

class TchargeSystem : public Thread
{
private:
    uint8_t count;

public:

    bool flagShowChargeSystem;

    float sensorValueCV;
	float offsetCV;
    float sensorValueCC;
	float offsetCC;
	float chargeVoltage;
    float chargeCurrent;

    void setup() {
        sensorValueCV = 0; // Converts and read the analog input value (value from 0.0 to 1.0)
		sensorValueCC = 0;
        chargeVoltage = 0;
        chargeCurrent = 0;
		offsetCV = 0;
		offsetCC = 0;
        flagShowChargeSystem = false;
        count = 20;
		doChargeEnable = LOW;
    }

    void activateRelay() {
		if (CONF_DISABLE_CHARGE_SERVICE) {
			return;
		}

        if(doChargeEnable == LOW){
			Buzzer.sound(SND_CHARGERELAYON);
		}
		doChargeEnable = HIGH;
		errorHandler.setInfo(F("Relay On\r\n"));
    }

    void deactivateRelay() {

		if (doChargeEnable != LOW) {
			Buzzer.sound(SND_CHARGERELAYOFF);
		}
		doChargeEnable = LOW;
		errorHandler.setInfo(F("Relay Off\r\n"));

		
    }


    virtual void run() {
        // Wird alle 53ms aufgerufen
        
        runned();
		if (CONF_DISABLE_CHARGE_SERVICE) {
			chargeVoltage = 0;
			chargeCurrent = 0;
			return;
		}
        sensorValueCV = aiCHARGEVOLTAGE.getVoltage() - offsetCV;
		if (sensorValueCV < 0) {
			sensorValueCV = 0;
		}
        float chargeVoltage1 = sensorValueCV  * BATTERYFACTOR_CS;

        sensorValueCC = aiCHARGECURRENT.getVoltage() - offsetCC;
		if (sensorValueCC < 0) {
			sensorValueCC = 0;
		}
		float chargeCurrent1 = sensorValueCC * CURRENTFACTOR_CS;

        const float accel = 0.1f;
       
              
        if (abs(chargeVoltage - chargeVoltage1)>5)
            chargeVoltage = chargeVoltage1;
        else
            chargeVoltage = (1.0f - accel) * chargeVoltage + accel * chargeVoltage1;

        if (abs(chargeCurrent - chargeCurrent1)>0.3)
            chargeCurrent = chargeCurrent1;
        else
            chargeCurrent = (1.0f - accel) * chargeCurrent + accel * chargeCurrent1;


        if(flagShowChargeSystem && (++count > 20) ) {

            if(isBatFull()) {
                sprintf(errorHandler.msg,"!03,FULL adcCV: %f CV: %f  adcCC: %f  CC: %f Watt: %f\r\n", sensorValueCV, chargeVoltage, sensorValueCC, chargeCurrent, chargeVoltage*chargeCurrent);
                errorHandler.setInfo(); 
            } else {
                sprintf(errorHandler.msg,"!03,CHARGING adcCV: %f CV: %f  adcCC: %f  CC: %f Watt: %f\r\n", sensorValueCV, chargeVoltage, sensorValueCC, chargeCurrent, chargeVoltage*chargeCurrent);
                errorHandler.setInfo(); 

            }
            count = 0;
        }

    }
//bber24
    bool isInChargingStation() {
        //if (chargeVoltage > 10.0f)  {
        if (chargeVoltage > 1.0f)  {
             //bber24
            errorHandler.setInfoNoLog(F("\r\n=== station voltage detected ===\r\n"));
            return true;
        }
        return false;
    }


    bool isBatFull() {
        //float watt = chargeVoltage * chargeCurrent;
        //if (watt < 8.0f)  {

         if(chargeCurrent < 0.3f){
            return true;
        }

        return false;
    }

	void measureOffset() {

		errorHandler.setInfo(F("!03,ChargeSystem measure offsetCV\r\n"));
		errorHandler.setInfoNoLog(F("!03,"));
		offsetCV = aiCHARGEVOLTAGE.measureOffsetVoltage();
		errorHandler.setInfo(F("  offsetCV: %f\r\n"), offsetCV);

		errorHandler.setInfo(F("!03,ChargeSystem measure offsetCC\r\n"));
		errorHandler.setInfoNoLog(F("!03,"));
		offsetCC = aiCHARGEVOLTAGE.measureOffsetVoltage();
		errorHandler.setInfo(F("  offsetCC: %f\r\n"), offsetCC);
	}


	void showConfig()
	{
		errorHandler.setInfoNoLog(F("!03,charge system\r\n"));
		errorHandler.setInfoNoLog(F("!03,enabled: %lu\r\n"), enabled);
		errorHandler.setInfoNoLog(F("!03,interval: %lu\r\n"), interval);
		errorHandler.setInfoNoLog(F("!03,relay on: %d\r\n"), doChargeEnable.read());
		errorHandler.setInfoNoLog(F("!03,BATTERYFACTOR_CS %f\r\n"), BATTERYFACTOR_CS);
		errorHandler.setInfoNoLog(F("!03,CURRENTFACTOR_CS %f\r\n"), CURRENTFACTOR_CS);
	}

};

#endif

