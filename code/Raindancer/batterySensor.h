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

#ifndef BATTERIESENSOR_H
#define BATTERIESENSOR_H

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
//xdes1
#include "shutdown.h"

#define BATTERYFACTOR_BS 11.0f // Normally: (100+10) / 10; Voltagedivider. 10.9 determined by measuring
#define DIODEDROPVOLTAGE_BS 0.4f

//xdes1
extern TShutdown shutdown;

class TbatterieSensor : public Thread
{
  private:
    unsigned long time;
    bool flagShowVoltage;
    int8_t count = 10;
  public:

    float sensorValue;
    float voltage;
    //bber20
    uint16_t dataRate;
    uint16_t recordSentToCCO = 10;
    uint16_t maxRecordReturn;
    
    void setup() {
      sensorValue = 0;
      voltage = 24;
      //bber20
      dataRate = 3;
      maxRecordReturn = 0;//deactivate the cco on startup
      sensorValue = aiBATVOLT.getVoltage();
      time = millis();
    }
    //xdes1
    void showData() {
      errorHandler.setInfoNoLog(F("$batV,%f\r\n"), voltage);
    }
    //xdes1
    void show() {
      flagShowVoltage = true;
      showData();
    }
    //xdes1
    void hide() {
      flagShowVoltage = false;
    }

    virtual void run() {
      // Wird alle 1000ms aufgerufen
      runned();

      if (CONF_DISABLE_BATTERY_SERVICE) {
        sensorValue = 0;
        voltage = 24;
        return;
      }

      sensorValue = aiBATVOLT.getVoltage();
      float readVolt = sensorValue * BATTERYFACTOR_BS + DIODEDROPVOLTAGE_BS; // The diode sucks 0.4V

      const float accel = 0.1f;

      if (flagShowVoltage) {
         count++;
        //bber20
        if ((count >= dataRate) && (recordSentToCCO <= maxRecordReturn)) { // show value not every time the service is called
          showData();
          recordSentToCCO++;
          count = 0;
        }
      }

      if (abs(voltage - readVolt) > 5) {
        voltage = readVolt;
      }
      else {
        voltage = (1.0f - accel) * voltage + accel * readVolt;
      }


      if (voltage < CONF_VOLTAGE_SWITCHOFF_BS) {
        unsigned long dt = millis() - time;
        errorHandler.setInfo(F("!03,Switch off voltage reached. Time: %lu dt: %lu\r\n"), time , dt);

        // Wait 60sec before switch off.
        if (dt > 60000ul) {
          shutdown.enabled = true;
        }
      }
      else {
        time = millis();
        //errorHandler.setInfo("set time= &lu", time);
      }

    }

    bool isVoltageLow() {
      if (voltage < CONF_VOLTAGE_LOW_BS) {
        return true;
      }

      return false;
    }

    void showConfig()
    {
      errorHandler.setInfoNoLog(F("!03,Battery Sensor Config\r\n"));
      errorHandler.setInfoNoLog(F("!03,enabled: %lu\r\n"), enabled);
      errorHandler.setInfoNoLog(F("!03,interval: %lu\r\n"), interval);
      errorHandler.setInfoNoLog(F("!03,BATTERYFACTOR_BS %f\r\n"), BATTERYFACTOR_BS);
      errorHandler.setInfoNoLog(F("!03,DIODEDROPVOLTAGE_BS %f\r\n"), DIODEDROPVOLTAGE_BS);
      errorHandler.setInfoNoLog(F("!03,VOLTAGE_LOW_BS %f\r\n"), CONF_VOLTAGE_LOW_BS);
      errorHandler.setInfoNoLog(F("!03,VOLTAGE_SWITCHOFF_BS %f\r\n"), CONF_VOLTAGE_SWITCHOFF_BS);
    }

};

#endif

