// rainSensor.h

#ifndef _RAINSENSOR_h
#define _RAINSENSOR_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif
/*
http://www.netzmafia.de/skripten/hardware/RasPi/Projekt-Regensensor/index.html
// Sensoreingang
#define PIN 2
// Integrationszeit in Sekunden
#define LIMIT 60

int State, Sum;

void setup()
{
pinMode(PIN, INPUT);
Serial.begin(9600);
}

void loop()
{
State = digitalRead(PIN);
if (State == 1)
{ Sum = (Sum < LIMIT)? Sum + 1 : LIMIT; }
else
{ Sum = (Sum > 0)? Sum - 1 : 0; }
State = (Sum > 0);
Serial.println(State,DEC);
delay(1000);
}
*/

#include "Thread.h"
#include "hardware.h"
#include "errorhandler.h"
#include "config.h"

class TrainSensor : public Thread
    {
    private:
        bool _isRainingDefault, _isRainingADC;
        byte _count;

    public:
        bool flagShowRainSensor;

        void setup()
            {
            _isRainingDefault = false;
            _isRainingADC = false;
            _count = 0;
            }


        virtual void run()
            {
            // Will be called every 1773ms
            runned();

            if (CONF_DISABLE_RAIN_SERVICE)
                {
                return;
                }

            if (CONF_RAINSENSOR_USE_ADC)
                {
                int value32 = aiPinRain.read_int32();

                if (value32 < CONF_RAINSENSOR_ADC_THRESHOLD)
                    {
                    _count++;
                    if (_count > 100) {_count = 100;} // limit count
                    if (_count > 5) {_isRainingADC = true;}
                    }
                else
                    {
                    _count = 0;
                    _isRainingADC = false;
                    }

                if (flagShowRainSensor)
                    {
                    errorHandler.setInfo(F("Is raining: %d adc: %d count: %d\r\n"), _isRainingADC, value32, _count);
                    }
                }

            if (CONF_RAINSENSOR_USE_DEFAULT)
                {
                _isRainingDefault = (diPinRain == LOW);
                if (flagShowRainSensor)
                    {
                    errorHandler.setInfo(F("Is raining: %d\r\n"), _isRainingDefault);
                    }
                }
            }

        bool isRaining()
            {
            return _isRainingDefault || _isRainingADC;
            }

        void showConfig()
            {
            errorHandler.setInfoNoLog(F("!03,Rain Sensor Config\r\n"));
            errorHandler.setInfoNoLog(F("!03,enabled: %lu\r\n"), enabled);
            errorHandler.setInfoNoLog(F("!03,interval: %lu\r\n"), interval);
            errorHandler.setInfoNoLog(F("!03,is _isRainingDefault: %d\r\n"), _isRainingDefault);
            errorHandler.setInfoNoLog(F("!03,is _isRainingADC: %d\r\n"), _isRainingADC);
            }

    };
#endif

