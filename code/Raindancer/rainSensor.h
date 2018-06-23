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
	bool _isRaining;
	
public:
	bool flagShowRainSensor;

	void setup() {
		_isRaining = false;
	}


	virtual void run() {
		// Wird alle 1700ms aufgerufen
		runned();

		if (CONF_DISABLE_RAIN_SERVICE) {
			return;
		}

		_isRaining = (diPinRain == LOW);

		if (flagShowRainSensor) {
			errorHandler.setInfo(F("Is raining: %d\r\n"), _isRaining);
		}
	}

	bool isRaining() {
	
			return _isRaining;
	}

	void showConfig()
	{
		errorHandler.setInfoNoLog(F("!03,Rain Sensor Config\r\n"));
		errorHandler.setInfoNoLog(F("!03,enabled: %lu\r\n"), enabled);
		errorHandler.setInfoNoLog(F("!03,interval: %lu\r\n"), interval);
		errorHandler.setInfoNoLog(F("!03,is raining: %d\r\n"), _isRaining);
	}

};
#endif

