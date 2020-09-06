// shutdown.h

#ifndef _SHUTDOWN_h
#define _SHUTDOWN_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "hardware.h"
#include "motor.h"
#include "config.h"
#include "Protothread.h"

extern TMotorInterface srvMotor;
extern bool _controlManuel;

class TShutdown : public Protothread {
private:
	unsigned long startTime;
public:

	void setup() {
		Stop();
	}
	// When the mower is running, this service is disabled. 
	// The mower can be power off by the temp service, bat service or over the UI interface.
	// When the mower should shutdown, the service will be enabled and begin the power off sequece.
	bool Run() {
		unsigned long time;
		unsigned long delta;

		if (millis() - last_run < interval) return true;

		last_run = millis();

		switch (_ptLine) {
		case 0:
			startTime = millis();
			_controlManuel = true; // Set manual mode
			srvMotor.stopAllMotors();
			if (CONF_WAIT_FOR_PI_SHUTDOWN) {
				errorHandler.setInfo(F("!03,PowerOff requested. Wait 50 sec for Raspberry PI shutdown.\r\n"));
			}
			else {
				errorHandler.setInfo(F("!03,PowerOff requested.\n"));
			}
			errorHandler.setInfo(F("$PwrOff\r\n"));
			_ptLine = 1;
			break;
		case 1:
			// wait that motors hs been stopped
			if (srvMotor.isCLCStopped()) {
				_ptLine = 2;
			}
			break;
		case 2:
			errorHandler.setInfo(F("$PwrOff\r\n"));
			time = millis();
			delta = time - startTime;
			if (!CONF_WAIT_FOR_PI_SHUTDOWN) {
				delta = 60000;
			}
			if (delta > 50000) { //put 50 second as safety to be sure the PI have time to shutdown properly but certainly 20 Sec is OK
				errorHandler.setInfo(F("!03,PCB OFF NOW\r\n"));
				delay(50); // Wait 50ms that signal can send to the controlcenter
				doBatteryOffSwitch = LOW;
				_ptLine = 3;
			}
			else {
				errorHandler.setInfo(F("!03,Shutdown in %lu seconds\r\n"), (50000ul - delta) / 1000);
			}
			break;
		case 3:
			doBatteryOffSwitch = LOW;
			startTime = millis();
			_ptLine = 4;
		case 4:
			time = millis();
			delta = time - startTime;
			if (delta > 3000) {
				// In case of the shutdown is bridged on PCB1.3 deactivate service
				Stop();
				errorHandler.setInfo(F("!03,Looks like power off is bridged on PCB\r\n"));
			}
			break;
		default:;
		}//ENDSWITCH

		return true;
	}//ENDRUN



	void showConfig() {
		errorHandler.setInfo(F("!03,Shutdown Config\r\n"));
		errorHandler.setInfo(F("!03,enabled: %d\r\n"), IsRunning());
		errorHandler.setInfo(F("!03,interval: %lu\r\n"), interval);
	}
};// ENDCLASS

#endif

