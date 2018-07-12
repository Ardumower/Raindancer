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
extern TMotorInterface motor;
extern bool _controlManuel;

class TShutdown : public Thread {
private:
    unsigned long startTime;
    uint8_t state;
public:

    void setup() {
        state = 0;
        }

    virtual void run() {
        unsigned long time;
        unsigned long delta;

        // Will be called every 1000ms if service is activated
        runned();

        switch (state) {
            case 0:
                startTime = millis();
                _controlManuel = true; // Set manual mode
                motor.stopAllMotors();
                if (CONF_WAIT_FOR_PI_SHUTDOWN) {
                    errorHandler.setInfoNoLog(F("!03,PowerOff requested. Wait 50 sec for Raspberry PI shutdown.\r\n"));
                    }
                else {
                    errorHandler.setInfoNoLog(F("!03,PowerOff requested.\n"));
                    }
                errorHandler.setInfoNoLog(F("$PwrOff\r\n"));
                state = 1;
                break;
            case 1:
                // wait that motors hs been stopped
                if (motor.isCLCStopped()) {
                    state = 2;
                    }
                break;
            case 2:
                errorHandler.setInfoNoLog(F("$PwrOff\r\n"));
                time = millis();
                delta = time - startTime;
                if (!CONF_WAIT_FOR_PI_SHUTDOWN) {
                    delta = 60000;
                    }
                if (delta > 50000) { //put 50 second as safety to be sure the PI have time to shutdown properly but certainly 20 Sec is OK
                    errorHandler.setInfoNoLog(F("!03,PCB OFF NOW\r\n"));
                    delay(50); // Wait 50ms that signal can send to the controlcenter
                    doBatteryOffSwitch = LOW;
                    state = 3;
                    }
                else {
                    errorHandler.setInfoNoLog(F("!03,Shutdown in %lu seconds\r\n"), (50000ul - delta) / 1000);
                    }
                break;
            case 3:
                doBatteryOffSwitch = LOW;
                startTime = millis();
                state = 4;
            case 4:
                time = millis();
                delta = time - startTime;
                if (delta > 3000) {
                    // In case of the shutdown is bridged on PCB1.3 deactivate service
                    enabled = false;
                    state = 0;
                    errorHandler.setInfoNoLog(F("!03,Looks like power off is bridged on PCB\r\n"));
                    }
                break;

            }//ENDSWITCH
        }//ENDRUN



    void showConfig() {
        errorHandler.setInfoNoLog(F("!03,Shurdown Config\r\n"));
        errorHandler.setInfoNoLog(F("!03,enabled: %lu\r\n"), enabled);
        errorHandler.setInfoNoLog(F("!03,interval: %lu\r\n"), interval);
        }
    };// ENDCLASS

#endif

