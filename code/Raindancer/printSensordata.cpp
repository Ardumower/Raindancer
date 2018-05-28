/*
Robotic Lawn Mower
Copyright (c) 2017 by Kai WÃ¼rtz

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

#include "printSensordata.h"


#include "hardware.h"
#include "cmd.h"
#include "perimeter.h"
#include "batterySensor.h"
#include "motor.h"
#include "closedloopcontrol.h"
#include "mowclosedloopcontrol.h"
#include "errorhandler.h"
#include "mowmotorSensor.h"
#include "rangeSensor.h"
#include "bumperSensor.h"
#include "Blackboard.h"
#include "behaviour.h"
#include "bPerimeterTracking.h"
#include "chargeSystem.h"
#include "bRotate.h"

extern unsigned long loopCounter;
extern unsigned long maxLoopTime;
extern unsigned long startLoopTime;

extern void executeLoop();
extern unsigned long lastTimeShowError;

// mow motor closed loop control - no closed loop used
extern TMowClosedLoopControlThread clcM;
// drive motor left closed loop control
extern TClosedLoopControlThread clcL;
// drive motor rigth closed loop control
extern TClosedLoopControlThread clcR;
//drive motor left position control
extern TPositionControl pcL;
//drive motor right position control
extern TPositionControl pcR;
// Motorensteuerung Interface. KEIN THREAD. Wird verwendet um clcX und pcX zu steuern.
extern TMotorInterface motor;
// Daten von Perimetersensoren vom 446RE
extern TPerimeterThread perimeterSensoren;
// Messung der Batteriespannung
extern TbatterieSensor batterieSensor;
// Messung des MÃ¤hmotorstroms
extern TMowMotorSensor mowMotorSensor;
// SRF08 Range Sensor Messung der Entfernung
extern TrangeSensor rangeSensor;
// Bumper Sensor
extern TbumperSensor bumperSensor;
// Charge System
extern TchargeSystem chargeSystem;

extern Blackboard myBlackboard;
extern TBehaviour myBehaviour;

extern TErrorHandler errorHandler;


bool _printProcessingData = false;

void printSensordata()
{

    if(_printProcessingData) {
        unsigned long loopsPerSec;
        loopsPerSec = loopCounter;
        loopCounter = 0;

		errorHandler.setInfoNoLog(F("!01,%lu,%lu, %lu,%.2f,%.2f,%.2f,"),millis(),loopsPerSec, maxLoopTime, clcL.getCurrentSpeedInRPM(),clcR.getCurrentSpeedInRPM(),mowMotorSensor.watt);
		errorHandler.setInfoNoLog(F("%d,%d,"),perimeterSensoren.magnetudeL , perimeterSensoren.magnetudeR);
		errorHandler.setInfoNoLog(F("%.2f,"),batterieSensor.voltage);
		errorHandler.setInfoNoLog(F("%.2f,"),chargeSystem.chargeVoltage*chargeSystem.chargeCurrent);
       
		errorHandler.setInfoNoLog(F("\r\n"));

		//must reset here, not to include the output above in the calculation
		startLoopTime = micros();
		maxLoopTime = 0;
     }

}

