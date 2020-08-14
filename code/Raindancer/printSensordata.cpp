/*
Robotic Lawn Mower
Copyright (c) 2017 by Kai WÃ¼rtz



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
#include "bCreateTree.h"
#include "chargeSystem.h"


extern unsigned long loopCounter;
extern unsigned long maxLoopTime;
extern unsigned long startLoopTime;

extern void executeLoop();
extern unsigned long lastTimeShowError;

// mow motor closed loop control - no closed loop used
extern TMowClosedLoopControlThread srvClcM;
// drive motor left closed loop control
extern TClosedLoopControlThread srvClcL;
// drive motor rigth closed loop control
extern TClosedLoopControlThread srvClcR;
//drive motor left position control
extern TPositionControl srvPcL;
//drive motor right position control
extern TPositionControl srvPcR;
// Motorensteuerung Interface. KEIN THREAD. Wird verwendet um clcX und pcX zu steuern.
extern TMotorInterface srvMotor;
// Daten von Perimetersensoren vom 446RE
extern TPerimeterThread srvPerSensoren;
// Messung der Batteriespannung
extern TbatterieSensor srvBatSensor;
// Messung des MÃ¤hmotorstroms
extern TMowMotorSensor srvMowMotorSensor;
// SRF08 Range Sensor Messung der Entfernung
extern TrangeSensor srvRangeSensor;
// Bumper Sensor
extern TbumperSensor srvBumperSensor;
// Charge System
extern TchargeSystem srvChargeSystem;

extern Blackboard myBlackboard;
extern TCreateTree myCreateTree;

extern TErrorHandler errorHandler;


bool _printProcessingData = false;

void printSensordata()
{

    if(_printProcessingData) {
        //errorHandler.setInfoNoLog(F("!01,%lu,%lu, %lu,%.2f,%.2f,%.2f,"),millis(),loopsPerSec, maxLoopTime, srvClcL.getCurrentSpeedInRPM(),srvClcR.getCurrentSpeedInRPM(),srvMowMotorSensor.watt);
        //errorHandler.setInfoNoLog(F("%d,%d,"),srvPerSensoren.magnetudeL , srvPerSensoren.magnetudeR);
        //errorHandler.setInfoNoLog(F("%.2f,"),srvBatSensor.voltage);
        //errorHandler.setInfoNoLog(F("%.2f,"),srvChargeSystem.chargeVoltage*srvChargeSystem.chargeCurrent);
        //errorHandler.setInfoNoLog(F("\r\n"));
//xdes1
        errorHandler.setInfoNoLog(F("$loop,%lu, %lu\r\n"), loopCounter, maxLoopTime);

		//must reset here, not to include the output above in the calculation
		startLoopTime = micros();
		maxLoopTime = 0;
        loopCounter = 0;
     }

}

