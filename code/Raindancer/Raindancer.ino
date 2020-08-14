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



****************************************************************************************************
*      INTO CONFIG.H
*
#define RAINDANCER_CHASSIS true
#define PARANELLO_CHASSIS false
*
Set to true only the correct CHASSIS
****************************************************************************************************
*/

#include <Wire.h>
#include "config.h"

#include "DueTimer.h"
#include "adcman.h"
#include "cmd.h"
#include "Thread.h"
#include "ThreadController.h"
#include "Blackboard.h"
#include "bCreateTree.h"
#include "ui.h"
#include "bt.h"

#include "UseServices.h"

/*********************************************************************/
// Global Variables
/*********************************************************************/
#define VERSION "2.0.0 Raindancer"

unsigned long loopCounter = 0;
unsigned long maxLoopTime = 0;
unsigned long startLoopTime = 0;
bool _controlManuel = true;

/* Private function prototypes -----------------------------------------------*/
void loop();

/*********************************************************************/
// Controller threads that are called at certain intervals
// Or services that are not integrated into the thread controller
// ATTENTION: In TreadController.h set the number of threads if 15 is exceeded
/*********************************************************************/
// Hardware abstaraction layer run function
Thread srvHal;
// mow motor closed loop control - no closed loop used
TMowClosedLoopControlThread srvClcM;
// drive motor left closed loop control
TClosedLoopControlThread srvClcL;
// drive motor rigth closed loop control
TClosedLoopControlThread srvClcR;
//drive motor left position control
TPositionControl srvPcL;
//drive motor right position control
TPositionControl srvPcR;
// Motorensteuerung Interface. Wird verwendet um clcX und pcX zu steuern und hat diverse testfunktionen
TMotorInterface srvMotor;
// Verarbeitet Ein-/Ausgabe über Serial line console oder bluetooth. in hardware.cpp die debug zuweisung ändern.
Thread srvCmd;
// Daten von Perimetersensoren vom 446RE
TPerimeterThread srvPerSensoren;
// Messung der Batteriespannung
TbatterieSensor srvBatSensor;
// Rain sensor
TrainSensor srvRainSensor;
// Messung des Motorstroms
TmotorSensor srvMotorSensorL(aiMotorLeftCurrent, diMotorLeftFault, doMotorEnable, 'L');
TmotorSensor srvMotorSensorR(aiMotorRightCurrent, diMotorRightFault, doMotorEnable, 'R');
TMowMotorSensor srvMowMotorSensor(diMotorMowFault, doMotorMowEnable);
// SRF08 Range Sensor Messung der Entfernung
TrangeSensor srvRangeSensor;
// Bumper Sensor
TbumperSensor srvBumperSensor;
// Charge System
TchargeSystem srvChargeSystem;
// Print Sensordata for processing
Thread srvProcessingSensorData;
// Real time clock
Trtc srvRtc;
// EEPROM will not be inserted in thread controller
TEEPROM srvEeprom;
// Buzzer
BuzzerClass srvBuzzer;
// Shutdown service
TShutdown srvShutdown;
// DHT Temperature sensor
TDHT srvDht(DHTTYPE);
//GPS Service
Tgps srvGps;
//-----------------------------------------------------

// Instantiate a new ThreadController
ThreadController controller = ThreadController(); // Thread die vor manuellen mode laufen müssen

/*********************************************************************/
// Behaviour Objects die auf die Threads oben zugreifen
/*********************************************************************/

Blackboard myBlackboard;
TCreateTree myCreateTree(myBlackboard);


/*********************************************************************/
// Module Variables
/*********************************************************************/

//unsigned long lastTimeShowError = 0;
bool _diableErrorhandling = false;


#if CONF_ENABLEWATCHDOG ==  true
void watchdogSetup(void) {} // muss definiert werden.
							// denn die default Funktion macht ein: WDT_Disable (WDT);
							// und das Disable verhindert den nächsten enable. somit kann Watchdog nicht mehr aktiviert werden.
#endif

void setup() {
	//---------------------------------
	// Initialize interface to hardware
	//---------------------------------
	hardwareSetup();
	errorHandler.setInfo(F("HardwareSetup finished\r\n"));


	//---------------------------------
	// Threads configuration
	// Motor-/Positionthreads runs every 33ms and 100ms.
	// The other services should not run in the same intervalls or a a multiple of that.
	//---------------------------------

	srvHal.setInterval(0);
	srvHal.onRun(hardwareRun);

	srvClcM.setup(1);  // Mow Motor 1 of sabertooth
	srvClcM.setInterval(198);

	srvClcL.setup(1, &encoderL);  // Motor 1 of sabertooth
	srvClcL.setInterval(33);
	srvClcR.setup(2, &encoderR);  // Motor 2 of sabertooth
	srvClcR.setInterval(33);

	srvPcL.setup(&srvClcL, &encoderL);  // Position control left
	srvPcL.setInterval(100);
	srvPcR.setup(&srvClcR, &encoderR);  // Position control right
	srvPcR.setInterval(100);

	srvMotor.setup(&srvClcM, &srvClcL, &srvClcR, &srvPcL, &srvPcR);
	srvMotor.setInterval(100);

	//---------------------------------
	cmdInit();
	srvCmd.onRun(cmdPoll);
	srvCmd.setInterval(116);
	//---------------------------------
	errorHandler.setInfo(F("srvPerSensoren Setup Startet\r\n"));
	srvPerSensoren.coilL.showMatchedFilter = true;
	srvPerSensoren.setup();
	srvPerSensoren.coilL.showMatchedFilter = false;
	srvPerSensoren.setInterval(1); // immer aufrufen und checken ob ein neues byte empfangen wurde
	errorHandler.setInfo(F("srvPerSensoren Setup Finished\r\n"));
	//---------------------------------
	srvBatSensor.setup();
	srvBatSensor.setInterval(1974);

	srvRainSensor.setup();
	srvRainSensor.setInterval(1773);


	srvMotorSensorL.setup();
	srvMotorSensorL.setInterval(77);

	srvMotorSensorR.setup();
	srvMotorSensorR.setInterval(77);

	srvMowMotorSensor.setup();
	srvMowMotorSensor.setInterval(97);
	//---------------------------------
	srvRangeSensor.setup();
	srvRangeSensor.setInterval(137);
	//srvRangeSensor.enabled = false;
	//---------------------------------
	srvBumperSensor.setup();
	srvBumperSensor.setInterval(0);
	//---------------------------------
	srvChargeSystem.setup();
	srvChargeSystem.setInterval(53);
	//---------------------------------
	srvProcessingSensorData.onRun(printSensordata);
	srvProcessingSensorData.setInterval(1001);
	//---------------------------------
	srvRtc.setup();
	srvRtc.setInterval(10017);
	//---------------------------------
	srvEeprom.setup();
	srvEeprom.enabled = false; // run() not needed at the moment
	//---------------------------------
	srvBuzzer.setup();
	//srvBuzzer.setInterval(0); // will be controled by the class itselfe
	srvBuzzer.enabled = false;  // will be controled by the class itselfe
    //---------------------------------
	srvShutdown.setup();
	srvShutdown.setInterval(1000);
	srvShutdown.enabled = false;  // when activated, service initiate shutdown
	//---------------------------------
	srvDht.setup();
	srvDht.setInterval(20013);
	//---------------------------------
	srvGps.setup();
	srvGps.setInterval(0);


	//------------
	  // Important: In TreadController.h the number of threads must be configured if services are more than 25
	controller.add(&srvHal);

	controller.add(&srvClcM);
	controller.add(&srvClcL);
	controller.add(&srvClcR);
	controller.add(&srvPcL);
	controller.add(&srvPcR);
	controller.add(&srvMotor);

	controller.add(&srvCmd);

	controller.add(&srvPerSensoren);

	controller.add(&srvBatSensor);
	controller.add(&srvRainSensor);

	controller.add(&srvMotorSensorL);
	controller.add(&srvMotorSensorR);
	controller.add(&srvMowMotorSensor);

	controller.add(&srvRangeSensor);

	controller.add(&srvBumperSensor);

	controller.add(&srvChargeSystem);
	controller.add(&srvProcessingSensorData);

	controller.add(&srvRtc);

	controller.add(&srvBuzzer);

	controller.add(&srvShutdown);
	controller.add(&srvGps);
	//---------------------------------
	// Behaviour Objects konfigurieren
	//---------------------------------

	myCreateTree.setup();


	//---------------------------------
	// Userinterface setup
	//---------------------------------
	cmd_setup();

	srvMotor.stopAllMotors();

	srvMotorSensorL.measureOffset();
	srvMotorSensorR.measureOffset();
	srvMowMotorSensor.measureOffset();

	// Check if in charging station
	errorHandler.setInfo(F("Check if in charging station\r\n"));
	for (int i = 0; i < 10; i++) { // Read charging voltage
		srvChargeSystem.run();
	}
	if (srvChargeSystem.isInChargingStation()) {
		errorHandler.setInfo(F("Charging station detected\r\n"));
		myBlackboard.setBehaviour(BH_CHARGING);
		_controlManuel = false;
	}
	else {
		errorHandler.setInfo(F("NO Charging station detected\r\n"));
		errorHandler.setInfo(F("MANUAL MODE ACTIVATED\r\n"));
		_controlManuel = true;
		srvChargeSystem.measureOffset();
	}


	// Show perimeter signals with arduino serial plotter
	//srvPerSensoren.coilL.showADCWithoutOffset = true;
	//srvPerSensoren.coilL.showCorrelation = true;
	//srvPerSensoren.coilL.showPSNRFunction = true;
	//srvPerSensoren.coilR.showADCWithoutOffset = true;
	//srvPerSensoren.coilR.showCorrelation = true;
	//srvPerSensoren.coilR.showPSNRFunction = true;


#if CONF_ENABLEWATCHDOG ==  true
	// Wenn Watchdog enabled/disabled wurde, kann dieser nicht wieder disabled/enabled werden.
	// Das geht nur wenn der Microcontroller neu gestartet wird.
	watchdogEnable(3000);
	errorHandler.setInfo(F("WATCHDOG ENABLED\r\n"));
#endif

	errorHandler.setInfo(F("Setup finished. Loop is running.\r\n"));
	errorHandler.setInfo(F("Version %s\r\n\r\n"), VERSION);
	errorHandler.setInfo(F("Press H for help.\r\n"));


#if TEST_ON_DUE_ONLY ==  true
	errorHandler.setInfo(F("********************\r\n"));
	errorHandler.setInfo(F("* TEST_ON_DUE_ONLY *\r\n"));
	errorHandler.setInfo(F("********************\r\n"));
#endif

	//Startsound ausgeben
	srvBuzzer.sound(SND_START);
}

void executeLoop() //wird von ui.cpp verwendet wenn Hilfe ausgegeben wird
{
	loop();
}

void loop() {
	// we don't use serialevent
	//while (true) {

#if  CONF_ENABLEWATCHDOG ==  true
	watchdogReset();
#endif


	//Show that loop is running and not hangs
	/*
	if (lastTimeShowError == 0) {
		lastTimeShowError = millis();
	}
	if (millis() - lastTimeShowError > 2000) {
		lastTimeShowError = millis();
		doMyLED = !doMyLED;
		debug.serial.println(lastTimeShowError);
	}
	*/

	startLoopTime = micros();
	loopCounter++;

	if (!_diableErrorhandling) {
		if (errorHandler.isErrorActive()) {
			_controlManuel = true;
			srvMotor.stopAllMotors();

			/*
				  if (millis() - lastTimeShowError > 2000) {
					  lastTimeShowError = millis();
					  doMyLED = !doMyLED;
					  errorHandler.printError();
				  }
			*/
		}
	}

	// Run the services
	controller.run();


	//srvClcR.testEncoder();

	// Auswerten ob user manuellen mode gesetzt hat und kommando ausführen
	// Auszuwertende variablen sind in Klasse TSerialEventThread definiert
	if (_controlManuel) {
		srvDht.run(); // run srvDht here, because in auto it runs from BHT and not from controller
	}
	else {
		myCreateTree.loop();
	}

	unsigned long loopTime = micros() - startLoopTime;
	if (loopTime > maxLoopTime) {
		maxLoopTime = loopTime;
		//must reset in printSensorData, not to include the output of Senordata in the calculation
	}

	//}//while(true)
}




