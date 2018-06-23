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

****************************************************************************************************
*      INTO CONFIG.H 
*
#define RAINDANCER_CHASSIS true
#define PARANELLO_CHASSIS false
*
Set to true only the correct CHASSIS
****************************************************************************************************
*/

#include "config.h"
#include "rainSensor.h"
#include "bRestoreHistory.h"
#include "buzzer.h"
#include "DueTimer.h"
#include "rtc.h"
#include "adcman.h"
#include "hardware.h"
#include "cmd.h"
#include "perimeter.h"
#include "Thread.h"
#include "ThreadController.h"
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
#include "chargeSystem.h"
#include "ui.h"
#include "printSensordata.h"
#include "bt.h"
#include "EEPROM.h"
#include "motorSensor.h"
#include "DHT.h"


/*********************************************************************/
// Global Variables
/*********************************************************************/
unsigned long loopCounter = 0;
unsigned long maxLoopTime = 0;
unsigned long startLoopTime = 0;




/* Private function prototypes -----------------------------------------------*/
void loop();


/*********************************************************************/
// Controller Threads die in bestimmten intervallen aufgerufen werden
// Oder Services die nicht in in de Threadcontroller integriert werden
// ACHTUNG: In TreadController.h die Anzahl der threads einstellen falls 15 überschritten wird
/*********************************************************************/
// Hardware abstaraction layer run function
Thread hal;
// mow motor closed loop control - no closed loop used
TMowClosedLoopControlThread clcM;
// drive motor left closed loop control
TClosedLoopControlThread clcL;
// drive motor rigth closed loop control
TClosedLoopControlThread clcR;
//drive motor left position control
TPositionControl pcL;
//drive motor right position control
TPositionControl pcR;
// Motorensteuerung Interface. Wird verwendet um clcX und pcX zu steuern und hat diverse testfunktionen
TMotorInterface motor;
// Verarbeitet Ein-/Ausgabe über Serial line console oder bluetooth. in hardware.cpp die debug zuweisung ändern.
Thread cmd;
// Daten von Perimetersensoren vom 446RE
TPerimeterThread perimeterSensoren;
// Messung der Batteriespannung
TbatterieSensor batterieSensor;
// Rain sensor
TrainSensor rainSensor;
// Messung des Motorstroms
TmotorSensor motorSensorL(aiMotorLeftCurrent, diMotorLeftFault, doMotorEnable, 'L');
TmotorSensor motorSensorR(aiMotorRightCurrent, diMotorRightFault, doMotorEnable, 'R');
TMowMotorSensor mowMotorSensor(diMotorMowFault, doMotorMowEnable);
// SRF08 Range Sensor Messung der Entfernung
TrangeSensor rangeSensor;
// Bumper Sensor
TbumperSensor bumperSensor;
// Charge System
TchargeSystem chargeSystem;
// Print Sensordata for processing
Thread processingSensorData;
// Real time clock
Trtc rtc;
// EEPROM will not insert in thread controller
TEEPROM eeprom;
// Buzzer
BuzzerClass Buzzer;

// DHT Temperature sensor
TDHT dht( DHTTYPE);

//-----------------------------------------------------


// Instantiate a new ThreadController
ThreadController controller = ThreadController(); // Thread die vor manuellen mode laufen müssen



/*********************************************************************/
// Behaviour Objects die auf die Threads oben zugreifen
/*********************************************************************/

Blackboard myBlackboard(motor, perimeterSensoren, mowMotorSensor, rangeSensor, bumperSensor, batterieSensor, encoderL, encoderR, chargeSystem, eeprom, rainSensor, dht);
TBehaviour myBehaviour(myBlackboard);


/*********************************************************************/
// Module Variables
/*********************************************************************/

unsigned long lastTimeShowError = 0;
bool _controlManuel = true;
bool _diableErrorhandling = false;


#if CONF_ENABLEWATCHDOG ==  true
void watchdogSetup(void) {} // muss definiert werden.
							// denn die default Funktion macht ein: WDT_Disable (WDT);
							// und das Disable verhindert den nächsten enable. somit kann Watchdog nicht mehr aktiviert werden.
#endif

void setup()
{
	//---------------------------------
	// Initialize interface to hardware
	//---------------------------------
	hardwareSetup();
	errorHandler.setInfo(F("HardwareSetup finished\r\n"));


	//---------------------------------
	// Threads konfigurieren
	// Motor-/Positionthreads laufen in 20ms bzw 100ms Takt.
	// Alle andern in anderen Intervallen.
	//---------------------------------

	hal.setInterval(0);
	hal.onRun(hardwareRun);

	clcM.setup(1);  // Mow Motor 1 of sabertooth
	clcM.setInterval(198);

	clcL.setup(1, &encoderL);  // Motor 1 of sabertooth
	clcL.setInterval(33);
	clcR.setup(2, &encoderR);  // Motor 2 of sabertooth
	clcR.setInterval(33);

	pcL.setup(&clcL, &encoderL);  // Position control left
	pcL.setInterval(100);
	pcR.setup(&clcR, &encoderR);  // Position control right
	pcR.setInterval(100);

	motor.setup(&clcM, &clcL, &clcR, &pcL, &pcR);
	motor.setInterval(100);

	//---------------------------------
	cmdInit();
	cmd.onRun(cmdPoll);
	cmd.setInterval(116);
	//---------------------------------
	errorHandler.setInfo(F("perimeterSensoren Setup Startet\r\n"));
	perimeterSensoren.coilL.showMatchedFilter = true;
	perimeterSensoren.setup();
	perimeterSensoren.coilL.showMatchedFilter = false;
	perimeterSensoren.setInterval(1); // immer aufrufen und checken ob ein neues byte empfangen wurde
	errorHandler.setInfo(F("perimeterSensoren Setup Finished\r\n"));
	//---------------------------------
	batterieSensor.setup();
	batterieSensor.setInterval(1974);

	rainSensor.setup();
	rainSensor.setInterval(1773);


	motorSensorL.setup();
	motorSensorL.setInterval(77);

	motorSensorR.setup();
	motorSensorR.setInterval(77);

	mowMotorSensor.setup();
	mowMotorSensor.setInterval(97);
	//---------------------------------
	rangeSensor.setup();
	rangeSensor.setInterval(137);
	//rangeSensor.enabled = false;
	//---------------------------------
	bumperSensor.setup();
	bumperSensor.setInterval(0);
	//---------------------------------
	chargeSystem.setup();
	chargeSystem.setInterval(53);
	//---------------------------------
	processingSensorData.onRun(printSensordata);
	processingSensorData.setInterval(1007);
	//---------------------------------
	rtc.setup();
	rtc.setInterval(10017);
	//---------------------------------
	eeprom.setup();
	eeprom.enabled = false; // run() not needed at the moment
	//---------------------------------
	Buzzer.setup();
	//Buzzer.setInterval(0); // will be controled by the class itselfe
	Buzzer.enabled = false;  // will be controled by the class itselfe
	//---------------------------------
	dht.setup();
	dht.setInterval(20013);


  //------------
	// ACHTUNG: In TreadController.h die Anzahl der threads einstellen falls 25 überschritten wird
	controller.add(&hal);

	controller.add(&clcM);
	controller.add(&clcL);
	controller.add(&clcR);
	controller.add(&pcL);
	controller.add(&pcR);
	controller.add(&motor);

	controller.add(&cmd);

	controller.add(&perimeterSensoren);

	controller.add(&batterieSensor);
	controller.add(&rainSensor);

	controller.add(&motorSensorL);
	controller.add(&motorSensorR);
	controller.add(&mowMotorSensor);

	controller.add(&rangeSensor);

	controller.add(&bumperSensor);

	controller.add(&chargeSystem);
	controller.add(&processingSensorData);

	controller.add(&rtc);

	controller.add(&Buzzer);

	//---------------------------------
	// Behaviour Objects konfigurieren
	//---------------------------------

	myBehaviour.setup();


	//---------------------------------
	// Userinterface setup
	//---------------------------------
	cmd_setup();

	motor.stopAllMotors();

	motorSensorL.measureOffset();
	motorSensorR.measureOffset();
	mowMotorSensor.measureOffset();

	// Check if in charging station
	errorHandler.setInfo(F("Check if in charging station\r\n"));
	for (int i = 0; i < 10; i++) { // Read charging voltage
		chargeSystem.run();
	}
	if (chargeSystem.isInChargingStation()) {
		errorHandler.setInfo(F("Charging station detected\r\n"));
		myBlackboard.setBehaviour(BH_CHARGING);
		_controlManuel = false;
	}
	else {
		errorHandler.setInfo(F("NO Charging station detected\r\n"));
		errorHandler.setInfo(F("MANUAL MODE ACTIVATED\r\n"));
		_controlManuel = true;
		chargeSystem.measureOffset();
	}

	errorHandler.setInfo(F("Setup finished. Loop is running.\r\n\r\n"));

	errorHandler.setInfo(F("Press H for help.\r\n"));

	// Show perimeter signals with arduino serial plotter
	//perimeterSensoren.coilL.showADCWithoutOffset = true;
	//perimeterSensoren.coilL.showCorrelation = true;
	//perimeterSensoren.coilL.showPSNRFunction = true;
	//perimeterSensoren.coilR.showADCWithoutOffset = true;
	//perimeterSensoren.coilR.showCorrelation = true;
	//perimeterSensoren.coilR.showPSNRFunction = true;


	//Startsound ausgeben
	Buzzer.sound(SND_START);


#if CONF_ENABLEWATCHDOG ==  true
	// Wenn Watchdog enabled/disabled wurde, kann dieser nicht wieder disabled/enabled werden.
	// Das geht nurl wenn der Microcontroller neu gestartet wird.
	watchdogEnable(3000);
	errorHandler.setInfo(F("WATCHDOG ENABLED\r\n"));
#endif


}

void executeLoop() //wird von ui.cpp verwendet wenn Hilfe ausgegeben wird
{
	loop();
}

void loop()
{
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
				motor.stopAllMotors();

				if (millis() - lastTimeShowError > 2000) {
					lastTimeShowError = millis();
					doMyLED = !doMyLED;
					errorHandler.printError();
				}
			}
		}

		// Run the services
		controller.run();


		//clcR.testEncoder();

		// Auswerten ob user manuellen mode gesetzt hat und kommando ausführen
		// Auszuwertende variablen sind in Klasse TSerialEventThread definiert
		if (_controlManuel) {

		}
		else {
			myBehaviour.loop();
		}

		unsigned long loopTime = micros() - startLoopTime;
		if (loopTime > maxLoopTime) {
			maxLoopTime = loopTime;
			//must reset in printSensorData, not to include the output of Senordata in the calculation
		}

	//}//while(true)
}




