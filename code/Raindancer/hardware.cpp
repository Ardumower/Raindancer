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

#include "hardware.h"
#include "pinman.h"
#include "adcman.h"
#include <Wire.h>
#include "errorhandler.h"
#include "config.h"



#define IOREF 3.3   // I/O reference voltage 

// ------ pins---------------------------------------
#define pinMotorEnable  37         // EN motors enable
#define pinMotorLeftPWM 5          // M1_IN1 left motor PWM pin
#define pinMotorLeftDir 31         // M1_IN2 left motor Dir pin
#define pinMotorLeftSense A1       // M1_FB  left motor current sense
#define pinMotorLeftFault 25       // M1_SF  left motor fault

#define pinMotorRightPWM  3        // M2_IN1 right motor PWM pin
#define pinMotorRightDir 33        // M2_IN2 right motor Dir pin
#define pinMotorRightSense A0      // M2_FB  right motor current sense
#define pinMotorRightFault 27      // M2_SF  right motor fault

#define pinMotorMowPWM 2           // M1_IN1 mower motor PWM pin (if using MOSFET, use this pin)
#define pinMotorMowDir 29          // M1_IN2 mower motor Dir pin (if using MOSFET, keep unconnected)
#define pinMotorMowSense A3        // M1_FB  mower motor current sense  
#define pinMotorMowFault 26        // M1_SF  mower motor fault   (if using MOSFET/L298N, keep unconnected)
#define pinMotorMowEnable 28       // EN mower motor enable      (if using MOSFET/L298N, keep unconnected)

#define pinMotorMowRpm A11

#define pinBumperLeft 39           // bumper pins
#define pinBumperRight 38

#define pinDropLeft 45           // drop pins  Dropsensor - Absturzsensor
#define pinDropRight 23          // drop pins  Dropsensor - Absturzsensor

#define pinSonarCenterTrigger 24   // ultrasonic sensor pins
#define pinSonarCenterEcho 22
#define pinSonarRightTrigger 30    
#define pinSonarRightEcho 32
#define pinSonarLeftTrigger 34         
#define pinSonarLeftEcho 36

#define pinPerimeterRight A4       // perimeter
#define pinPerimeterLeft A5

#define pinLED 13                  // LED
#define pinBuzzer 53               // Buzzer
#define pinTilt 35                 // Tilt sensor (required for TC-G158 board)
#define pinButton 51               // digital ON/OFF button
#define pinBatteryVoltage A2       // battery voltage sensor
#define pinBatterySwitch 4         // battery-OFF switch   
#define pinChargeVoltage A9        // charging voltage sensor
#define pinChargeCurrent A8        // charge current sensor
#define pinChargeRelay 50          // charge relay
#define pinRemoteMow 12            // remote control mower motor
#define pinRemoteSteer 11          // remote control steering 
#define pinRemoteSpeed 10          // remote control speed
#define pinRemoteSwitch 52         // remote control switch
#define pinVoltageMeasurement A7   // test pin for your own voltage measurements

#define pinOdometryLeft DAC0     // left odometry sensor
#define pinOdometryLeft2 DAC1    // left odometry sensor (optional two-wire)
#define pinOdometryRight CANRX   // right odometry sensor  
#define pinOdometryRight2 CANTX  // right odometry sensor (optional two-wire)  

#define pinLawnFrontRecv 40        // lawn sensor front receive
#define pinLawnFrontSend 41        // lawn sensor front sender 
#define pinLawnBackRecv 42         // lawn sensor back receive
#define pinLawnBackSend 43         // lawn sensor back sender 

#define pinUserSwitch1 46          // user-defined switch 1
#define pinUserSwitch2 47          // user-defined switch 2 -> diBumperSensor Own Bumper Duino
#define pinUserSwitch3 48          // user-defined switch 3 -> diNearObsacleSensor Own Sonar Sensor connected to own Bumper Duino
//bber2
#define pinDHT 49     
//#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321                 
//-----
#define pinRain 44                 // rain sensor




byte DS1307_ADDRESS = B1101000;
byte AT24CX_ADDRESS = B1010000;

// Serial
BufferSerial pc(Serial, 1);
BufferSerial wan(Serial1,1); //Used serial 2 to Receive. Sabertoothdriver sends on this line.
BufferSerial bt(Serial2, 1);
BufferSerial serialGPS(Serial3, 1);
BufferSerial nativeUSB(SerialUSB, 1); //communication with raspberry pi

BufferSerial *debug = &pc;
//BufferSerial *debug = &bt;
//BufferSerial &perRX = per;
//BufferSerial &sabertoothTX = per;

TErrorHandler errorHandler;

// PWM Object
PinManager PinMan;
ADCManager ADCMan;



// Create Motordriver Object. Used serial 2 to Sent in order nothin is received
// Sabertooth motordriver(129, Serial1);
// Sabertooth mowMotorDriver(128, Serial1);

//SRF08 rangeMod1(SRF08_SDA_Pin, SRF08_SCL_Pin, 0xE2); //SRF08 ranging module 1 (PinName SDA, PinName SCL, int i2cAddress)


AnalogIn aiBATVOLT(pinBatteryVoltage);
AnalogIn aiCHARGEVOLTAGE(pinChargeVoltage);
AnalogIn aiCHARGECURRENT(pinChargeCurrent);
AnalogIn aiRandomIn(pinVoltageMeasurement); // Pin für die srand Funktion


DigitalOut doChargeEnable(pinChargeRelay);
DigitalOut doBatteryOffSwitch(pinBatterySwitch);



DigitalOut doBuzzer(pinBuzzer);
DigitalOut doMyLED(pinLED);

DigitalInOut dioDHT(pinDHT, INPUT);

DigitalIn  diPinRain(pinRain, false);

// left wheel motor
DigitalOut doMotorEnable(pinMotorEnable);
PwmOut     pwmMotorLeft(pinMotorLeftPWM, PinMan);
DigitalOut doMotorLeftDir(pinMotorLeftDir);
AnalogIn   aiMotorLeftCurrent(pinMotorLeftSense);
DigitalIn  diMotorLeftFault(pinMotorLeftFault, false);

// right wheel motor
PwmOut     pwmMotorRight(pinMotorRightPWM, PinMan);
DigitalOut doMotorRightDir(pinMotorRightDir);
AnalogIn   aiMotorRightCurrent(pinMotorRightSense);
DigitalIn  diMotorRightFault(pinMotorRightFault, false);

// mower motor
DigitalOut doMotorMowEnable(pinMotorMowEnable);
PwmOut     pwmMotorMowPWM(pinMotorMowPWM, PinMan);
DigitalOut doMotorMowDir(pinMotorMowDir);
AnalogIn   aiMotorMowCurrent(pinMotorMowSense);
DigitalIn  diMotorMowFault(pinMotorMowFault, false);


// Orignal Bumper connectors
// Both inputs are low active!
DigitalIn diBumperL(pinBumperLeft, true);
DigitalIn diBumperR(pinBumperRight, true);

// My own BumperDuino and sonar Sensor
// Both inputs are low active!
DigitalIn  diNearObsacleSensor(pinUserSwitch3, false);
DigitalIn  diBumperSensor(pinUserSwitch2, false);

// odometry
DigitalIn diEncLA(pinOdometryLeft, false);
//DigitalIn diEncLB(ENCODERLEFT_B_Pin, true);
DigitalIn diEncRA(pinOdometryRight, false);
//DigitalIn diEncRB(ENCODERRIGTH_B_Pin, true);


AnalogIn aiCoilLeft(pinPerimeterLeft);
AnalogIn aiCoilRight(pinPerimeterRight);




CRotaryEncoder encoderL(diEncLA);
CRotaryEncoder encoderR(diEncRA);

MC33926Wheels motorDriver(encoderL, encoderR);
MC33926Mow mowMotorDriver;


i2cInOut i2cRTC(DS1307_ADDRESS);
i2cInOut i2cEEPROM(AT24CX_ADDRESS);


static void ISR_ML_ENC_SIGA() {
	encoderL.rise();
}

static void ISR_MR_ENC_SIGA() {
	encoderR.rise();
}


void hardwareRun() {
	ADCMan.run();
}

void hardwareSetup() {
	
	doBuzzer.setup();

	pc.begin(CONF_PC_SERIAL_SPEED);
	wan.begin(CONF_WAN_SERIAL_SPEED);
	bt.begin(CONF_BT_SERIAL_SPEED);
	serialGPS.begin(CONF_GPS_SERIAL_SPEED);
	nativeUSB.begin(CONF_NATIVE_USB_SPEED);

	// From her on errorHandler is working

	delay(2000); // wait for motordriver, serial and SRF08 ready

	errorHandler.setInfo(F("HardwareSetup started\r\n"));

	//Delete Serial Line Data
	while (pc.available()) {
		pc.getChar();
	}
	pc.flush();

	while (wan.available()) {
		wan.getChar();
	}
	wan.flush();

	while (bt.available()) {
		bt.getChar();
	}
	bt.flush();

	while (serialGPS.available()) {
		serialGPS.getChar();
	}
	serialGPS.flush();

	while (nativeUSB.available()) {
		nativeUSB.getChar();
	}
	nativeUSB.flush();

    
	errorHandler.setInfo(F("I2c reset started\r\n"));
	i2cInOut::I2C_reset();
	Wire.begin();
	if (errorHandler.isErrorActive()) {
		errorHandler.setInfo(F("I2C bus error.Could not clear\r\n"));
		errorHandler.printError();
	}
	else {
		errorHandler.setInfo(F("I2c reset OK\r\n"));
	}


	analogReadResolution(12);
	PinMan.begin();
	ADCMan.begin();

	aiBATVOLT.setup();

	aiCHARGEVOLTAGE.setup();
	aiCHARGECURRENT.setup();


	aiRandomIn.setup();



	doChargeEnable.setup();
	doBatteryOffSwitch.setup();

	
	doMyLED.setup();

	dioDHT.setup();
	dioDHT.write(LOW);

	diPinRain.setup();

	diBumperL.setup();
	diBumperR.setup();

	// left wheel motor
	doMotorEnable.setup(HIGH);
	pwmMotorLeft.setup();
	doMotorLeftDir.setup();
	aiMotorLeftCurrent.setup();
	diMotorLeftFault.setup();

	// right wheel motor
	pwmMotorRight.setup();
	doMotorRightDir.setup();
	aiMotorRightCurrent.setup();
	diMotorRightFault.setup();

	// mower motor
	doMotorMowEnable.setup(HIGH);
	pwmMotorMowPWM.setup();
	doMotorMowDir.setup();
	aiMotorMowCurrent.setup();
	diMotorMowFault.setup();

	// Bumper and sonar Sensor
	diNearObsacleSensor.setup();
	diBumperSensor.setup();

	// odometry
	diEncLA.setup();
	//DigitalIn diEncLB.setup();
	diEncRA.setup();
	//DigitalIn diEncRB.setup();

	aiCoilLeft.setup(512);
	aiCoilRight.setup(512);

	i2cRTC.setup();
	i2cRTC.I2C_scan();




	doBatteryOffSwitch = HIGH;  // keep battery switched ON
	
								// Postprocessing of date
	if (CONF_LEFT_ENCODER_INVERSE) {
		encoderL.isReversed();
		errorHandler.setInfoNoLog(F("Set L encoder reverse\r\n"));
	}
	else {
		encoderL.isNotReversed();
		errorHandler.setInfoNoLog(F("Set L encoder not reverse\r\n"));
	}

	if (CONF_RIGHT_ENCODER_INVERSE) {
		encoderR.isReversed();
		errorHandler.setInfoNoLog(F("Set R encoder reverse\r\n"));
	}
	else {
		encoderR.isNotReversed();
		errorHandler.setInfoNoLog(F("Set R encoder not reverse\r\n"));
	}



	attachInterrupt(pinOdometryLeft, ISR_ML_ENC_SIGA, CHANGE);
	attachInterrupt(pinOdometryRight, ISR_MR_ENC_SIGA, CHANGE);
	PinMan.setDebounce(pinOdometryLeft, 100);  // reject spikes shorter than usecs on pin
	PinMan.setDebounce(pinOdometryRight, 100);  // reject spikes shorter than usecs on pin	

	motorDriver.resetFault(true);
	mowMotorDriver.resetFault(true);
	
}

