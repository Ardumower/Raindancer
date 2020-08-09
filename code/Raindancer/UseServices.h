#ifndef _USESERVICES_h
#define _USESERVICES_h

#pragma once


#include "hardware.h"

#include "closedloopcontrol.h"
#include "positioncontrol.h"
#include "motor.h"
#include "perimeter.h"
#include "batterySensor.h"
#include "rainSensor.h"
#include "motorSensor.h"
#include "mowmotorSensor.h"
#include "rangeSensor.h"
#include "bumperSensor.h"
#include "chargeSystem.h"
#include "printSensordata.h"
#include "rtc.h"
#include "EEPROM.h"
#include "buzzer.h"
#include "shutdown.h"
#include "DHT.h"
#include "gps.h"

#include "mowclosedloopcontrol.h"
#include "errorhandler.h"



extern Thread srvHal;
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
// Rain Sensor
extern TrainSensor srvRainSensor;
// Rain sensor
extern TmotorSensor srvMotorSensorL;
extern TmotorSensor srvMotorSensorR;

// Messung des Mähmotorstroms
extern TMowMotorSensor srvMowMotorSensor;
// SRF08 Range Sensor Messung der Entfernung
extern TrangeSensor srvRangeSensor;
// Bumper Sensor
extern TbumperSensor srvBumperSensor;
// Charge System
extern TchargeSystem srvChargeSystem;
// Print Sensordata for processing
extern Thread srvProcessingSensorData;
// Real time clock
extern Trtc srvRtc;
// EEPROM will not insert in thread controller
extern TEEPROM srvEeprom;
// Buzzer
extern BuzzerClass srvBuzzer;
// Shutdown service
extern TShutdown srvShutdown;
// DHT Temperature sensor
extern TDHT srvDht;
//GPS Service
extern Tgps srvGps;


#endif