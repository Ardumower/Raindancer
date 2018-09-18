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

#include "ui.h"

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
#include "bt.h"
#include "adcman.h"
#include "rtc.h"
#include "EEPROM.h"
#include "motorSensor.h"
#include "bGotoAreaX.h"
#include "rainSensor.h"
#include "DHT.h"
#include "shutdown.h"
#include "gps.h"


extern void executeLoop();

extern bool _controlManuel;
extern bool _printProcessingData;
extern unsigned long maxLoopTime;
//extern unsigned long lastTimeShowError;
extern TfindTriangle findTriangle;

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
// Rain Sensor
extern TrainSensor rainSensor;

extern TPreUpdateHistoryBump preUpdateHistoryBump;
extern TPreUpdateHistory preUpdateHistory;
extern TCalcAngle calcAngle;
//extern TPostUpdateHistory postUpdateHistory;
extern TRotateX rotateX;

extern Blackboard myBlackboard;
extern TBehaviour myBehaviour;
extern TlineFollow lineFollow;

extern TErrorHandler errorHandler;

extern Trtc rtc;
extern TEEPROM eeprom;
extern TDHT dht;

extern Tgps gps;


extern TmotorSensor motorSensorL;
extern TmotorSensor motorSensorR;

extern void FreeMem(void);

//xdes1
extern TShutdown shutdown;

bool checkManualMode()
{
  if (_controlManuel != true)
  {
    errorHandler.setInfoNoLog(F("!03,NEED TO BE IN MANUAL MODE!\r\n"));
    return false;
  }

  return true;
}
/********************************************************************************
********************************************************************************
**  functions for user commands
**  each function represents one user command
********************************************************************************
*********************************************************************************/

void cmd_help(int arg_cnt, char **args)
{
  unsigned long wait;

  errorHandler.setInfoNoLog(F("Raindancer interface emulator\r\n"));
  errorHandler.setInfoNoLog(F("=============================\r\n"));
  errorHandler.setInfoNoLog(F("Available debug commands: (lines end with CRLF or '\\r')\r\n"));
  errorHandler.setInfoNoLog(F("H will print this help message again\r\n"));
  wait = millis();
  while (millis() - wait < 100) executeLoop();

  errorHandler.setInfoNoLog(F("hello print hello message\r\n"));
  errorHandler.setInfoNoLog(F("args,1,2,3 show args 1 2 3\r\n"));
  wait = millis();
  while (millis() - wait < 100) executeLoop();

  errorHandler.setInfoNoLog(F("\r\n=== MODE SELECTION ===\r\n"));
  errorHandler.setInfoNoLog(F("A        //automatic control over actuators\r\n"));
  errorHandler.setInfoNoLog(F("M        //manual control over actuators\r\n"));
  errorHandler.setInfoNoLog(F("area,12  //drive 12m at perimeter and begin mowing\r\n"));
  errorHandler.setInfoNoLog(F("gohome   //drive to docking station. Call again to deactivate\r\n"));
  errorHandler.setInfoNoLog(F("tpt      //test perimeter tracking to dock. Mower stands on perimeter\r\n"));

  errorHandler.setInfoNoLog(F("poweroff  //shutdown the sytem\r\n"));


  //errorHandler.setInfoNoLog(F("rh,3    //restores 3 drive directions of the history\r\n"));

  wait = millis();
  while (millis() - wait < 100) executeLoop();

  errorHandler.setInfoNoLog(F("\r\n=== ERROR HANDLING ===\r\n"));
  errorHandler.setInfoNoLog(F("error //show errormessage\r\n"));
  errorHandler.setInfoNoLog(F("reset //reset error and motor faults\r\n"));
  wait = millis();
  while (millis() - wait < 100) executeLoop();


  errorHandler.setInfoNoLog(F("\r\n=== BLUETOOTH ===\r\n"));
  errorHandler.setInfoNoLog(F("bt.show //try to detect BT module\r\n"));
  errorHandler.setInfoNoLog(F("bt.set  //configure BT module\r\n"));
  wait = millis();
  while (millis() - wait < 100) executeLoop();

  errorHandler.setInfoNoLog(F("\r\n=== I2C/RTC//EEPROM SERVICE ===\r\n"));
  errorHandler.setInfoNoLog(F("i2c.scan          //i2c scanner\r\n"));
  errorHandler.setInfoNoLog(F("rtc.show          //show rtc values every rtc read (10sec)\r\n"));
  errorHandler.setInfoNoLog(F("rtc.config        //show rtc service config\r\n"));
  errorHandler.setInfoNoLog(F("rtc.find          //tries to find RTC and show result\r\n"));
  errorHandler.setInfoNoLog(F("rtc.set,8,17,3,25,01,2017 //set rtc time=8:17 dayOfWeek=3 date=25.01.2017\r\n"));

  wait = millis();
  while (millis() - wait < 100) executeLoop();

  errorHandler.setInfoNoLog(F("eep.config        //show EEPROM service config\r\n"));
  errorHandler.setInfoNoLog(F("eep.u8t,10        //show uint8_t at address 10\r\n"));
  errorHandler.setInfoNoLog(F("eep.s32t,10       //show int32_t at address 10\r\n"));
  errorHandler.setInfoNoLog(F("eep.f,10          //show float at address 10\r\n"));

  wait = millis();
  while (millis() - wait < 100) executeLoop();

  errorHandler.setInfoNoLog(F("eep.set.u8t,10,7     //write value uint8_t=7 to address=10 \r\n"));
  errorHandler.setInfoNoLog(F("eep.set.s32t,10,1234 //write value int32_t=1234 to address=10 \r\n"));
  errorHandler.setInfoNoLog(F("eep.set.f,10,7.3     //write value float=7.3 to address=10 \r\n"));
  errorHandler.setInfoNoLog(F("eep.erase            //erase the eeprom\r\n"));



  wait = millis();
  while (millis() - wait < 100) executeLoop();


  errorHandler.setInfoNoLog(F("\r\n=== DRIVE MOTOR CLOSED LOOP CONTROL SERVICE ===\r\n"));
  errorHandler.setInfoNoLog(F("clc.config      //show clcL/R config\r\n"));
  errorHandler.setInfoNoLog(F("clc.enc         //show encoder values \r\n"));

  wait = millis();
  while (millis() - wait < 100) executeLoop();

  errorHandler.setInfoNoLog(F("clc.scl         //show setpoint, currentspeed left\r\n"));
  errorHandler.setInfoNoLog(F("clc.scr         //show setpoint, currentspeed right\r\n"));
  errorHandler.setInfoNoLog(F("clc.speedl      //show speed left\r\n"));
  errorHandler.setInfoNoLog(F("clc.speedr      //show speed right\r\n"));
  errorHandler.setInfoNoLog(F("clc.ser         //show call of enableXXXRamping functions\r\n"));
  wait = millis();
  while (millis() - wait < 100) executeLoop();
  errorHandler.setInfoNoLog(F("clc.v,30        //drives both motors in closed loop with speed of 30%\r\n"));
  errorHandler.setInfoNoLog(F("                //value: -100%% to 100%%\r\n"));
  errorHandler.setInfoNoLog(F("clc.s           //stop drive motors\r\n"));
  wait = millis();
  while (millis() - wait < 100) executeLoop();
  errorHandler.setInfoNoLog(F("clc.p,123.34    //sets drive motors proportional term\r\n"));
  errorHandler.setInfoNoLog(F("clc.i,123.34    //sets drive motors integral term\r\n"));

  wait = millis();
  while (millis() - wait < 100) executeLoop();
  errorHandler.setInfoNoLog(F("clc.ag,3.0,1.0  //sets agility setOutputZeroAtRPm=3.0 stopReachedThresholdAtRpm=1.0 in RPM\r\n"));
  wait = millis();
  while (millis() - wait < 100) executeLoop();
  errorHandler.setInfoNoLog(F("clc.mt,1,150    //direct motor test. run motor=1 with speed=150\r\n"));
  errorHandler.setInfoNoLog(F("                //motor: 1=L, 2=R  speed= -255 to 255\r\n"));
  errorHandler.setInfoNoLog(F("                //deactivates closed loop control\r\n"));
  wait = millis();
  while (millis() - wait < 100) executeLoop();

  errorHandler.setInfoNoLog(F("                //end test with: clc.mt,0,0\r\n"));
  errorHandler.setInfoNoLog(F("                //value < 100 will to start the motor\r\n"));

  wait = millis();
  while (millis() - wait < 100) executeLoop();

  errorHandler.setInfoNoLog(F("\r\n=== MOW MOTOR CLOSED LOOP CONTROL SERVICE ===\r\n"));
  errorHandler.setInfoNoLog(F("clcm.config       //show clcM config\r\n"));
  errorHandler.setInfoNoLog(F("clcm.speed        //show speed 0-255 \r\n"));
  errorHandler.setInfoNoLog(F("clcm.accel,2000   //set ramp factor 2000. The higher the slower the acc.\r\n"));
  wait = millis();
  while (millis() - wait < 100) executeLoop();
  errorHandler.setInfoNoLog(F("clcm.limit,200    //set speedLimit to 200  Values: 0-255\r\n"));
  errorHandler.setInfoNoLog(F("z                 //mow motor start\r\n"));
  errorHandler.setInfoNoLog(F("t                 //mow motor stop\r\n"));

  wait = millis();
  while (millis() - wait < 100) executeLoop();
  errorHandler.setInfoNoLog(F("\r\n=== POSITION CONTROL SERVICE ===\r\n"));
  errorHandler.setInfoNoLog(F("pc.config         //show pcL/R config\r\n"));
  errorHandler.setInfoNoLog(F("pc.L              //show result after pos reached\r\n"));
  wait = millis();
  while (millis() - wait < 100) executeLoop();
  errorHandler.setInfoNoLog(F("pc.R              //show result after pos reached\r\n"));
  errorHandler.setInfoNoLog(F("pc.tuneup,2.0,1.85   //stopCmBeforeTarget,addCmToTargetPosition\r\n"));
  errorHandler.setInfoNoLog(F("pc.a,60,30        //rotate wheel 60 degrees with speed 30\r\n"));
  wait = millis();
  while (millis() - wait < 100) executeLoop();
  //xdes1
  errorHandler.setInfoNoLog(F("pc.cm,40,60,30,50    //drives left wheel 40cm at 30% speed and right 60cm at 50% speed\r\n"));
  errorHandler.setInfoNoLog(F("                  //negative cm drives backward\r\n"));
  errorHandler.setInfoNoLog(F("pc.s              //stop Positioning\r\n"));
  errorHandler.setInfoNoLog(F("pc.sp             //stop Positioning at perimeter\r\n"));

  wait = millis();
  while (millis() - wait < 100) executeLoop();


  errorHandler.setInfoNoLog(F("\r\n=== MOTOR INTERFACE SERVICE ===\r\n"));
  errorHandler.setInfoNoLog(F("turnto,60,30         //turn 60 degrees right (-60=>left) with speed 30\r\n"));
  errorHandler.setInfoNoLog(F("mot.mfb,40,80        //drive motors from 40%% to 80%% \r\n"));
  errorHandler.setInfoNoLog(F("                     //end test with: mot.mfb,0,0\r\n"));
  wait = millis();
  while (millis() - wait < 100) executeLoop();
  errorHandler.setInfoNoLog(F("mot.mfsp,-60,80      //drive motors from -60%% to 80%%\r\n"));
  errorHandler.setInfoNoLog(F("                     //stops first before run to next speed\r\n"));
  errorHandler.setInfoNoLog(F("                     //end test with: mot.mfsp,0,0\r\n"));
  wait = millis();
  while (millis() - wait < 100) executeLoop();

  errorHandler.setInfoNoLog(F("mot.mpfsb,360,80     //rotate both drive motor to 360° and then -360° with 80%% speed \r\n"));
  errorHandler.setInfoNoLog(F("                     //stops first before run to next speed\r\n"));
  errorHandler.setInfoNoLog(F("                     //end test with: mot.pfsb,0,0\r\n"));

  errorHandler.setInfoNoLog(F("mot.ort,20           //overrun test. drives robot until perimeter outside reached with 20%% speed\r\n"));
  errorHandler.setInfoNoLog(F("                     //needed to determin the FF values in TOverRun class\r\n"));

  wait = millis();
  while (millis() - wait < 100) executeLoop();

  errorHandler.setInfoNoLog(F("\r\n=== MOTOR L/R/M CURRENT SERVICE ===\r\n"));
  errorHandler.setInfoNoLog(F("mot.config        //show config\r\n"));
  errorHandler.setInfoNoLog(F("mot.cur           //show drive motor current\r\n"));
  errorHandler.setInfoNoLog(F("mot.curm          //show mow motor current\r\n"));
  wait = millis();
  while (millis() - wait < 100) executeLoop();
  errorHandler.setInfoNoLog(F("mot.scalel,1.2,1.4 //calculate scalefactor motor L for measured A of 1.2. Current shown with mot.cur = 1.4\r\n"));
  errorHandler.setInfoNoLog(F("mot.scaler,1.2,1.4 //calculate scalefactor motor R for measured A of 1.2. Current shown with mot.cur = 1.4\r\n"));
  errorHandler.setInfoNoLog(F("mot.scalem,1.2,1.4 //calculate scalefactor motor M for measured A of 1.2. Current shown with mot.curm = 1.4\r\n"));
  wait = millis();
  while (millis() - wait < 100) executeLoop();

  errorHandler.setInfoNoLog(F("\r\n=== CHARGE SYSTEM SERVICE ===\r\n"));
  errorHandler.setInfoNoLog(F("charge.config     //show config\r\n"));
  errorHandler.setInfoNoLog(F("charge.show       //show charge sensors\r\n"));
  errorHandler.setInfoNoLog(F("charge.relay,1/0  //turn relay on/off\r\n"));
  wait = millis();
  while (millis() - wait < 100) executeLoop();

  errorHandler.setInfoNoLog(F("\r\n=== BATTERY SERVICE ===\r\n"));
  errorHandler.setInfoNoLog(F("bat.config //show config\r\n"));
  errorHandler.setInfoNoLog(F("bat.show   //show battery voltage\r\n"));

  errorHandler.setInfoNoLog(F("\r\n=== TEMPERATURE SERVICE ===\r\n"));
  errorHandler.setInfoNoLog(F("temp.show   //show temperature and humidity\r\n"));


  errorHandler.setInfoNoLog(F("\r\n=== RAIN SENSOR SERVICE ===\r\n"));
  errorHandler.setInfoNoLog(F("rain.config //show config\r\n"));
  errorHandler.setInfoNoLog(F("rain.show   //show sensor value\r\n"));

  wait = millis();
  while (millis() - wait < 100) executeLoop();


  errorHandler.setInfoNoLog(F("\r\n=== ADC MANAGER ===\r\n"));
  errorHandler.setInfoNoLog(F("adc.config  //show adc config\r\n"));
  errorHandler.setInfoNoLog(F("adc.samples //show adc values\r\n"));
  wait = millis();
  while (millis() - wait < 100) executeLoop();

  errorHandler.setInfoNoLog(F("\r\n=== BHT COMANDS ===\r\n"));
  errorHandler.setInfoNoLog(F("set.spiral,0/1  //0=Off 1=On\r\n"));
  errorHandler.setInfoNoLog(F("show.distance   //show distance while drving to areaX\r\n"));
  errorHandler.setInfoNoLog(F("show.rot        //show rotate values\r\n"));
  errorHandler.setInfoNoLog(F("show.hist       //show history\r\n"));
  wait = millis();
  while (millis() - wait < 100) executeLoop();

  errorHandler.setInfoNoLog(F("set.lfki,1.1    //set line follower ki\r\n"));
  errorHandler.setInfoNoLog(F("bht.tri         //show states of trinagle finding\r\n"));
  errorHandler.setInfoNoLog(F("bht.ln          //show last called node of each BHT run\r\n"));

  wait = millis();
  while (millis() - wait < 100) executeLoop();


  errorHandler.setInfoNoLog(F("\r\n=== PERIMETER SERVICE ===\r\n"));
  errorHandler.setInfoNoLog(F("per.config  //show config\r\n"));
  errorHandler.setInfoNoLog(F("per.show    //show perimeter service sensor values\r\n"));
  errorHandler.setInfoNoLog(F("per.max     //show maximum perimeter value\r\n"));
  errorHandler.setInfoNoLog(F("per.adcocl  //show adc l offset corrected\r\n"));
  wait = millis();
  while (millis() - wait < 100) executeLoop();
  errorHandler.setInfoNoLog(F("per.adcocr  //show adc r offset corrected\r\n"));
  errorHandler.setInfoNoLog(F("per.corrl   //show correlation l\r\n"));
  errorHandler.setInfoNoLog(F("per.corrr   //show correlation r\r\n"));
  errorHandler.setInfoNoLog(F("per.corrsql //show squared correlation l\r\n"));
  errorHandler.setInfoNoLog(F("per.corrsqr //show squared correlation r\r\n"));
  wait = millis();
  while (millis() - wait < 100) executeLoop();
  errorHandler.setInfoNoLog(F("per.psnrfl  //show psnr array l\r\n"));
  errorHandler.setInfoNoLog(F("per.psnrfr  //show psnr array r\r\n"));
  errorHandler.setInfoNoLog(F("per.resultl //show matched filter results l\r\n"));
  errorHandler.setInfoNoLog(F("per.resultr //show matched filter results r\r\n"));
  errorHandler.setInfoNoLog(F("per.fftl    //show matched filter translation l\r\n"));
  errorHandler.setInfoNoLog(F("per.fftr    //show matched filter translation r\r\n"));
  wait = millis();
  while (millis() - wait < 100) executeLoop();

  errorHandler.setInfoNoLog(F("\r\n=== BUMPER SERVICE ===\r\n"));
  errorHandler.setInfoNoLog(F("bumper.config //show config\r\n"));
  errorHandler.setInfoNoLog(F("bumper.show   //show bumper sensor event\r\n"));

  wait = millis();
  while (millis() - wait < 100) executeLoop();

  errorHandler.setInfoNoLog(F("\r\n=== RANGE SERVICE ===\r\n"));
  errorHandler.setInfoNoLog(F("range.config //show config\r\n"));
  errorHandler.setInfoNoLog(F("range.show   //show range sensor event\r\n"));

  wait = millis();
  while (millis() - wait < 100) executeLoop();

  errorHandler.setInfoNoLog(F("\r\n=== GPS SERVICE ===\r\n"));
  errorHandler.setInfoNoLog(F("gps.config //show config\r\n"));
  errorHandler.setInfoNoLog(F("gps.show   //show calculated gps data\r\n"));

  wait = millis();
  while (millis() - wait < 100) executeLoop();

  errorHandler.setInfoNoLog(F("\r\n=== OTHER ===\r\n"));
  errorHandler.setInfoNoLog(F("show.mem  //show free memory\r\n"));
  errorHandler.setInfoNoLog(F("show.stat //show statistic\n"));
  errorHandler.setInfoNoLog(F("h         //hide showing\r\n"));

  wait = millis();
  while (millis() - wait < 100) executeLoop();

  //xdes1
  errorHandler.setInfoNoLog(F("\r\n=== Control Center ===\r\n"));
  errorHandler.setInfoNoLog(F("set.cco,1/0  //turn output for Control Center on/off\r\n"));


}



// Closed loop control comands

void cmd_clc_setKP(int arg_cnt, char **args)
{
  float val = cmdStr2Float(args[1]);
  clcL.kp = val;
  clcR.kp = val;
}

void cmd_clc_setKI(int arg_cnt, char **args)
{
  float val = cmdStr2Float(args[1]);
  clcL.ki = val;
  clcR.ki = val;
}


void cmd_clc_show_config(int arg_cnt, char **args)
{
  clcL.showConfig();
  clcR.showConfig();
}

void cmd_clc_showSpeedL(int arg_cnt, char **args)
{
  clcL.flagShowSpeed = !clcL.flagShowSpeed;
}

void cmd_clc_showSpeedR(int arg_cnt, char **args)
{
  clcR.flagShowSpeed = !clcR.flagShowSpeed;
}


void cmd_clc_showSetpointCurSpeedL(int arg_cnt, char **args)
{
  clcL.flagShowSetpointCurrSpeed = !clcL.flagShowSetpointCurrSpeed;
}


void cmd_clc_showSetpointCurSpeedR(int arg_cnt, char **args)
{
  clcR.flagShowSetpointCurrSpeed = !clcR.flagShowSetpointCurrSpeed;
}

void cmd_clc_showCallOfEnableXRamping(int arg_cnt, char **args)
{
  clcL.flagShowEnableRamping = !clcL.flagShowEnableRamping;
  clcR.flagShowEnableRamping = !clcR.flagShowEnableRamping;
}




void cmd_clc_setSpeed(int arg_cnt, char **args)
{
  if (checkManualMode())
  {
    int val = cmdStr2Num(args[1], 10);
    clcL.setSpeed(val);
    clcR.setSpeed(val);
  }
}


void cmd_clc_setAgility(int arg_cnt, char **args)
{
  float val0 = cmdStr2Float(args[1]);
  float val1 = cmdStr2Float(args[2]);
  clcL.setOutputToZeroAtRPm = val0;
  clcL.stopReachedThresholdAtRpm = val1;
  clcR.setOutputToZeroAtRPm = val0;
  clcR.stopReachedThresholdAtRpm = val1;

}



void cmd_clc_driveStop(int arg_cnt, char **args)
{
  if (checkManualMode())
  {
    //motor.stop();
    clcL.stop();
    clcR.stop();
  }
}

void cmd_clc_showEncoder(int arg_cnt, char **args)
{
  clcL.flagShowEncoder = !clcL.flagShowEncoder;
  clcR.flagShowEncoder = !clcR.flagShowEncoder;
}

void cmd_clc_motorTest(int arg_cnt, char **args)
{
  int mot = cmdStr2Num(args[1], 10);
  int val = cmdStr2Num(args[2], 10);

  if (checkManualMode())
  {

    if (val == 0)
    {
      clcL.flagControldirect = false;
      clcR.flagControldirect = false;
      clcL.controlDirect(val);
      clcR.controlDirect(val);
      clcL.stop();
      clcR.stop();
      return;
    }
    if (mot == 1)
    {
      clcL.flagControldirect = true;
      motorDriver.resetFault(true);
      clcL.controlDirect(val);
    }
    if (mot == 2)
    {
      clcR.flagControldirect = true;
      motorDriver.resetFault(true);
      clcR.controlDirect(val);
    }

  }
}


// Runs both motors in closed loop from speedMinTest to speedMaxTest and vice versa
void cmd_motor_motorStepSpeed(int arg_cnt, char **args)
{
  if (checkManualMode())
  {

    motor.speedMinTest = cmdStr2Num(args[1], 10);
    motor.speedMaxTest = cmdStr2Num(args[2], 10);

    // Turn off test
    if (motor.speedMinTest == 0 && motor.speedMaxTest == 0)
    {
      motor.stateTest = 99;
    }
    // Turn on test
    else
    {
      motor.stateTest = 1;
      motor.flagMotorStepSpeed = true;
    }
  }
}

void cmd_motor_motorFSB(int arg_cnt, char **args)
{

  if (checkManualMode())
  {

    motor.speedMinTest = cmdStr2Num(args[1], 10);
    motor.speedMaxTest = cmdStr2Num(args[2], 10);

    // Turn off test
    if (motor.speedMinTest == 0 && motor.speedMaxTest == 0)
    {
      motor.stateTest = 99;
    }
    // Turn on test
    else
    {
      motor.stateTest = 1;
      motor.flagMotorFSB = true;
    }
  }

}


void cmd_motor_motorPFSB(int arg_cnt, char **args)
{

  if (checkManualMode())
  {

    motor.speedMinTest = cmdStr2Num(args[1], 10); // Angle
    motor.speedMaxTest = cmdStr2Num(args[2], 10); // Speed

    // Turn off test
    if (motor.speedMinTest == 0 && motor.speedMaxTest == 0)
    {
      motor.stateTest = 99;
    }
    // Turn on test
    else
    {
      motor.stateTest = 1;
      motor.flagMotorPFSB = true;

    }
  }

}


void cmd_motor_motorOverRunTest(int arg_cnt, char **args)
{

  if (checkManualMode())
  {

    motor.speedMinTest = cmdStr2Num(args[1], 10); // Angle
    // Turn off test
    if (motor.speedMinTest == 0)
    {
      motor.stateTest = 99;
    }
    // Turn on test
    else
    {
      motor.stateTest = 1;
      motor.flagMotorPerOverrun = true;

    }
  }

}

void cmd_clcM_motorTest(int arg_cnt, char **args)
{
  int mot = cmdStr2Num(args[1], 10);
  int val = cmdStr2Num(args[2], 10);
  if (checkManualMode())
  {

    if (val == 0)
    {
      clcM.enabled = true;
      mowMotorDriver.motor(1, val);
      clcM.stop();
      return;
    }

    if (mot == 1)
    {
      clcM.enabled = false;
      mowMotorDriver.resetFault(true);
      mowMotorDriver.motor(1, val);
    }

  }
}



void cmd_clcm_show_config(int arg_cnt, char **args)
{
  clcM.showConfig();
}

void cmd_clcm_showSpeed(int arg_cnt, char **args)
{
  clcM.flagShowSpeed = !clcM.flagShowSpeed;
}

void cmd_clcm_setRamp(int arg_cnt, char **args)
{
  int val = cmdStr2Num(args[1], 10);
  clcM.motorMowAccel = val;
}

void cmd_clcm_setSpeedLimit(int arg_cnt, char **args)
{
  float val = cmdStr2Float(args[1]);
  clcM.speedLimit = val;
}

void cmd_pc_setTuneup(int arg_cnt, char **args)
{
  pcL.stopCmBeforeTarget = cmdStr2Float(args[1]);
  //pcL.addCmToTargetPosition = cmdStr2Float(args[2]);

  pcR.stopCmBeforeTarget = pcL.stopCmBeforeTarget;
  //pcR.addCmToTargetPosition = pcL.addCmToTargetPosition;
}

void cmd_pos_show_config(int arg_cnt, char **args)
{
  pcL.showConfig();
  pcR.showConfig();
}

void cmd_pos_show_resultsL(int arg_cnt, char **args)
{
  pcL.flagShowResults = !pcL.flagShowResults;
}

void cmd_pos_show_resultsR(int arg_cnt, char **args)
{
  pcR.flagShowResults = !pcR.flagShowResults;
}




void cmd_goHome(int arg_cnt, char **args)
{
  myBlackboard.flagGoHome = true;
}

void cmd_testPerimeterTracking(int arg_cnt, char **args)
{
  _controlManuel = false;
  myBehaviour.reset();
  myBlackboard.setBehaviour(BH_PERITRACK);
  errorHandler.setInfoNoLog(F("Drive to dock\r\n"));
}

void cmd_driveAngle(int arg_cnt, char **args)
{
  if (checkManualMode())
  {
    float winkel = cmdStr2Float(args[1]);
    float speed = cmdStr2Float(args[2]);
    motor.rotateAngle(winkel, speed);
  }
}

void cmd_driveCM(int arg_cnt, char **args)
{
  if (checkManualMode())
  {
    float cmL = cmdStr2Float(args[1]);
    float cmR = cmdStr2Float(args[2]);
    float speedL = cmdStr2Float(args[3]);
    float speedR = cmdStr2Float(args[4]);
    motor.rotateCM(cmL, cmR, speedL, speedR);
  }
}

void cmd_turnTo(int arg_cnt, char **args)
{
  if (checkManualMode())
  {
    float winkel = cmdStr2Float(args[1]);
    float speed = cmdStr2Float(args[2]);
    motor.turnTo(winkel, speed);
  }
}

void cmd_stopPositioning(int arg_cnt, char **args)
{
  if (checkManualMode())
  {
    motor.stopPC();
  }
}

void cmd_stopPositioningAtPer(int arg_cnt, char **args)
{
  if (checkManualMode())
  {
    motor.stopPCAtPerimeter();
  }
}




void cmd_startMowMot(int arg_cnt, char **args)
{
  if (checkManualMode())
  {
    motor.mowMotStart();
    motor.M->uiMotorDisabled = false;
    motor.M->motorDisabled = false;
  }
  else
  {
    motor.M->uiMotorDisabled = false;
  }
}

void cmd_stopMowMot(int arg_cnt, char **args)
{
  if (checkManualMode())
  {
    motor.mowMotStop();
  }
  else
  {
    motor.M->uiMotorDisabled = true;
  }
}

void cmd_cntrManuel(int arg_cnt, char **args)
{
  _controlManuel = true;;
  myBlackboard.setBehaviour(BH_NONE);
  motor.stopAllMotors();
  //myBehaviour.reset();

}

void cmd_cntrAuto(int arg_cnt, char **args)
{
  if (myBlackboard.flagEnableMowing == false)
  {
    myBehaviour.reset();
    myBlackboard.setBehaviour(BH_MOW);
    _controlManuel = false;
  }

}

void cmd_cntrGotoAreaX(int arg_cnt, char **args)
{
  myBehaviour.reset();
  long distance = cmdStr2Num(args[1], 10);
  if (CONF_PASS_THROUGH_CHARGING_STATION)
  {
    myBlackboard.setBehaviour(BH_GOTOAREA);
  }
  else
  {
    myBlackboard.setBehaviour(BH_LEAVE_HEAD_CS);
  }

  _controlManuel = false;
  //must set after set behaviour, because behaviour resets the BB
  myBlackboard.areaTargetDistanceInMeter = distance;
  errorHandler.setInfoNoLog(F("Drive to area: %l\r\n"), distance);

}


/*
  void cmd_cntrRestoreHistory(int arg_cnt, char **args)
  {
    _controlManuel = false;
    //myBehaviour.reset();
    int num = cmdStr2Num(args[1], 10);
    myBlackboard.setBehaviour(BH_RESTOREHISTORY);
    //must set after set behaviour, because behaviour resets the BB
    myBlackboard.numberToRestoreHist = num;
    errorHandler.setInfoNoLog(F("Restore history: %d\r\n"), num);

  }
*/

void cmd_showBattery(int arg_cnt, char **args)
{
  //xdes1
  errorHandler.setInfoNoLog(F("Battery Voltage: %f sensorValue: %f "), batterieSensor.voltage, batterieSensor.sensorValue);
  errorHandler.setInfoNoLog(F("aiBATVOLT.read_int32() %d\r\n"), aiBATVOLT.read_int32());
  //errorHandler.setInfoNoLog(F("$batV,%f,%f,%d\r\n"), batterieSensor.voltage, batterieSensor.sensorValue);
}

void cmd_showRain(int arg_cnt, char **args)
{
  rainSensor.flagShowRainSensor = !rainSensor.flagShowRainSensor;
}

//bber2
void cmd_showTemperature(int arg_cnt, char **args)
{

  if (CONF_DISABLE_DHT_SERVICE == true)
  {
    errorHandler.setInfoNoLog(F("Temperature service deactivated\r\n"));
    return;
  }
  //xdes1
  if (checkManualMode())
  {
    float temp;
    temp = dht.readTemperature();
    errorHandler.setInfoNoLog(F("Current Temperature: %f\r\n"), temp);
    errorHandler.setInfoNoLog(F("Current Humidity: %f\r\n"), dht.readHumidity());
    errorHandler.setInfoNoLog(F("Temperature stored in service: %f,%d\r\n"), dht.getLastReadTemperature(), dht.errorCounter);
    //errorHandler.setInfoNoLog(F("$temp, %.1f,%d\r\n"), dht.getLastReadTemperature(), dht.errorCounter);
  }
  else
  {
    errorHandler.setInfoNoLog(F("Temperature stored in service: %f\r\n"), dht.getLastReadTemperature());
    //errorHandler.setInfoNoLog(F("$temp, %.1f,%d\r\n"), dht.getLastReadTemperature(), dht.errorCounter);
  }

}

void cmd_activateControlCenterOutput(int arg_cnt, char **args)
{

  int i = atoi(args[1]);
  //xdes1
  if (i == 0)
  {
    gps.flagSendToCC = false;
    dht.hide();
    batterieSensor.hide();
    _printProcessingData = false;
    errorHandler.setInfoNoLog(F("Control Center Output Off\r\n"), i);
  }
  else
  {

    if (CONF_DISABLE_DHT_SERVICE == false)
    {
      dht.show();
    }
    if (CONF_DISABLE_BATTERY_SERVICE == false)
    {
      batterieSensor.show();
    }
    if (CONF_DISABLE_GPS == false)
    {
      gps.flagSendToCC = true;
    }
    _printProcessingData = true;
    errorHandler.setInfoNoLog(F("Control Center Output On\r\n"), i);
  }
}//ENDFUNC

//----------
void cmd_showMowSensor(int arg_cnt, char **args)
{
  mowMotorSensor.showValuesOnConsole = !mowMotorSensor.showValuesOnConsole;
}


void cmd_showStatistik(int arg_cnt, char **args)
{

  errorHandler.setInfoNoLog(F("CURRENT:\r\n"));

  unsigned long time = millis() - myBlackboard.timeInMowBehaviour;
  double minutes = (double)time / 60000.0;
  errorHandler.setInfoNoLog(F("MOWTIME %fmin\r\n"), minutes);

  float ticks = (motor.L->myEncoder->getAbsTicksCounter() + motor.R->myEncoder->getAbsTicksCounter()) / 2.0f;
  float mowway = motor.getMForCounts(ticks);
  errorHandler.setInfoNoLog(F("MOWDIRVENWAY %f\r\n"), mowway);

  errorHandler.setInfoNoLog(F("ROTATIONCOUNT %d\r\n"), (int)myBlackboard.numberOfRotations);


  errorHandler.setInfoNoLog(F("SAVED:\r\n"));
  int count = eeprom.read32t(EEPADR_CHARGINGCOUNT);
  float mowtime = eeprom.readFloat(EEPADR_MOWTIME);
  mowway = eeprom.readFloat(EEPADR_MOWDIRVENWAY);
  int32_t rotations = eeprom.read32t(EEPADR_ROTATIONCOUNT);
  errorHandler.setInfoNoLog(F("CHARGINGCOUNT %d\r\n"), count);
  errorHandler.setInfoNoLog(F("MOWTIME %fm\r\n"), mowtime * 60.0f);
  errorHandler.setInfoNoLog(F("MOWTIME %fh\r\n"), mowtime);
  errorHandler.setInfoNoLog(F("MOWDIRVENWAY %fm\r\n"), mowway);
  errorHandler.setInfoNoLog(F("ROTATIONCOUNT %d\r\n"), rotations);


}


void cmd_mot_show_config(int arg_cnt, char **args)
{
  motorSensorL.showConfig();
  motorSensorR.showConfig();
  mowMotorSensor.showConfig();
}


void cmd_showMotorCurrent(int arg_cnt, char **args)
{
  motorSensorL.showValuesOnConsole = !motorSensorL.showValuesOnConsole;
  motorSensorR.showValuesOnConsole = !motorSensorR.showValuesOnConsole;
}


void cmd_motorCalculateLScale(int arg_cnt, char **args)
{
  float data = cmdStr2Float(args[1]);
  float data2 = cmdStr2Float(args[2]);
  motorSensorL.calculateScale(data, data2);
}
void cmd_motorCalculateRScale(int arg_cnt, char **args)
{
  float data = cmdStr2Float(args[1]);
  float data2 = cmdStr2Float(args[2]);
  motorSensorR.calculateScale(data, data2);
}

void cmd_motorCalculateMScale(int arg_cnt, char **args)
{
  float data = cmdStr2Float(args[1]);
  float data2 = cmdStr2Float(args[2]);
  mowMotorSensor.calculateScale(data, data2);
}



void cmd_showPerimeter(int arg_cnt, char **args)
{
  perimeterSensoren.showValuesOnConsole = !perimeterSensoren.showValuesOnConsole;
}

void cmd_showPerimeterMax(int arg_cnt, char **args)
{
  errorHandler.setInfoNoLog(F("magMax:   %ld\r\n"), perimeterSensoren.magMax);
}

void cmd_showPerAdcOffsetCorrectedL(int arg_cnt, char **args)
{
  perimeterSensoren.coilL.showADCWithoutOffset = !perimeterSensoren.coilL.showADCWithoutOffset;
}

void cmd_showPerAdcOffsetCorrectedR(int arg_cnt, char **args)
{
  perimeterSensoren.coilR.showADCWithoutOffset = !perimeterSensoren.coilR.showADCWithoutOffset;
}


void cmd_showPerCorrelationL(int arg_cnt, char **args)
{
  perimeterSensoren.coilL.showCorrelation = !perimeterSensoren.coilL.showCorrelation;
}

void cmd_showPerCorrelationR(int arg_cnt, char **args)
{
  perimeterSensoren.coilR.showCorrelation = !perimeterSensoren.coilR.showCorrelation;
}


void cmd_showPerCorrelationSQL(int arg_cnt, char **args)
{
  perimeterSensoren.coilL.showCorrelationSQ = !perimeterSensoren.coilL.showCorrelationSQ;
}

void cmd_showPerCorrelationSQR(int arg_cnt, char **args)
{
  perimeterSensoren.coilR.showCorrelationSQ = !perimeterSensoren.coilR.showCorrelationSQ;
}


void cmd_showPerPsnrFL(int arg_cnt, char **args)
{
  perimeterSensoren.coilL.showPSNRFunction = !perimeterSensoren.coilL.showPSNRFunction;
}

void cmd_showPerPsnrFR(int arg_cnt, char **args)
{
  perimeterSensoren.coilR.showPSNRFunction = !perimeterSensoren.coilR.showPSNRFunction;
}


void cmd_showPerResultL(int arg_cnt, char **args)
{
  perimeterSensoren.coilL.showValuesResults = !perimeterSensoren.coilL.showValuesResults;
}

void cmd_showPerResultR(int arg_cnt, char **args)
{
  perimeterSensoren.coilR.showValuesResults = !perimeterSensoren.coilR.showValuesResults;
}


void cmd_showFFTL(int arg_cnt, char **args)
{
  perimeterSensoren.coilL.showMatchedFilter = !perimeterSensoren.coilL.showMatchedFilter;
}

void cmd_showFFTR(int arg_cnt, char **args)
{
  perimeterSensoren.coilR.showMatchedFilter = !perimeterSensoren.coilR.showMatchedFilter;
}



void cmd_showRTC(int arg_cnt, char **args)
{
  //bber11
  //rtc.flagShowRTCRead = !rtc.flagShowRTCRead;

  //if (rtc.flagShowRTCRead)
  //{
  errorHandler.setInfoNoLog(F("millis():   %lu\r\n"), millis());

  //errorHandler.setInfoNoLog(F("Current RTC:\r\n"));  //here i suppose it's the time in the due and not updated by the rtc
  //rtc.showImmediately();

  errorHandler.setInfoNoLog(F("Read from RTC:\r\n"));
  rtc.readDS1307();
  rtc.showImmediately();
  //}
  //errorHandler.setInfoNoLog(F( "micros():   %lu\r\n",micros());
  //errorHandler.setInfoNoLog(F( "micros64(): %llu\r\n",micros64());
  //errorHandler.setInfoNoLog(F( "millis64(): %llu\r\n",millis64());
  //errorHandler.setInfoNoLog(F( "Sekunden(): %llu\r\n",micros64()/1000000ULL);
  //errorHandler.setInfoNoLog(F( "Minuten():  %llu\r\n",micros64()/1000000ULL/60ULL);
}

void cmd_showRTCfind(int arg_cnt, char **args)
{
  if (checkManualMode())
  {
    rtc.findDS1307();
  }
}

void cmd_showRTCconfig(int arg_cnt, char **args)
{
  rtc.showConfig();
}




void cmd_setRTC(int arg_cnt, char **args)
{
  if (checkManualMode())
  {
    rtc.hour = cmdStr2Num(args[1], 10);;
    rtc.minute = cmdStr2Num(args[2], 10);;
    rtc.dayOfWeek = cmdStr2Num(args[3], 10);;
    rtc.day = cmdStr2Num(args[4], 10);;
    rtc.month = cmdStr2Num(args[5], 10);;
    rtc.year = cmdStr2Num(args[6], 10);;
    rtc.showImmediately();
    errorHandler.setInfoNoLog(F("saving...\r\n"));
    rtc.setDS1307();
    errorHandler.setInfoNoLog(F("saved\r\n"));
    delay(500);
    errorHandler.setInfoNoLog(F("reading from rtc...\r\n"));
    rtc.readDS1307();
    rtc.showImmediately();
    errorHandler.setInfoNoLog(F("read\r\n"));

  }
}


void cmd_showEEPROM_Erase(int arg_cnt, char **args)
{
  eeprom.erase();
}

void cmd_showEEPROMCconfig(int arg_cnt, char **args)
{
  eeprom.showConfig();
}

void cmd_setEEPROMbyte(int arg_cnt, char **args)
{
  if (checkManualMode())
  {
    int16_t address = cmdStr2Num(args[1], 10);
    uint8_t data = cmdStr2Num(args[2], 10);

    eeprom.writeu8t(address, data);
    errorHandler.setInfoNoLog(F("saved\r\n"));
  }

}

void cmd_setEEPROM32t(int arg_cnt, char **args)
{
  if (checkManualMode())
  {
    int16_t address = cmdStr2Num(args[1], 10);
    int32_t data = cmdStr2Num(args[2], 10);

    eeprom.write32t(address, data);
    errorHandler.setInfoNoLog(F("saved\r\n"));
  }

}

void cmd_setEEPROMfloat(int arg_cnt, char **args)
{
  if (checkManualMode())
  {
    int16_t address = cmdStr2Num(args[1], 10);
    float data = cmdStr2Float(args[2]);

    eeprom.writeFloat(address, data);
    errorHandler.setInfoNoLog(F("saved\r\n"));
  }

}

void cmd_showEEPROMbyte(int arg_cnt, char **args)
{
  uint8_t data;

  if (checkManualMode())
  {
    int16_t address = cmdStr2Num(args[1], 10);

    data = eeprom.readu8t(address);
    errorHandler.setInfoNoLog(F("Value: %d\r\n"), data);

  }
}


void cmd_showEEPROM32t(int arg_cnt, char **args)
{
  int32_t data;

  if (checkManualMode())
  {
    int16_t address = cmdStr2Num(args[1], 10);

    data = eeprom.read32t(address);
    errorHandler.setInfoNoLog(F("Value: %d\r\n"), data);

  }
}

void cmd_showEEPROMfloat(int arg_cnt, char **args)
{
  float data;

  if (checkManualMode())
  {
    int16_t address = cmdStr2Num(args[1], 10);
    data = eeprom.readFloat(address);
    errorHandler.setInfoNoLog(F("Value: %f\r\n"), data);

  }
}


void cmd_showi2c(int arg_cnt, char **args)
{
  if (checkManualMode())
  {
    i2cInOut::I2C_scan();
  }
}


void cmd_range_show_config(int arg_cnt, char **args)
{
  rangeSensor.showConfig();
}

void cmd_showRange(int arg_cnt, char **args)
{
  rangeSensor.flagShowRange = !rangeSensor.flagShowRange;
}

void cmd_gps_show_config(int arg_cnt, char **args)
{
  gps.showConfig();
}

void cmd_showGPS(int arg_cnt, char **args)
{
  if (CONF_DEACTIVATE_GPS_CALCULATION)
  {
    errorHandler.setInfoNoLog(F("GPS calculation deactivated\r\n"));
  }
  else
  {
    gps.flagShowGPS = !gps.flagShowGPS;
  }
}

void cmd_bumper_show_config(int arg_cnt, char **args)
{
  bumperSensor.showConfig();
}

void cmd_showBumper(int arg_cnt, char **args)
{
  bumperSensor.flagShowBumper = !bumperSensor.flagShowBumper;
}

void cmd_showChargeSystem(int arg_cnt, char **args)
{
  chargeSystem.flagShowChargeSystem = !chargeSystem.flagShowChargeSystem;
}

void cmd_charge_show_config(int arg_cnt, char **args)
{
  chargeSystem.showConfig();
}


void cmd_bat_show_config(int arg_cnt, char **args)
{
  batterieSensor.showConfig();
}


void cmd_rain_show_config(int arg_cnt, char **args)
{
  rainSensor.showConfig();
}

void cmd_per_show_config(int arg_cnt, char **args)
{
  perimeterSensoren.showConfig();
}



void cmd_printError(int arg_cnt, char **args)
{
  if (checkManualMode())
  {
    errorHandler.print();
  }

}

void cmd_resetError(int arg_cnt, char **args)
{
  errorHandler.resetError();
  //lastTimeShowError = 0;
  motorDriver.resetFault(true);
  mowMotorDriver.resetFault(true);
  maxLoopTime = 0;
}



void cmd_showMem(int arg_cnt, char **args)
{
  FreeMem();
}


void cmd_showADCPrint(int arg_cnt, char **args)
{
  ADCMan.printInfo();
}

void cmd_showADCValues(int arg_cnt, char **args)
{
  if (checkManualMode())
  {
    ADCMan.showValuesOnConsole = !ADCMan.showValuesOnConsole;
  }
}


void cmd_showGotoAreaDistance(int arg_cnt, char **args)
{
  motor.flagShowDistance = true;
}


void cmd_showRotate(int arg_cnt, char **args)
{

  myBlackboard.flagShowRotateX = !myBlackboard.flagShowRotateX;
  errorHandler.setInfoNoLog(F("showRot\r\n"));
}

void cmd_showHistory(int arg_cnt, char **args)
{

  if (_controlManuel == true)
  {


    for (int x = 0; x < HISTROY_BUFSIZE; x++)
    {
      errorHandler.setInfoNoLog(F("============================\r\n"));
      errorHandler.setInfoNoLog(F("!05,driveDirection   %s\r\n"), enuDriveDirectionString[myBlackboard.history[x].driveDirection]);
      errorHandler.setInfoNoLog(F("!05,coilFirstOutside %s\r\n"), enuFlagCoilsOutsideString[myBlackboard.history[x].coilFirstOutside]);
      errorHandler.setInfoNoLog(F("!05,rotAngleSoll     %f\r\n"), myBlackboard.history[x].rotAngleSoll);
      errorHandler.setInfoNoLog(F("!05,flagForceRotDir  %s\r\n"), enuFlagForceRotateDirectionString[myBlackboard.history[x].flagForceRotDirection]);
      errorHandler.setInfoNoLog(F("!05,timeAdded        %d \r\n"), myBlackboard.history[x].timeAdded);
      errorHandler.setInfoNoLog(F("---\r\n"));
      errorHandler.setInfoNoLog(F("!05,distanceDriven   %f\r\n"), myBlackboard.history[x].distanceDriven);
      errorHandler.setInfoNoLog(F("!05,rotAngleIst      %f \r\n"), myBlackboard.history[x].rotAngleIst);
      errorHandler.setInfoNoLog(F("!05,restored         %d \r\n"), myBlackboard.history[x].restored);

    }

    return;
  }



  myBlackboard.flagShowHistory = !myBlackboard.flagShowHistory;
  errorHandler.setInfoNoLog(F("showHist\r\n"));
}


void cmd_showBHTTriangle(int arg_cnt, char **args)
{
  findTriangle.flagShowFindTriangleStates = !findTriangle.flagShowFindTriangleStates;
}

void cmd_showBHTShowLastNode(int arg_cnt, char **args)
{
  myBlackboard.flagBHTShowLastNode = !myBlackboard.flagBHTShowLastNode;
}



void cmd_hideShowing(int arg_cnt, char **args)
{
  clcL.flagShowSpeed = false;
  clcR.flagShowSpeed = false;
  clcL.flagShowEncoder = false;
  clcR.flagShowEncoder = false;
  mowMotorSensor.showValuesOnConsole = false;
  perimeterSensoren.showValuesOnConsole = false;
  rangeSensor.flagShowRange = false;
  gps.flagShowGPS = false;
  bumperSensor.flagShowBumper = false;
  chargeSystem.flagShowChargeSystem = false;
  motor.flagShowDistance = false;

  rainSensor.flagShowRainSensor = false;

  ADCMan.showValuesOnConsole = false;
  rtc.flagShowRTCRead = false;

  clcL.flagShowSetpointCurrSpeed = false;
  clcR.flagShowSetpointCurrSpeed = false;
  clcL.flagShowEnableRamping = false;
  clcR.flagShowEnableRamping = false;

  pcL.flagShowResults = false;
  pcR.flagShowResults = false;
  clcM.flagShowSpeed = false;
  motorSensorL.showValuesOnConsole = false;
  motorSensorR.showValuesOnConsole = false;

  perimeterSensoren.coilL.showADCWithoutOffset = false;
  perimeterSensoren.coilR.showADCWithoutOffset = false;
  perimeterSensoren.coilL.showCorrelation = false;
  perimeterSensoren.coilR.showCorrelation = false;
  perimeterSensoren.coilL.showCorrelationSQ = false;
  perimeterSensoren.coilR.showCorrelationSQ = false;
  perimeterSensoren.coilL.showPSNRFunction = false;
  perimeterSensoren.coilR.showPSNRFunction = false;
  perimeterSensoren.coilL.showValuesResults = false;
  perimeterSensoren.coilR.showValuesResults = false;
  perimeterSensoren.coilL.showMatchedFilter = false;
  perimeterSensoren.coilR.showMatchedFilter = false;

  findTriangle.flagShowFindTriangleStates = false;
  myBlackboard.flagBHTShowLastNode = CONF_bb_flagBHTShowLastNode;
  myBlackboard.flagShowRotateX = false;
  myBlackboard.flagShowHistory = false;

  errorHandler.setInfoNoLog(F("HIDE\r\n"));

}


void cmd_setLineFollowerKi(int arg_cnt, char **args)
{

  lineFollow.Ki = cmdStr2Float(args[1]);
  errorHandler.setInfoNoLog(F("KI: %f\r\n"), lineFollow.Ki);

}

void cmd_setChargeRelay(int arg_cnt, char **args)
{
  int i = cmdStr2Num(args[1], 10);

  if (i == 0)
  {
    chargeSystem.deactivateRelay();
    errorHandler.setInfoNoLog(F("Relay disabled i: %d\r\n"), i);
  }
  else
  {
    chargeSystem.activateRelay();
    errorHandler.setInfoNoLog(F("Relay enabled: %d\r\n"), i);
  }
}





void cmd_checkBTModule(int arg_cnt, char **args)
{
  if (checkManualMode())
  {
    BluetoothConfig bt;
    bt.checkModule(true);
  }
}

void cmd_setBTBodule(int arg_cnt, char **args)
{
  if (checkManualMode())
  {
    BluetoothConfig bt;
    bt.setParams(F("Ardumower"), 1234, CONF_BT_SERIAL_SPEED, true);
  }
}

void cmd_setCruiseSpiral(int arg_cnt, char **args)
{
  int i = cmdStr2Num(args[1], 10);

  if (i == 0)
  {
    myBlackboard.flagCruiseSpiral = false;
    errorHandler.setInfoNoLog(F("CruiseSpiral disabled i: %d\r\n"), i);
  }
  else
  {
    myBlackboard.flagCruiseSpiral = true;
    errorHandler.setInfoNoLog(F("CruiseSpiral enabled: %d\r\n"), i);
  }
}


//xdes1
void cmd_PowerOff(int arg_cnt, char **args)
{
  shutdown.enabled = true;
}

// Print "hello world" when called from the command line.
//
// Usage:
// hello
void cmd_hello(int arg_cnt, char **args)
{
  errorHandler.setInfoNoLog(F("Hello world.\r\n"));
}
// Display the contents of the args string array.
//
// Usage:
// args 12 34 56 hello gothic baby
//
// Will display the contents of the args array as a list of strings
// Output:
// Arg 0: args
// Arg 1: 12
// Arg 2: 34
// Arg 3: 56
// Arg 4: hello
// Arg 5: gothic
// Arg 6: baby
void cmd_arg_display(int arg_cnt, char **args)
{
  for (int i = 0; i < arg_cnt; i++)
  {
    errorHandler.setInfoNoLog(F("Arg %i: %s\r\n"), i, args[i]);
  }
}

/********************************************************************************
********************************************************************************
**  setup function for user commands
**
********************************************************************************
*********************************************************************************/


void cmd_setup()
{


  cmdAdd((char *)"hello", cmd_hello);

  cmdAdd((char *)"args", cmd_arg_display);
  cmdAdd((char *)"H", cmd_help);

  //xdes1
  cmdAdd((char *)"poweroff", cmd_PowerOff);


  // Mode Selection
  cmdAdd((char *)"M", cmd_cntrManuel);
  cmdAdd((char *)"A", cmd_cntrAuto);
  cmdAdd((char *)"area", cmd_cntrGotoAreaX);
  cmdAdd((char *)"gohome", cmd_goHome);
  cmdAdd((char *)"tpt", cmd_testPerimeterTracking);
  //cmdAdd((char *)"rh", cmd_cntrRestoreHistory);



  cmdAdd((char *)"rtc.show", cmd_showRTC);
  cmdAdd((char *)"rtc.config", cmd_showRTCconfig);
  cmdAdd((char *)"rtc.find", cmd_showRTCfind);
  cmdAdd((char *)"rtc.set", cmd_setRTC);
  cmdAdd((char *)"eep.config", cmd_showEEPROMCconfig);
  cmdAdd((char *)"eep.u8t", cmd_showEEPROMbyte);
  cmdAdd((char *)"eep.s32t", cmd_showEEPROM32t);
  cmdAdd((char *)"eep.f", cmd_showEEPROMfloat);
  cmdAdd((char *)"eep.set.u8t", cmd_setEEPROMbyte);
  cmdAdd((char *)"eep.set.s32t", cmd_setEEPROM32t);
  cmdAdd((char *)"eep.set.f", cmd_setEEPROMfloat);
  cmdAdd((char *)"eep.erase", cmd_showEEPROM_Erase);



  // closed loop control service
  //------------------------------
  cmdAdd((char *)"clc.config", cmd_clc_show_config);
  cmdAdd((char *)"clc.enc", cmd_clc_showEncoder);


  cmdAdd((char *)"clc.scl", cmd_clc_showSetpointCurSpeedL);
  cmdAdd((char *)"clc.scr", cmd_clc_showSetpointCurSpeedR);
  cmdAdd((char *)"clc.ser", cmd_clc_showCallOfEnableXRamping);

  cmdAdd((char *)"clc.speedl", cmd_clc_showSpeedL);
  cmdAdd((char *)"clc.speedr", cmd_clc_showSpeedR);
  cmdAdd((char *)"clc.v", cmd_clc_setSpeed);
  cmdAdd((char *)"clc.s", cmd_clc_driveStop);
  cmdAdd((char *)"clc.p", cmd_clc_setKP);
  cmdAdd((char *)"clc.i", cmd_clc_setKI);

  cmdAdd((char *)"clc.ag", cmd_clc_setAgility);


  cmdAdd((char *)"clc.mt", cmd_clc_motorTest);


  // Mow Motor CLCM
  cmdAdd((char *)"clcm.config", cmd_clcm_show_config);
  cmdAdd((char *)"clcm.speed", cmd_clcm_showSpeed);
  cmdAdd((char *)"clcm.accel", cmd_clcm_setRamp);
  cmdAdd((char *)"clcm.limit", cmd_clcm_setSpeedLimit);

  cmdAdd((char *)"z", cmd_startMowMot);
  cmdAdd((char *)"t", cmd_stopMowMot);

  // Position Control Service
  //------------------------------
  cmdAdd((char *)"pc.config", cmd_pos_show_config);
  cmdAdd((char *)"pc.L", cmd_pos_show_resultsL);
  cmdAdd((char *)"pc.R", cmd_pos_show_resultsR);
  cmdAdd((char *)"pc.tuneup", cmd_pc_setTuneup);
  cmdAdd((char *)"pc.a", cmd_driveAngle);
  cmdAdd((char *)"pc.cm", cmd_driveCM);
  cmdAdd((char *)"pc.s", cmd_stopPositioning);
  cmdAdd((char *)"pc.sp", cmd_stopPositioningAtPer);





  // motor current services
  //------------------------------
  cmdAdd((char *)"mot.config", cmd_mot_show_config);
  cmdAdd((char *)"mot.cur", cmd_showMotorCurrent);
  cmdAdd((char *)"mot.curm", cmd_showMowSensor);
  cmdAdd((char *)"mot.scalel", cmd_motorCalculateLScale);
  cmdAdd((char *)"mot.scaler", cmd_motorCalculateRScale);
  cmdAdd((char *)"mot.scalem", cmd_motorCalculateMScale);
  cmdAdd((char *)"mot.mfb", cmd_motor_motorStepSpeed);
  cmdAdd((char *)"mot.mfsp", cmd_motor_motorFSB);
  cmdAdd((char *)"mot.mpfsb", cmd_motor_motorPFSB);
  cmdAdd((char *)"mot.ort", cmd_motor_motorOverRunTest);



  // charge system services
  //------------------------------
  cmdAdd((char *)"charge.config", cmd_charge_show_config);
  cmdAdd((char *)"charge.show", cmd_showChargeSystem);
  cmdAdd((char *)"charge.relay", cmd_setChargeRelay);


  // battery voltage services
  //------------------------------
  cmdAdd((char *)"bat.config", cmd_bat_show_config);
  cmdAdd((char *)"bat.show", cmd_showBattery);

  // Temperature services
  //------------------------------
  cmdAdd((char *)"temp.show", cmd_showTemperature);

  // rain sensor
  cmdAdd((char *)"rain.config", cmd_rain_show_config);
  cmdAdd((char *)"rain.show", cmd_showRain);

  // ADC manager
  //------------------------------
  cmdAdd((char *)"adc.config", cmd_showADCPrint);
  cmdAdd((char *)"adc.samples", cmd_showADCValues);


  //  Perimeter manager
  //------------------------------

  cmdAdd((char *)"per.config", cmd_per_show_config);
  cmdAdd((char *)"per.show", cmd_showPerimeter);
  cmdAdd((char *)"per.max", cmd_showPerimeterMax);
  cmdAdd((char *)"per.adcocl", cmd_showPerAdcOffsetCorrectedL);
  cmdAdd((char *)"per.adcocr", cmd_showPerAdcOffsetCorrectedR);
  cmdAdd((char *)"per.corrl", cmd_showPerCorrelationL);
  cmdAdd((char *)"per.corrr", cmd_showPerCorrelationR);
  cmdAdd((char *)"per.corrsql", cmd_showPerCorrelationSQL);
  cmdAdd((char *)"per.corrsqr", cmd_showPerCorrelationSQR);
  cmdAdd((char *)"per.psnrfl", cmd_showPerPsnrFL);
  cmdAdd((char *)"per.psnrfr", cmd_showPerPsnrFR);
  cmdAdd((char *)"per.resultl", cmd_showPerResultL);
  cmdAdd((char *)"per.resultr", cmd_showPerResultR);
  cmdAdd((char *)"per.fftl", cmd_showFFTL);
  cmdAdd((char *)"per.fftr", cmd_showFFTR);




  cmdAdd((char *)"set.lfki", cmd_setLineFollowerKi);


  cmdAdd((char *)"set.spiral", cmd_setCruiseSpiral);

  cmdAdd((char *)"turnto", cmd_turnTo);


  cmdAdd((char *)"bumper.config", cmd_bumper_show_config);
  cmdAdd((char *)"bumper.show", cmd_showBumper);

  cmdAdd((char *)"range.config", cmd_range_show_config);
  cmdAdd((char *)"range.show", cmd_showRange);

  cmdAdd((char *)"gps.config", cmd_gps_show_config);
  cmdAdd((char *)"gps.show", cmd_showGPS);

  cmdAdd((char *)"i2c.scan", cmd_showi2c);









  cmdAdd((char *)"show.mem", cmd_showMem);
  cmdAdd((char *)"show.distance", cmd_showGotoAreaDistance);
  cmdAdd((char *)"show.rot", cmd_showRotate);
  cmdAdd((char *)"show.hist", cmd_showHistory);
  cmdAdd((char *)"bht.tri", cmd_showBHTTriangle);
  cmdAdd((char *)"bht.ln", cmd_showBHTShowLastNode);

  cmdAdd((char *)"show.stat", cmd_showStatistik);





  cmdAdd((char *)"h", cmd_hideShowing);


  // Bluetooth
  cmdAdd((char *)"bt.show", cmd_checkBTModule);
  cmdAdd((char *)"bt.set", cmd_setBTBodule);
  // error handling
  cmdAdd((char *)"error", cmd_printError);
  cmdAdd((char *)"reset", cmd_resetError);

  //xdes1
  cmdAdd((char *)"set.cco", cmd_activateControlCenterOutput);  //enable: $T,1 disable: $T,0
}



