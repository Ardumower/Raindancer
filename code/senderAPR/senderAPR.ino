/*

      Ardumower (www.ardumower.de)
      Copyright (c) 2013-2014 by Alexander Grau
      Copyright (c) 2013-2014 by Sven Gennat
      Copyright (c) 2019      by Kai Würtz


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

/*
      Perimeter sender for Advanced Perimeter Receiver  (default sender description  http://wiki.ardumower.de/index.php?title=Perimeter_wire  )
      Requires: Perimeter sender PCB v1.0   ( https://www.marotronics.de/Perimeter-Sender-Prototyp )

*/

/*
      If the robot is out of station, the perimetersignal is activated.
      When the roboter enters the station, the sender detects this by the charging current of about 30mA (LED plus the current which runs through the voltage divider in the robot).
      The robot detects, that it is in the chargingstation by measuring 29.4V at the charging contacts.
      After entering the charging station the robot has to wait until the perimeter signal is down before starting with charging (closing the chatging relay),
      because the current could be too high for the charging device, while charging and sending the perimeter signal together.
      This 20 seconds wait time is used to settle the contacts and the robot have time to correct the position.

      For sigcode[] only use -1 and 1. If 0 is used, the motordriver will be disabled. To enable it again needs more time (ms instead of us)
      than to change pinM1_IN1 and pinM1_IN2 and the timing of the perimeter signal therefore is not correct.
*/


#include "TimerOne.h"
#include "EEPROM.h"
#include "RunningMedian.h"



#define pinEnable       5  // EN             PD5        (connect to motor driver enable)             

#define pinM1_IN1       9  // M1_IN1         PB1        (if using old L298N driver, connect this pin to L298N-IN1)
#define pinM1_IN2       2  // M1_IN2         PD2        (if using old L298N driver, connect this pin to L298N-IN2)
#define pinM1_nD2pwm    3  // M1_PWM / nD2              (if using old L298N driver, leave open)
#define pinM1_Fault     4  // M1_nSF                    motor driver fault pin

#define pinM2_IN1       7  // M2_IN1         PD7        (if using old L298N driver, connect this pin to L298N-IN1)
#define pinM2_IN2       6  // M2_IN2         PD6        (if using old L298N driver, connect this pin to L298N-IN2)
#define pinM2_nD2pwm    11 // M2_PWM / nD2              (if using old L298N driver, leave open)
#define pinM2_Fault     8  // M2_nSF                    motor driver fault pin


#define USE_PERI_FAULT        1     // use pinM1_Fault for driver fault detection? (set to '0' if not connected!)

// motor driver feedback pin (=perimeter open/close detection, used for status LED)
#define USE_PERI_CURRENT      1     // use pinM1_Feedback for perimeter current measurements? (set to '0' if not connected!)
#define pinM1_Feedback A0  // M1_FB
#define pinM2_Feedback A1  // M2_FB
#define PERI_CURRENT_MIN    0.03     // minimum Ampere for perimeter-is-closed detection 

// ---- sender automatic standby (via current sensor for charger) ----
// sender detects robot via a charging current through the charging pins
#define USE_CHG_CURRENT       1     // use charging current sensor for robot detection? (set to '0' if not connected!)
#define pinChargeCurrent     A2     // ACS712-05 current sensor OUT
#define CHG_CURRENT_MIN   0.07      // 0.008 mit MBR1045, 0.02 ohne MBR1045// minimum Ampere for charging detection
#define ROBOT_OUT_OF_STATION_TIMEOUT_MINS 720  // timeout for perimeter switch-off if robot not in station (minutes)

// ---- sender status LED ----
#define  pinLEDCharge 13  // ON: charging, BLINK: motordriver fault
#define  pinLEDGreen1 12  // ON: Interrupt is running
#define  pinLEDGreen2 A5  // ON: Perimeter current OK
#define  pinLEDRed2   A4  // ON: Perimeter current too low or perimeter broken, BLINK: motordriver fault




// code version
#define VER "APR_1.0.0"

// --------------------------------------



double chargeCurrent = 0;
double periCurrentAvg = 0;
double periCurrentMax = 0;
int faults = 0;
boolean isCharging = false;
unsigned int chargeADCZero = 0;
RunningMedian<unsigned int, 16> periCurrentMeasurements;
RunningMedian<unsigned int, 96> chargeCurrentMeasurements;

int timeSeconds = 0;

unsigned long nextTimeCalculation = 0;
unsigned long nextTimeControl = 0;
unsigned long nextTimeInfo = 0;
unsigned long nextTimeToggleLED = 0;
unsigned long nextTimeSec = 0;
int robotOutOfStationTimeMins = 0;


// must be multiple of 2 !
// http://grauonline.de/alexwww/ardumower/filter/filter.html
// "pseudonoise4_pw" signal (sender)


// a more motor driver friendly signal (sender)
//const int8_t sigcode[] = {  1, 1, -1};
//const int8_t sigcode[] = { -1, 1, 1, -1, 1, -1, -1, 1, 1, -1, -1, 1, 1, -1, -1, 1, 1, -1, -1, 1, -1, 1, 1, -1, -1, 1, 1, -1, -1, 1, 1, -1, 1, -1, -1, 1, -1, 1, 1, -1, -1, 1, 1, -1, 1, -1, -1, 1, 1, -1, -1, 1, -1, 1, 1, -1, 1, -1, -1, 1}; // 60 Zahlen

//const int8_t sigcode[] = { 1,-1,1,-1,1,1,-1,-1,1,-1,1,-1,1,-1,1,1,-1,-1,1,1,-1,-1,1,1,-1,1,-1,1,-1,-1,1,1,-1,1,-1,-1,1,-1,1,-1,1,1,-1,1,-1,1,-1,1,-1,1,-1,-1,1,-1,1,1,-1,1,-1,-1,1,1,-1,-1 }; /*Part of Bosch Indego Signal 64 Zahlen*/
//const int8_t sigcode[] = {-1, 1, 1, -1, 1, -1, -1, 1, 1, -1, -1, 1, 1, -1, -1, 1, 1, -1, -1, 1, -1, 1, 1, -1, -1, 1, 1, -1, -1, 1, 1, -1, 1, -1, -1, 1, -1, 1, 1, -1, -1, 1, 1, -1, 1, -1, -1, 1, 1, -1, -1, 1, -1, 1, 1, -1, 1, -1, -1, 1, -1, 1, -1, 1}; // 64 Zahlen

/*
      const int8_t sigcode[] = { 1, 1, -1, -1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, 1, -1, 1, 1, -1, -1, 1, 1, -1, 1, 1, -1, 1, -1, 1, -1, -1, 1, -1, -1, 1, -1, 1, 1, -1, 1, -1, 1, -1, 1, -1, 1, 1, -1, -1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1, 1, -1, -1, 1, -1,
      -1, 1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, 1, -1, 1, 1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1, 1, -1, -1, 1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1, 1, 1, -1, -1, 1
      }; // 128 Zahlen
*/

int8_t sigcode_0[102] = { -1, // Einschwingen
                        1, -1, -1, 1, -1, -1, 1, -1, 1, 1, -1, 1, -1, 1, -1, 1, -1, 1, 1, -1, -1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1, 1, -1, -1, 1, -1,
                        -1, 1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, 1, -1, 1, 1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1, 1, -1, -1, 1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1, 1, 1, -1, -1, 1,
                        -1 // Ausschwingen
                        }; //100+2 Zahlen
int8_t sigcode_1[25] = {-1, // Einschwingen
                        1,1,-1,-1,1,-1,1,-1,-1,1,-1,1,1,-1,-1,1,-1,-1,1,-1,-1,1,1,-1 }; // Use Ardumower signal as second signal



volatile boolean enableSender = false;
volatile int state = 0;
volatile int step = 0;
volatile int step1 = 0;
volatile const int maxSteps = 3*256; //1024 - 128;

#define SET_BIT(PORT, BIT)     ((PORT) |= (1 << BIT))
#define CLEAR_BIT(PORT, BIT)   ((PORT) &= ~(1 << BIT))

void timerCallback() {
      if (enableSender) {
            if (state == 0) {
                  // send signal in sigcode_0
                  if (sigcode_0[step] == 1) {
                        CLEAR_BIT(PORTB,PB1); //PORTB &= ~(1 << PB1);  //digitalWrite(pinM1_IN1, LOW);
                        SET_BIT(PORTD,PD2);   //PORTD |= (1 << PD2);   //digitalWrite(pinM1_IN2, HIGH);
                        SET_BIT(PORTD,PD5);   //PORTD |= (1 << PD5);   //digitalWrite(pinEnable, HIGH);
                  } else if (sigcode_0[step] == -1) {
                        SET_BIT(PORTB,PB1);   // PORTB  |= (1 << PB1); //digitalWrite(pinM1_IN1, HIGH);
                        CLEAR_BIT(PORTD,PD2); // PORTD &= ~(1 << PD2);  //digitalWrite(pinM1_IN2, LOW);
                        SET_BIT(PORTD,PD5);   //PORTD |= (1 << PD5);   //digitalWrite(pinEnable, HIGH);
                  } else {
                        CLEAR_BIT(PORTD,PD5); // PORTD &= ~(1 << PD5);  //// Disable PWM; digitalWrite(pinEnable, LOW);
                  }
                  step ++;
                  if (step == sizeof sigcode_0) {
                        state = 1;
                        step1 = 0;
                  }
            }//if(state=0)
            else if (state == 1) {
                  // disable output and wait until next signal should be sent this is 256+50-12. 256 is the next 2048 block an there start a little bit inside. 
                  CLEAR_BIT(PORTD,PD5); //PORTD &= ~(1 << PD5);  //// Disable PWM; digitalWrite(pinEnable, LOW);
                  SET_BIT(PORTB,PB1);
                  SET_BIT(PORTD,PD2);
                  SET_BIT(PORTD,PD7);
                  SET_BIT(PORTD,PD6);
                  step++;
                  if (step == (256+51-12)) {
                        state = 2;
                        step1 = 0;
                  }           
            }
            else if (state == 2) {
                        // send signal in sigcode
                  if (sigcode_1[step1] == 1) {
                        CLEAR_BIT(PORTD,PD7); //digitalWrite(pinM2_IN1, LOW);
                        SET_BIT(PORTD,PD6);    //digitalWrite(pinM2_IN2, HIGH);
                        SET_BIT(PORTD,PD5);    //digitalWrite(pinEnable, HIGH);
                  } else if (sigcode_1[step1] == -1) {
                        SET_BIT(PORTD,PD7);    //digitalWrite(pinM2_IN1, HIGH);
                        CLEAR_BIT(PORTD,PD6); ;  //digitalWrite(pinM2 _IN2, LOW);
                        SET_BIT(PORTD,PD5);      //digitalWrite(pinEnable, HIGH);
                  } else {
                        CLEAR_BIT(PORTD,PD5); // PORTD &= ~(1 << PD5);  //// Disable PWM; digitalWrite(pinEnable, LOW);
                  }
                  step ++;
                  step1 ++;
                  if (step1 == sizeof sigcode_1) {
                        state = 3;
                        step1 = 0;
                  }        
            }


            else if (state == 3) {
                  // disable output and wait until next signal should be sent this is 256+50-12. 256 is the next 2048 block an there start a little bit inside. 
                  CLEAR_BIT(PORTD,PD5); //PORTD &= ~(1 << PD5);  //// Disable PWM; digitalWrite(pinEnable, LOW);
                  SET_BIT(PORTB,PB1);
                  SET_BIT(PORTD,PD2);
                  SET_BIT(PORTD,PD7);
                  SET_BIT(PORTD,PD6);
                  step++;
                  if (step == ( (2*256)+51-12)) {
                        state = 4;
                        step1 = 0;
                  }           
            }
            else if (state == 4) {
                        // send signal in sigcode
                  if (sigcode_1[step1] == 1) {
                        CLEAR_BIT(PORTD,PD7); //digitalWrite(pinM2_IN1, LOW);
                        SET_BIT(PORTD,PD6);    //digitalWrite(pinM2_IN2, HIGH);
                        SET_BIT(PORTD,PD5);    //digitalWrite(pinEnable, HIGH);
                  } else if (sigcode_1[step1] == -1) {
                        SET_BIT(PORTD,PD7);    //digitalWrite(pinM2_IN1, HIGH);
                        CLEAR_BIT(PORTD,PD6); ;  //digitalWrite(pinM2 _IN2, LOW);
                        SET_BIT(PORTD,PD5);      //digitalWrite(pinEnable, HIGH);
                  } else {
                        CLEAR_BIT(PORTD,PD5); // PORTD &= ~(1 << PD5);  //// Disable PWM; digitalWrite(pinEnable, LOW);
                  }
                  step ++;
                  step1 ++;
                  if (step1 == sizeof sigcode_1) {
                        state = 5;
                        step1 = 0;
                  }        
            }


            else { // if (state == 0)
                  // disable output and wait until maxSteps is reached
                  CLEAR_BIT(PORTD,PD5); //PORTD &= ~(1 << PD5);  //// Disable PWM; digitalWrite(pinEnable, LOW);
                  SET_BIT(PORTB,PB1);
                  SET_BIT(PORTD,PD2);
                  SET_BIT(PORTD,PD7);
                  SET_BIT(PORTD,PD6);
                  
                  step++;
                  if (step == maxSteps) {
                        state = 0;
                        step = 0;
                        step1 = 0;
                  }
            } // else if (state == 0)


            SET_BIT(PORTB,PB4); //PORTB |= (1 << PB4);     //digitalWrite(pinLEDGreen1, HIGH); Informs user that interrupt sends signal.

      } else { //if (enableSender)
            digitalWrite(pinEnable, LOW);
            digitalWrite(pinLEDGreen1, LOW);
      } //else

} //void timerCallback()


void readEEPROM() {
      if (EEPROM.read(0) == 43) {
            // EEPROM data available
            chargeADCZero = (EEPROM.read(1) << 8) | EEPROM.read(2);
      } else {
            Serial.println(F("no EEPROM data found, using default calibration (INA169)"));
      }
      Serial.print(F("chargeADCZero="));
      Serial.println(chargeADCZero);
}


void calibrateChargeCurrentSensor() {
      Serial.println(F("calibrateChargeCurrentSensor (note: robot must be outside charging station!)"));
      RunningMedian<unsigned int, 32> measurements;
      for (unsigned int i = 0; i < measurements.getSize(); i++) {
            unsigned int m = analogRead(pinChargeCurrent);
            //Serial.println(m);
            measurements.add( m );
            delay(50);
      }
      float v;
      measurements.getAverage(v);
      chargeADCZero = v;
      EEPROM.write(0, 43);
      EEPROM.write(1, chargeADCZero >> 8);
      EEPROM.write(2, chargeADCZero & 255);
      Serial.println(F("calibration done"));
      readEEPROM();
}


void setup() {
      pinMode(pinM1_IN1, OUTPUT);
      pinMode(pinM1_IN2, OUTPUT);
      pinMode(pinM1_nD2pwm, OUTPUT);
      pinMode(pinM1_Feedback, INPUT);
      pinMode(pinM1_Fault, INPUT);

      pinMode(pinM2_IN1, OUTPUT);
      pinMode(pinM2_IN2, OUTPUT);
      pinMode(pinM2_nD2pwm, OUTPUT);
      pinMode(pinM2_Feedback, INPUT);
      pinMode(pinM2_Fault, INPUT);
      
      pinMode(pinEnable, OUTPUT);
      pinMode(pinLEDGreen1, OUTPUT);
      pinMode(pinLEDGreen2, OUTPUT);
      pinMode(pinLEDRed2, OUTPUT);
      pinMode(pinChargeCurrent, INPUT);

      // configure ADC reference
      // analogReference(DEFAULT); // ADC 5.0v ref
      analogReference(INTERNAL); // ADC 1.1v ref

      // sample rate 9615 Hz (19230,76923076923 / 2 => 9615.38) = 104us
      int T = 52; //1000.0 * 1000.0 / (9615.38*2);
      Serial.begin(19200);

      Serial.println(F("START"));
      Serial.print(F("Ardumower Sender "));
      Serial.println(VER);

      Serial.print(F("USE_PERI_FAULT="));
      Serial.println(USE_PERI_FAULT);
      Serial.print(F("USE_PERI_CURRENT="));
      Serial.println(USE_PERI_CURRENT);
      Serial.print(F("USE_CHG_CURRENT ="));
      Serial.println(USE_CHG_CURRENT );
      Serial.println(F("press..."));
      Serial.println(F("  1  for current sensor calibration"));
      Serial.println();

      readEEPROM();
      Serial.print(F("T="));
      Serial.println(T);
      Serial.print(F("f="));
      Serial.println(1000.0 * 1000.0 / T);
      Timer1.initialize(T);         // initialize timer1, and set period

      digitalWrite(pinM1_nD2pwm, LOW); // In case of fault, D1, /D2 or Enable must be toggled to clear the statusflag
      digitalWrite(pinM2_nD2pwm, LOW); // In case of fault, D1, /D2 or Enable must be toggled to clear the statusflag

      // Check LEDs
      digitalWrite(pinLEDGreen1, HIGH);
      digitalWrite(pinLEDGreen2, HIGH);
      digitalWrite(pinLEDCharge, HIGH);
      digitalWrite(pinLEDRed2, LOW);
      delay(800);
      digitalWrite(pinLEDGreen2, LOW);
      digitalWrite(pinLEDRed2, HIGH);
      delay(800);

      Timer1.attachInterrupt(timerCallback);
}


void checkKey() {
      if (Serial.available() > 0) {
            char ch = (char)Serial.read();
            Serial.print(F("received key="));
            Serial.println(ch);
            while (Serial.available()) Serial.read();
            switch (ch) {
                  case '1':
                        calibrateChargeCurrentSensor();
                        break;
            }
      }
}


void setLedFault() {
      digitalWrite(pinLEDGreen1, LOW);
      digitalWrite(pinLEDGreen2, LOW);

      for (int i = 0; i < 10; i++) {
            digitalWrite(pinLEDCharge, HIGH);
            digitalWrite(pinLEDRed2, HIGH);
            delay(50);
            digitalWrite(pinLEDCharge, LOW);
            digitalWrite(pinLEDRed2, LOW);
            delay(50);
      }
      digitalWrite(pinLEDCharge, LOW);
      digitalWrite(pinLEDRed2, LOW);

}


/*
      void setLedChargingPerOn() {
      // charging and perimeter on
      digitalWrite(pinLEDCharge, HIGH);
      if (periCurrentAvg >= PERI_CURRENT_MIN) {
      digitalWrite(pinLEDRed2, LOW);
      digitalWrite(pinLEDGreen2, HIGH);
      } else {
      digitalWrite(pinLEDRed2, HIGH);
      digitalWrite(pinLEDGreen2, LOW);
      }
      }
*/


void setLedChargingPerOff() {
      // charging and perimeter off
      digitalWrite(pinLEDCharge, HIGH);
      digitalWrite(pinLEDGreen2, LOW); //perimeter turned off
      digitalWrite(pinLEDRed2, LOW);   //perimeter turned off
}


void setLedPerOn() {
      // not charging and perimeter on
      digitalWrite(pinLEDCharge, LOW);
      // not charging => indicate perimeter wire state (Red=broken wire or not enough current, green = Current OK)
      if (periCurrentAvg >= PERI_CURRENT_MIN) {
            digitalWrite(pinLEDRed2, LOW);
            digitalWrite(pinLEDGreen2, HIGH);
      } else {
            digitalWrite(pinLEDRed2, HIGH);
            digitalWrite(pinLEDGreen2, LOW);
      }
}


void loop() {

      // Increase minute counter every minute: robotOutOfStationTimeMins
      if (millis() >= nextTimeSec) {
            nextTimeSec = millis() + 1000;
            timeSeconds++;
            if (timeSeconds >= 60) {
                  if (robotOutOfStationTimeMins < 1440) robotOutOfStationTimeMins++;
                  timeSeconds = 0;
            }
      }

      // Read perimeter current
      if (USE_PERI_CURRENT) {
            delay(2);
            periCurrentMeasurements.add( analogRead(pinM1_Feedback) );
      }

      // Read charging current
      if (USE_CHG_CURRENT) {
            delay(2);
            chargeCurrentMeasurements.add( analogRead( pinChargeCurrent) );
      }

      // Check key input, calculate charge current and perimeter current
      if (millis() >= nextTimeCalculation) {
            nextTimeCalculation = millis() + 500;

            checkKey();

            // Set LED off in order to check that Interrupt is running. Interrupt set LED to on
            digitalWrite(pinLEDGreen1, LOW);

            float v = 0;
            // determine charging current (Ampere)
            if (USE_CHG_CURRENT) {
                  chargeCurrentMeasurements.getAverage(v);
                  chargeCurrent = ((double)(((int)v)  - ((int)chargeADCZero))) / 1023.0 * 1.1;
                  isCharging = (abs(chargeCurrent) >= CHG_CURRENT_MIN);
                  if (isCharging) robotOutOfStationTimeMins = 0; // reset timeout
            }

            if (USE_PERI_CURRENT) {
                  v = 0;
                  // determine perimeter current (Ampere)
                  periCurrentMeasurements.getAverage(v);
                  periCurrentAvg = ((double)v) / 1023.0 * 1.1 / 0.525;   // 525 mV per amp
                  unsigned int h = 0;
                  periCurrentMeasurements.getHighest(h);
                  periCurrentMax = ((double)h) / 1023.0 * 1.1 / 0.525;   // 525 mV per amp
            }

      }

      // Determine if in station, charging or motordriver fault and activate/deactivate perimetersignal in interrupt
      if (millis() >= nextTimeControl) {
            nextTimeControl = millis() + 100;

            // Perimeter motor driver fault
            if ( USE_PERI_FAULT  && ( (digitalRead(pinM1_Fault) == LOW)   || (digitalRead(pinM2_Fault) == LOW)      ) ) {
                  // Motordriver fault
                 enableSender = false;
                 
                 if(digitalRead(pinM1_Fault) == LOW){ 
                      Serial.println(F("MC_FAULT M1"));
                 }

                 if(digitalRead(pinM2_Fault) == LOW){ 
                      Serial.println(F("MC_FAULT M2"));
                 }

                  digitalWrite(pinM1_nD2pwm, LOW); // In case of fault, D1, /D2 or Enable must be toggled to clear the statusflag
                  digitalWrite(pinM2_nD2pwm, LOW);
                  faults++;
                  
                  setLedFault();
            }
            // Is charging
            else if  ( (isCharging) || (robotOutOfStationTimeMins >= ROBOT_OUT_OF_STATION_TIMEOUT_MINS) ) {
                  // switch off perimeter
                  enableSender = false;
                  digitalWrite(pinM1_nD2pwm, LOW);
                  digitalWrite(pinM2_nD2pwm, LOW);
                  setLedChargingPerOff();
            }
            // Perimeter on
            else {
                  // switch on perimeter
                  enableSender = true;
                  digitalWrite(pinM1_nD2pwm, HIGH);
                  digitalWrite(pinM2_nD2pwm, HIGH);
                  setLedPerOn();
            }
      }


      // info output on console
      if (millis() >= nextTimeInfo) {
            nextTimeInfo = millis() + 500;
            Serial.print(F("time="));
            Serial.print(millis() / 1000);
            Serial.print(F("\tchgCurrent="));
            Serial.print(chargeCurrent, 3);
            Serial.print(F("\tchgCurrentADC="));
            float v = 0;
            chargeCurrentMeasurements.getAverage(v);
            Serial.print( v );
            Serial.print(F("\tisCharging="));
            Serial.print(isCharging);
            Serial.print(F("\tperiCurrent avg="));
            Serial.print(periCurrentAvg);
            Serial.print(F("\tmax="));
            Serial.print(periCurrentMax);
            Serial.print(F("\tfaults="));
            Serial.print(faults);
            Serial.print(F("\ttout="));
            Serial.print(robotOutOfStationTimeMins);
            Serial.println();
      }

}
