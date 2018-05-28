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
*/

#include "bumperSensor.h"

const int aiPinMPX5010DP = A0;
const int ACTIVATE_TRIGGER = 40;  // Adjust trigger sensitivity here
const int DEACTIVATE_DELTA = 20;  // Adjust restore sensitivity here

void   TbumperSensor::setup()
{
  int i;

  _bumperActivated = false;
  flagShowBumper = false;
  flagShowADCValues = false;


  for (i = 0; i < MPX5010DP_BUF_SIZE; i++) {
    int sensorValue = analogRead(aiPinMPX5010DP);
    _buf[i] =  sensorValue;
  }
};

bool TbumperSensor::isBumperActivated()
{
  return _bumperActivated;
}


void TbumperSensor::run()
{
  int i;
  int sum2 = 0;
  int sum3 = 0;
  int sum4 = 0;
  int sum5 = 0;
  int sum6 = 0;
  int sumReading = 0;

  runned();

  memcpy(&_buf[0], &_buf[1], sizeof(int) * (MPX5010DP_BUF_SIZE - 1));

  int sensorValue = analogRead(aiPinMPX5010DP);

  _buf[ MPX5010DP_BUF_SIZE - 1 ] = sensorValue;

  if (flagShowADCValues) {
    Serial.println(_buf[ MPX5010DP_BUF_SIZE - 1 ]);
  }


  // Calculate partial sum
  //=======================
  for (i = 0; i < 4; i++) {
    sum6 += _buf[i];
    //pc.printf("sum1 %f ", _buf[i]);
  }
  sum6 /= 4;


  for (i = MPX5010DP_BUF_SIZE - 32; i < MPX5010DP_BUF_SIZE - 28; i++) {
    sum5 += _buf[i];
    //pc.printf("sum4 %f ", _buf[i]);
  }
  sum5 /= 4;


  for (i = MPX5010DP_BUF_SIZE - 24; i < MPX5010DP_BUF_SIZE - 20; i++) {
    sum4 += _buf[i];
    //pc.printf("sum4 %f ", _buf[i]);
  }
  sum4 /= 4;

  for (i = MPX5010DP_BUF_SIZE - 16; i < MPX5010DP_BUF_SIZE - 12; i++) {
    sum3 += _buf[i];
    //pc.printf("sum3 %f ", _buf[i]);
  }
  sum3 /= 4;

  for (i = MPX5010DP_BUF_SIZE - 8; i < MPX5010DP_BUF_SIZE - 4; i++) {
    sum2 += _buf[i];
    //pc.printf("sum2 %f ", _buf[i]);
  }
  sum2 /= 4;

  for (i = MPX5010DP_BUF_SIZE - 3; i < MPX5010DP_BUF_SIZE; i++) {
    sumReading += _buf[i];
    //pc.printf("sumReading %f ", _buf[i]);
  }
  sumReading /= 3;



  // Check for step in function
  //=======================
  if (!_bumperActivated) {

    if (sumReading - sum2 > ACTIVATE_TRIGGER) {
      _bumperActivated = true;
      if (flagShowBumper) {
        Serial.print(F("reading-2: "));
        Serial.print(sumReading);
        Serial.print(F("-"));
        Serial.print(sum2);
        Serial.println();
      }
    }
    if (sumReading - sum3 > ACTIVATE_TRIGGER) {
      _bumperActivated = true;
      if (flagShowBumper) {
        Serial.print(F("reading-3: "));
        Serial.print(sumReading);
        Serial.print(F("-"));
        Serial.print(sum3);
        Serial.println();
      }
    }
    if (sumReading - sum4 > ACTIVATE_TRIGGER) {
      _bumperActivated = true;
      if (flagShowBumper) {
        Serial.print(F("reading-4: "));
        Serial.print(sumReading);
        Serial.print(F("-"));
        Serial.print(sum4);
        Serial.println();
      }
    }
    if (sumReading - sum5 > ACTIVATE_TRIGGER) {
      _bumperActivated = true;
      if (flagShowBumper) {
        Serial.print(F("reading-5: "));
        Serial.print(sumReading);
        Serial.print(F("-"));
        Serial.print(sum5);
        Serial.println();
      }
    }

    if (sumReading - sum6 > ACTIVATE_TRIGGER) {
      _bumperActivated = true;
      if (flagShowBumper) {
        Serial.print(F("reading-6: "));
        Serial.print(sumReading);
        Serial.print(F("-"));
        Serial.print(sum6);
        Serial.println();
      }
    }

    // calculate deactivate threshold out of sumReading
    if (_bumperActivated) {
      /* 
      _deactivateThreshold = sum6;

      if (sum2 < _deactivateThreshold)
        _deactivateThreshold = sum2;

      if (sum3 < _deactivateThreshold)
        _deactivateThreshold = sum3;

      if (sum4 < _deactivateThreshold)
        _deactivateThreshold = sum4;

      if (sum5 < _deactivateThreshold)
        _deactivateThreshold = sum5;

      _deactivateThreshold += DEACTIVATE_DELTA;
      */
      _deactivateThreshold = sumReading - DEACTIVATE_DELTA;

      if (flagShowBumper) {
        Serial.print(F("_deactivateThreshold: "));
        Serial.print(_deactivateThreshold);
        Serial.println();
        for (i = 0; i < MPX5010DP_BUF_SIZE; i++) {
          Serial.print(_buf[i]); Serial.print(F(" "));
        }
        Serial.println();
        //Serial.println(F("\r\n#######\r\n"));
      }
    }
  } //if (!_bumperActivated)
  else {
    if (sumReading < _deactivateThreshold) {
      _bumperActivated = false;
      if (flagShowBumper) {
        Serial.print(F("reading-deacThreshold:"));
        Serial.print(sumReading);
        Serial.print(F("-"));
        Serial.print(_deactivateThreshold);
        Serial.println();
        Serial.println(F("\r\n#######\r\n"));
      }
      // Reset array to new measured value because current values are bumper pressed values
      for (i = 0; i < MPX5010DP_BUF_SIZE - 2; i++) {
        _buf[i] = sumReading;
      }
    }

  }


  //pc.printf("#######\r\n");
};





