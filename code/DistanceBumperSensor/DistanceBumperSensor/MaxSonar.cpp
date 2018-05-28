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

#include "MaxSonar.h"


const int aiPinSonarL = A2;
const int aiPinSonarR = A1;
const int doPinSonarTrig = 2;

const int threshold = 100;  // 100cm

void   TMaxSonar::setup()
{
  _NearObstacle = false;
  flagShowResults = false;
  state = 0;
  pinMode(doPinSonarTrig, OUTPUT);
  digitalWrite(doPinSonarTrig, LOW);
  sample1 = 900;
  sample2 = 900;
  timeObstDetected = 0;
};

bool TMaxSonar::isNearObstacle()
{
  return _NearObstacle;
}

 // Will be called every 1 ms
void TMaxSonar::run()
{
  unsigned long currentTime;
  int  anVoltL, anVoltR;
  runned();

  switch (state) {
    case 0 :
      // Trigger MaxSonar
      digitalWrite(doPinSonarTrig, HIGH);
      startTimeMeasurement = millis();
      state = 1;
      break;      

    case 1 :
      // Set Trigger to LOW
      digitalWrite(doPinSonarTrig, LOW);
      state = 2;
      break;      
      
    case 2 :
      //Wait until results arrived and calculate distance
      currentTime = millis();
      if ( (currentTime - startTimeMeasurement) > 200) {
        anVoltL = analogRead(aiPinSonarL);
        anVoltR = analogRead(aiPinSonarR);

        sample1 = sample2;

        if(anVoltL<anVoltR){
          sample2 = anVoltL;
        } else{
          sample2 = anVoltR;
        }
        
        if (flagShowResults) {
          Serial.print(anVoltL);
          Serial.print(" ");
          Serial.println(anVoltR);
        }

        if ( sample1 < threshold && sample2 < threshold) {
          _NearObstacle = true;
          timeObstDetected = currentTime;
        } else {
           if ( (currentTime - timeObstDetected) > 1000lu){ // Latch detection for 1 sec
              _NearObstacle = false;
           }
        }
        
        state = 0;
      }
      break;
  }

}
