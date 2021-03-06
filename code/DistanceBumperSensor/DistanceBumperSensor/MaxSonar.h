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

#ifndef MAXSONAR_H_INCLUDED
#define MAXSONAR_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "Thread.h"



class TMaxSonar : public Thread
{
public:
    bool flagShowResults;
    void setup();
    virtual void run();
    bool isNearObstacle();
private:
    bool _NearObstacle;
    int state;
    unsigned long startTimeMeasurement;
    unsigned long timeObstDetected;
    int sample1;
    int sample2;
    

};


#endif


