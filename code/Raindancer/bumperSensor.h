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

#ifndef BUMPER_H_INCLUDED
#define BUMPER_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "Protothread.h"
#include "helpers.h"
#include "hardware.h"


class TbumperSensor : public Protothread
{
public:
    bool flagShowBumper;
    void setup();
    virtual bool Run();
    bool isBumperActivated();
     //bber  ----------------------------
    bool isBumperActivatedLeft();
    bool isBumperActivatedRight();
    bool isBumperDuinoActivated();
    //----------------------------------
	void showConfig();
private:
    //bool _bumperActivated; don't need this' because _bumperLeftActivated and _bumperRightActivated already has this information in
     //bber -----------------------
    bool _bumperLeftActivated;
    bool _bumperRightActivated;
    //----------------------------
	bool _bumperDuinoActivated;

	bool _checkBumperLeft();
	bool _checkBumperRight();

};


#endif

