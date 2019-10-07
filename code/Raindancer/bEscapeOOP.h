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
#ifndef BH_ESCAPEOOP_H
#define BH_ESCAPEOOP_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "BehaviourTree.h"

class TsetDD_FORWARD: public Action    // Each task will be a class (derived from Node of course).
{
private:

public:

    TsetDD_FORWARD()  {}

    virtual void onInitialize(Blackboard& bb) {
    }

    virtual NodeStatus onUpdate(Blackboard& bb) {
        bb.driveDirection = DD_FORWARD;
         sprintf(errorHandler.msg,"!03,set bb.driveDirection = DD_FORWARD\r\n");
            errorHandler.setInfo();
        return BH_SUCCESS;
    }


};









#endif

