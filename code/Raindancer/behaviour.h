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
// behaviour.h

#ifndef _BEHAVIOUR_h
#define _BEHAVIOUR_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "helpers.h"
#include "Blackboard.h"
#include "BehaviourTree.h"



class TBehaviour 
{
private:
    Blackboard& bb;
    
    BehaviourTree behaviorTree;
  
    Selector selRoot;
    
public:

    TBehaviour(Blackboard& _myBlackboard): bb(_myBlackboard){}
    
	BehaviourTree& gettree() { return behaviorTree; };

    void setup();

    void reset();
    
    void loop();
};
#endif

