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
#ifndef BH_CHECK2_H
#define BH_CHECK2_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "BehaviourTree.h"


class TCheck2LeftCoilSignal: public Node
{
private:
    int state;
public:

    TCheck2LeftCoilSignal() {
        state = 0;
    }

    virtual void onInitialize(Blackboard& bb) {
        if( state == 0) {
            unsigned long time = millis();
            if(  time-bb.perimeterSensoren.lastTimeSignalReceivedL > CONF_PER_SIGNAL_LOST_TIME_OONECOIL) {
                state = 1;
                sprintf(errorHandler.msg,"!03,Signal Left Lost\r\n");
                errorHandler.setInfo();
            }
        }
    }

    virtual NodeStatus onUpdate(Blackboard& bb) {


        if(state != 0) {
            unsigned long time = millis();
            if( time-bb.perimeterSensoren.lastTimeSignalReceivedL < 800ul ) {
                state = 0;
                sprintf(errorHandler.msg,"!03,Signal Left Found\r\n");
                errorHandler.setInfo();
            }

            if( state == 1) {
                bb.motor.stopAllMotors();
                state = 2;
                return BH_RUNNING;
            }

            if( state == 2) {
                return BH_RUNNING;
            }

        }
        return BH_FAILURE;

    }
};

class TCheck2RightCoilSignal: public Node
{
private:
    int state;
public:

    TCheck2RightCoilSignal() {
        state = 0;
    }

    virtual void onInitialize(Blackboard& bb) {
        if( state == 0) {
            unsigned long time = millis();
            if(  time-bb.perimeterSensoren.lastTimeSignalReceivedR > CONF_PER_SIGNAL_LOST_TIME_OONECOIL) {
                state = 1;
                sprintf(errorHandler.msg,"!03,Signal Right Lost\r\n");
                errorHandler.setInfo();
            }
        }
    }

    virtual NodeStatus onUpdate(Blackboard& bb) {


        if(state != 0) {
            unsigned long time = millis();
            if( time-bb.perimeterSensoren.lastTimeSignalReceivedR < 800ul ) {
                state = 0;
                sprintf(errorHandler.msg,"!03,Signal Right Found\r\n");
                errorHandler.setInfo();
            }

            if( state == 1) {
                bb.motor.stopAllMotors();
                state = 2;
                return BH_RUNNING;
            }

            if( state == 2) {
                return BH_RUNNING;
            }

        }
        return BH_FAILURE;

    }
};



class TCheck2PerSignal: public Node
{
private:
    int state;
public:

    TCheck2PerSignal() {
        state = 0;
    }

    virtual void onInitialize(Blackboard& bb) {
        if( state == 0) {
            unsigned long time = millis();
            if(  (time-bb.perimeterSensoren.lastTimeSignalReceivedL > CONF_PER_SIGNAL_LOST_TIME) &&
                    (time-bb.perimeterSensoren.lastTimeSignalReceivedR > CONF_PER_SIGNAL_LOST_TIME)/* &&
                    (time-bb.perimeterSensoren.lastTimeSignalReceivedB > 1000)*/ ) {

                state = 1;
                sprintf(errorHandler.msg,"!03,Signal Lost\r\n");
                errorHandler.setInfo();
            }
        }
    }

    virtual NodeStatus onUpdate(Blackboard& bb) {


        if(state != 0) {
            unsigned long time = millis();

            if(  (time-bb.perimeterSensoren.lastTimeSignalReceivedL < 800ul) &&
                    (time-bb.perimeterSensoren.lastTimeSignalReceivedR < 800ul)/* &&
                    (time-bb.perimeterSensoren.lastTimeSignalReceivedB  < 800)*/  ) {
                state = 0;
                sprintf(errorHandler.msg,"!03,Signal Found\r\n");
                errorHandler.setInfo();
            }



            if( state == 1) {
                bb.motor.stopAllMotors();
                state = 2;
                return BH_RUNNING;
            }

            if( state == 2) {
                return BH_RUNNING;
            }

        }
        return BH_FAILURE;

    }
};



class TCheck2AllCoilsOutside
    : public Node
{
private:
    int state;
    unsigned long startTime;
public:

    TCheck2AllCoilsOutside() {
        state = 0;
        startTime = 0;
    }

    virtual void onInitialize(Blackboard& bb) {
        if( state == 0) {
            if( bb.perimeterSensoren.isLeftOutside() && bb.perimeterSensoren.isRightOutside() /*&& bb.perimeterSensoren.isBackOutside()*/) {
                    startTime = millis();
                    state = 1;
					/*
                    sprintf(errorHandler.msg,"!03,All Coils Outside\r\n");
                    errorHandler.setInfo();
                    sprintf(errorHandler.msg,"!03,ML: %d MR: %d\r\n",bb.perimeterSensoren.magnetudeL , bb.perimeterSensoren.magnetudeR);
                    errorHandler.setInfo();
					*/
            }
        }
    }

    virtual NodeStatus onUpdate(Blackboard& bb) {
        if( state == 1 || state == 2 ) {
            if( bb.perimeterSensoren.isLeftInside() || bb.perimeterSensoren.isRightInside() /* || bb.perimeterSensoren.isBackInside()*/) {
                //sprintf(errorHandler.msg,"!03,One Coils Inside Again\r\n");
                //errorHandler.setInfo();
                state = 0;
            }
        }

        if( state == 1) {
            // 1 Sekunde warten
            if( millis()- startTime > 1000ul) {
                bb.motor.stopAllMotors();
                state = 2;
				sprintf(errorHandler.msg, "!03,All Coils Outside\r\n");
				errorHandler.setInfo();
                return BH_RUNNING;
            }
            return BH_FAILURE;
        }

        if( state == 2) {
            return BH_RUNNING;
        }

        return BH_FAILURE;

    }
};



class TCheck2CoilSignalAreaX : public Node
{
private:
	int state;
	unsigned long waittime;
public:

	TCheck2CoilSignalAreaX() {
		state = 0;
	}

	virtual void onInitialize(Blackboard& bb) {
		if (state == 0 && bb.flagGotoAreaXFirstCall == true) {
			waittime = millis();
			state = 1;
			bb.flagGotoAreaXFirstCall =false;  // bb.flagGotoAreaXFirstCall is set to true in void Blackboard::setBehaviour(enuBehaviour b)
		}
	}


	virtual NodeStatus onUpdate(Blackboard& bb) {


		if (state != 0) {
			unsigned long time = millis();

			if (state == 1) {

				// 2 Sekunden auf Signal warten da gerade eingeschaltet wurde
				if (time - waittime < 2000) {
					return BH_RUNNING;
				}

				// Wurde Signal empfangen
				if (bb.perimeterSensoren.magnetudeR0 != 0) {
					state = 0;
					bb.flagGotoAreaXFirstCall = false;
					sprintf(errorHandler.msg, "!03,Signal Right Found SignalAreaX1\r\n");
					errorHandler.setInfo();
					return BH_FAILURE;
				}

				// Drehe 10Grad CW

				bb.cruiseSpeed = bb.CRUISE_SPEED_LOW;
				bb.driveDirection = DD_ROTATECW;
				bb.motor.turnTo(10, bb.cruiseSpeed);
				state = 2;
				return BH_RUNNING;
			}

			if (state == 2) {

				if (getTimeInNode() > 10000) {
					errorHandler.setError("!03,TCheck2CoilSignalAreaX  too long in state\r\n");
				}

				if (bb.motor.isPositionReached()) {
					state = 0;
					bb.flagGotoAreaXFirstCall = false;
					sprintf(errorHandler.msg, "!03,TCheck2CoilSignalAreaX rotated 10deg\r\n");
					errorHandler.setInfo();
					return BH_FAILURE;
				}
				return BH_RUNNING;
			}

		}
		return BH_FAILURE;

	}
};
#endif

