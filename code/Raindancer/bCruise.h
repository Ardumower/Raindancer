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
#ifndef BH_CRUISE_H
#define BH_CRUISE_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "BehaviourTree.h"
#include "Blackboard.h"
#include "config.h"



//forums.parallax.com/discussion/154274/the-artist-robot/p5
class TCruiseSpiral : public Action    // Each task will be a class (derived from Node of course).
{
private:

	uint32_t lastTimeExecuted;

	float speedOuter;
	float speedInner;

	float waitUntilOuterCmReached;

	float radius;
	float addRadius;


public:

	TCruiseSpiral() {



	}

	virtual void onInitialize(Blackboard& bb) {

		addRadius = 17.0f / CONF_SPIRAL_SEGMENTS; // Radius that should be increased in one round divided by 16 because every 22Â° the speed and cm will calculated new. I chose 35.5/2=17 in because the cutter disc should overlap each round.

		radius = CONF_START_SPIRAL_RADIUS_CM; //22.0f; // Start with bigger radius in order not to stall the inner wheel 35.5

		/*
		sprintf(errorHandler.msg,"!03,addRadius = %f\r\n",addRadius);
		errorHandler.setInfo();
		sprintf(errorHandler.msg,"!03,radius = %f\r\n",radius);
		errorHandler.setInfo();
		*/

		calcSpeed(radius, bb);
		calcOuterCm(radius);
		bb.motor.startDistanceMeasurementSpiral();
		bb.motor.pcL->reset();
		bb.motor.pcR->reset();
		bb.motor.L->setSpeed(speedOuter);
		bb.motor.R->setSpeed(speedInner);
		bb.driveDirection = DD_SPIRAL_CW;

		bb.addHistoryEntry(bb.driveDirection, 0, 0, 0, FRD_NONE, CO_NONE);

	}

	void calcSpeed(float _r, Blackboard& bb) {
		float d = CONF_DISTANCE_BETWEEN_WHEELS_CM;

		speedOuter = mapf(_r, CONF_START_SPIRAL_RADIUS_CM, CONF_MAX_SPIRAL_RADIUS_CM, bb.CRUISE_SPEED_MEDIUM, bb.CRUISE_SPEED_HIGH);
		bb.cruiseSpeed = speedOuter;
		speedInner = speedOuter * (2 * _r - d) / (2 * _r + d);
	}

	void calcOuterCm(float _r) {
		float d = CONF_DISTANCE_BETWEEN_WHEELS_CM;
		waitUntilOuterCmReached = (PI * (2 * _r + d)) / CONF_SPIRAL_SEGMENTS; // cm for 360 degree divided by 16 to get cm for 22,5 degrees 
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {
		// get the driven distance: bb.motor.getDistanceLInCM() and check if it has driven the calculated way: waitUntilOuterCmReached
		if (bb.motor.getDistanceLInCMSpiral() > waitUntilOuterCmReached) {
			bb.motor.startDistanceMeasurementSpiral();
			radius += addRadius;
			//sprintf(errorHandler.msg,"!03,radius +=addRadius %f\r\n",radius);
			//errorHandler.setInfo();
			calcSpeed(radius, bb);
			calcOuterCm(radius);
			bb.motor.L->setSpeed(speedOuter);
			bb.motor.R->setSpeed(speedInner);
		}

		bb.history[0].distanceDriven = bb.motor.getDistanceInCM();

		if (radius > CONF_MAX_SPIRAL_RADIUS_CM) { // Stop Spiral after 150cm radius
			/*
			sprintf(errorHandler.msg,"!03,r: %f max: %f\r\n",radius, GETTF(TF_MAX_SPIRAL_RADIUS_CM));
			errorHandler.setInfo();
			errorHandler.setInfo("TCruiseSpiral end BH_SUCCESS\r\n");
			*/
			return BH_SUCCESS;
		}

		return BH_RUNNING;
	}

	virtual void onTerminate(NodeStatus status, Blackboard& bb) {
		bb.flagCruiseSpiral = false;
		bb.lastTimeSpiralStarted = millis();
		/*
		errorHandler.setInfo("TCruiseSpiral ot diable flagCruiseSpiral\r\n");

		if (status == BH_ABORTED) {
			errorHandler.setInfo("TCruiseSpiral BH_ABORTED\r\n");
		}
		*/
	}

};


class TCruiseBatLow : public Action
{
private:

public:

	TCruiseBatLow() {

	}

	virtual void onInitialize(Blackboard& bb) {

	}

	virtual NodeStatus onUpdate(Blackboard& bb) {
		bb.flagGoHome = false;
		bb.setBehaviour(BH_FINDPERIMETER);
        errorHandler.setInfo(F("!03,TCruiseBatLow bat low detected\r\n"));
		return BH_SUCCESS;
	}
};


class TCruiseRaining : public Action
    {
    private:

    public:

        TCruiseRaining()
            {
            }

        virtual void onInitialize(Blackboard& bb)
            {
            }

        virtual NodeStatus onUpdate(Blackboard& bb)
            {
            bb.flagGoHome = false;
            bb.setBehaviour(BH_FINDPERIMETER);
            errorHandler.setInfo(F("!03,TCruiseRaining rain detected\r\n"));
            bb.motor.mowMotStop();
            return BH_SUCCESS;
            }
    };

class TCruiseToPerimeter : public Action    // Each task will be a class (derived from Node of course).
{
private:
	int lastCruiseSpeed;
	bool firstTime;
public:

	TCruiseToPerimeter() : lastCruiseSpeed(0), firstTime(true) {}

	virtual void onInitialize(Blackboard& bb) {
		bb.motor.rotateCM(CONF_DRIVE_MAX_CM, bb.cruiseSpeed);
		bb.driveDirection = DD_FORWARD;
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {


		if (bb.perimeterSensoren.isLeftOutside() && bb.perimeterSensoren.isRightOutside()) {
			bb.flagCoilFirstOutside = CO_BOTH;
			bb.motor.stopPC(); //stop motor
			bb.cruiseSpeed = 0;
			sprintf(errorHandler.msg, "!03,->%s flagCoilFirstOutside = CO_BOTH\r\n", m_nodeName);
			errorHandler.setInfo();
			return BH_SUCCESS;

		}
		else if (bb.perimeterSensoren.isLeftOutside()) {
			bb.flagCoilFirstOutside = CO_LEFT;
			bb.motor.stopPC(); //stop motor
			bb.cruiseSpeed = 0;
			sprintf(errorHandler.msg, "!03,->%s flagCoilFirstOutside = CO_LEFT\r\n", m_nodeName);
			errorHandler.setInfo();
			return BH_SUCCESS;

		}
		else if (bb.perimeterSensoren.isRightOutside()) {
			bb.flagCoilFirstOutside = CO_RIGHT;
			bb.motor.stopPC(); //stop motor
			bb.cruiseSpeed = 0;
			sprintf(errorHandler.msg, "!03,->%s flagCoilFirstOutside= CO_RIGHT\r\n", m_nodeName);
			errorHandler.setInfo();
			return BH_SUCCESS;
		}

		bb.motor.changeSpeedPC(bb.cruiseSpeed);
		if (bb.motor.isPositionReached()) {
			errorHandler.setError(F("!03,TCruiseToPerimeter Position reached\r\n"));
		}
		return BH_RUNNING;
	}
};

class TCruiseStopped : public Action    // Each task will be a class (derived from Node of course).
{
private:

public:

	TCruiseStopped() {}


	virtual NodeStatus onUpdate(Blackboard& bb) {


		if (bb.motor.isPositionReached()) {
			bb.cruiseSpeed = 0;
			bb.driveDirection = DD_FORWARD;
			//errorHandler.setInfo("!03,TCruiseStopped SUCCESS\r\n" );
#if CONF_DISABLE_CHARGINGSTATION == true
			errorHandler.setError("BAT LOW\r\n");
#endif

			return BH_SUCCESS;
		}

		return BH_RUNNING;
	}
};

class TCruiseRotCW : public Action    // Each task will be a class (derived from Node of course).
{
private:
	int state;
public:

	TCruiseRotCW() {}

	virtual void onInitialize(Blackboard& bb) {



		if (bb.flagCoilFirstOutside == CO_RIGHT) {
			// Do nothing. Follow Line handles this
			state = 0;
			bb.driveDirection = DD_FORWARD;
		}
		else {
			bb.cruiseSpeed = bb.CRUISE_ROTATE_HIGH;
			bb.motor.turnTo(380, bb.cruiseSpeed);
			bb.driveDirection = DD_ROTATECW;
			state = 1;
		}
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		switch (state)
		{
		case 0:
			bb.setBehaviour(BH_PERITRACK);
			errorHandler.setInfo(F("!03,bb.setBehaviour(BH_PERITRACK);\r\n"));
			return BH_SUCCESS;
			break;
		case 1: //turn cw until both coils are inside
			if (bb.perimeterSensoren.isRightInside() && bb.perimeterSensoren.isLeftInside()) {
				state = 2;
			}
			break;
		case 2: // Wait until right coil is outside to start line following
			if (bb.perimeterSensoren.isRightOutside()) {
				bb.motor.stopPC();
				bb.cruiseSpeed = 0;
				state = 3;
			}
			break;

		case 3: // Wait until right coil is outside to start line following
			if (bb.motor.isPositionReached()) {
				bb.setBehaviour(BH_PERITRACK);
				errorHandler.setInfo(F("!03,bb.setBehaviour(BH_PERITRACK);\r\n"));
				return BH_SUCCESS;
			}
			break;

		default:
			break;
		}

		if (bb.motor.isPositionReached()) {
			errorHandler.setError(F("!03,TCruiseRotCW line not found\r\n"));
			return BH_FAILURE;
		}

		return BH_RUNNING;
	}
};


class TCruiseSpeedToMotor : public Action    // Each task will be a class (derived from Node of course).
{
private:

public:

	TCruiseSpeedToMotor(){}


	virtual void onInitialize(Blackboard& bb) {
		
		

		bb.motor.rotateCM(CONF_DRIVE_MAX_CM, bb.cruiseSpeed);
		bb.driveDirection = DD_FORWARD;

		bb.addHistoryEntry(bb.driveDirection, 0.0f, 0.0f, 0.0f, FRD_NONE, CO_NONE);



	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		bb.motor.changeSpeedPC(bb.cruiseSpeed);

		bb.history[0].distanceDriven = bb.motor.getDistanceInCM();

		if (bb.motor.isPositionReached()) {
			errorHandler.setError(F("!03,TCruiseSpeedToMotor Position reached\r\n"));
		}
		return BH_RUNNING;
	}

	virtual void onTerminate(NodeStatus status, Blackboard& bb) {
	}
};


class TCruiseObstacleNear : public Action    // Each task will be a class (derived from Node of course).
{
private:
	unsigned long lastTimeMotorDecceleration;
public:

	TCruiseObstacleNear() : lastTimeMotorDecceleration(0) {}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (bb.rangeSensor.isNearObstacle()) {


			//if (millis() - lastTimeMotorDecceleration > 100ul) { // Alle 100ms geschwindigkeit senken
				//lastTimeMotorDecceleration = millis();

			    //errorHandler.setInfo("!03,TCruiseObstacleNear\r\n");
				bb.cruiseSpeed = bb.CRUISE_SPEED_OBSTACLE;
				bb.timeCruiseSpeedSet = millis();
				return BH_SUCCESS;
				/*
				// geschwindigkeit absenken auf obstacle speed
				if (bb.cruiseSpeed > bb.CRUISE_SPEED_OBSTACLE) {
					bb.cruiseSpeed -= 6;
					if (bb.cruiseSpeed < bb.CRUISE_SPEED_OBSTACLE) {
						bb.cruiseSpeed = bb.CRUISE_SPEED_OBSTACLE;
					}
				}
				else if (bb.cruiseSpeed < bb.CRUISE_SPEED_OBSTACLE) {
						bb.cruiseSpeed = bb.CRUISE_SPEED_OBSTACLE;
				}
				bb.timeCruiseSpeedSet = millis();
				//errorHandler.setInfo("!03,TCruiseObstacleNear\r\n");
				//errorHandler.setInfo(F("NO "));
				*/
			//}

		}

		return BH_FAILURE;
	}
};

class TCruisePerimeterNear : public Action    // Each task will be a class (derived from Node of course).
{
private:
	unsigned long lastTimeMotorDecceleration;
public:

	TCruisePerimeterNear() : lastTimeMotorDecceleration(0) {}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (CONF_ACTVATE_AUTO_SPIRAL) {
			if (bb.mowMotorSensor.motorUnderHeavyLoad) {
				if (millis() - bb.lastTimeSpiralStarted > 60000ul) {
					bb.flagCruiseSpiral = true;
				}
			}
			if ((millis() - bb.lastTimeSpiralStarted > 180000ul) && bb.mowMotorSensor.checkIfUnderLoad()) { // Alle 3 Minuten Spirale starten wenn motor belastet
				bb.flagCruiseSpiral = true;
			}
		}


		if (bb.perimeterSensoren.isNearPerimeter() || bb.mowMotorSensor.motorUnderHeavyLoad) {
			//if (millis() - lastTimeMotorDecceleration > 100ul) { // Alle 100ms geschwindigkeit senken
				//lastTimeMotorDecceleration = millis();

				bb.cruiseSpeed = bb.CRUISE_SPEED_MEDIUM;
				bb.timeCruiseSpeedSet = millis();
				return BH_SUCCESS;
				/*
				// geschwindigkeit absenken auf medium speed
				if (bb.cruiseSpeed > bb.CRUISE_SPEED_MEDIUM) {
					bb.cruiseSpeed -= 6;
					if (bb.cruiseSpeed < bb.CRUISE_SPEED_MEDIUM) {
						bb.cruiseSpeed = bb.CRUISE_SPEED_MEDIUM;
					}
				}
				else if (bb.cruiseSpeed < bb.CRUISE_SPEED_MEDIUM) {
					bb.cruiseSpeed += 1;
					if (bb.cruiseSpeed < bb.CRUISE_SPEED_OBSTACLE) {
						bb.cruiseSpeed = bb.CRUISE_SPEED_OBSTACLE;
					}
					if (bb.cruiseSpeed > bb.CRUISE_SPEED_MEDIUM) {
						bb.cruiseSpeed = bb.CRUISE_SPEED_MEDIUM;
					}
				}
				bb.timeCruiseSpeedSet = millis();
				//errorHandler.setInfo("!03,TCruisePerimeterNear\r\n");
				//errorHandler.setInfo(F("NP "));
				*/
			//}

		}

		return BH_FAILURE;
	}
};


class TCruiseHighSpeed : public Action    // Each task will be a class (derived from Node of course).
{
private:
	unsigned long lastTimeMotorAcceleration;
public:

	TCruiseHighSpeed() : lastTimeMotorAcceleration(0) {}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (millis() - bb.timeCruiseSpeedSet > 1000) { // After 1 Seconds set accelerate cruise speed to high
			if (millis() - lastTimeMotorAcceleration > 100) { // Alle 100ms geschwindigkeit erhoehen
				lastTimeMotorAcceleration = millis();

				bb.cruiseSpeed = bb.CRUISE_SPEED_HIGH;

				//errorHandler.setInfo("!03,TCruiseHighSpeed %d\r\n", bb.cruiseSpeed);
				/*
				bb.cruiseSpeed += 1;
				//errorHandler.setInfo(F("CH1 "));


				if (bb.cruiseSpeed < bb.CRUISE_SPEED_OBSTACLE) {
					bb.cruiseSpeed = bb.CRUISE_SPEED_OBSTACLE;
					//errorHandler.setInfo(F("CH1 "));
				}

				if (bb.cruiseSpeed > bb.CRUISE_SPEED_HIGH) {
					bb.cruiseSpeed = bb.CRUISE_SPEED_HIGH;
					//errorHandler.setInfo(F("CH2 "));
				}
				//errorHandler.setInfo("!03,TCruiseHighSpeed\r\n");
				*/

			}
		}
		return BH_SUCCESS;
	}
};




class TCruiseStartMowMotor : public Action
{
private:
	unsigned long startTime;
public:

	TCruiseStartMowMotor() {

	}

	virtual void onInitialize(Blackboard& bb) {

		if (CONF_DISABLE_MOW_MOTOR == true) {
			return;
		}

		if (!bb.motor.isMowMotRunning()) {
			bb.motor.mowMotStart();
			bb.motor.stopCLC(); // Drivemotors must be stopped. Normally they are, but here only for safety.
			startTime = millis();
		}
		else {
			startTime = millis() - 10000ul;
		}
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (CONF_DISABLE_MOW_MOTOR == true) {
			return BH_FAILURE;
		}
		// Wait 5 Sekonds until mow motor is running full speed
		if (millis() - startTime < 5000ul) {
			return BH_RUNNING;
		}

		// Set starttime in order that if( millis()- startTime < 5000ul) will not called again. Also not when millis overrun.
		// The mow mnotor is stopped in bb.setBehaviour()
		startTime = millis() - 10000ul;
		return BH_FAILURE;

	}
};




#endif

