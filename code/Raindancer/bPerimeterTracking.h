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
#ifndef BH_FOLLOWLINE_H
#define BH_FOLLOWLINE_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "helpers.h"
#include "BehaviourTree.h"
#include "positioncontrol.h"
#include "config.h"
#include "Blackboard.h"
#include "pid.h"
#include "errorhandler.h"

class TperTrackChargingStationReached : public Action {
private:

public:

	TperTrackChargingStationReached() {
	}

	virtual void onInitialize(Blackboard& bb) {
		srvMotor.enableDefaultRamping();
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		bb.setBehaviour(BH_CHARGING);
		//debug->printf("bb.setBehaviour(BH_CHARGING);");
		return BH_SUCCESS;
	}
};



class TLFRotateCC105 : public Action {
private:

public:

	TLFRotateCC105() {

	}

	virtual void onInitialize(Blackboard& bb) {

		THistory hist = bb.getInitialisedHistoryEntry();
		hist.cruiseSpeed = bb.CRUISE_ROTATE_HIGH;
		hist.driveDirection = DD_ROTATECC;
		hist.distanceSoll = -105;
		bb.addHistoryEntry(hist);
		srvMotor.turnTo(-105, hist.cruiseSpeed);

	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (getTimeInNode() > 10000) {
			errorHandler.setError(F("!03,TFLRotateCC105 too long in state\r\n"));
		}

		if (srvMotor.isPositionReached()) {
			bb.setBehaviour(BH_FINDPERIMETER);
			return BH_SUCCESS;
		}

		return BH_RUNNING;
	}



};
/*
	class TlineFollow : public Action {
	private:
	bool waitForRightInside;
	double integral;
	unsigned long lastRun;
	unsigned long lastTransitionTime;

	public:
	double Ki;

	TlineFollow() {
		Ki = 1.1f / 1.5;
		lastRun = 0;
		integral = 0;
	}

	virtual void onInitialize(Blackboard& bb) {
		lastRun = millis() - 50ul;
		integral = 0;
		waitForRightInside = false;
		lastTransitionTime = millis();
		srvMotor.enablePerTrackRamping();
		//errorHandler.setInfoNoLog(F("TlineFollow onInitialize \r\n"));

		THistory hist = bb.getInitialisedHistoryEntry();
		hist.cruiseSpeed = bb.LINEFOLLOW_SPEED_HIGH;
		hist.driveDirection = DD_FORWARD;
		hist.distanceSoll = 40000;
		bb.addHistoryEntry(hist);
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {


		if (millis() - lastRun < 33) return BH_RUNNING;
		lastRun = millis();


		// Calculate Speed
		//============================================
		double error = srvPerSensoren.magnetudeR0; //important use magnetudeR0 here with includes 0 also
		if (error < 0) {
			error = -1;
		}
		else {
			error = 1;
		}

		integral = integral + (Ki * error);

		//debug->printf("error: %f\r\n",error);

		//Set integral to 0 if crossing the line
		if (sign0minus(error) != sign0minus(integral)) { //sign0minus => 0 belongs to minus
			integral = 0;
			lastTransitionTime = millis();
		}

		//debug->printf("trans: %lu\r\n",millis()-srvPerSensoren.perimeterLastTransitionTimeB);

		double Output = integral;

		//debug->printf("p:%f i%f d:%f o:%f\r\n",Kp * error , integral, Kd * derivate, Output);

		if (Output > bb.LINEFOLLOW_SPEED_HIGH) Output = bb.LINEFOLLOW_SPEED_HIGH;
		if (Output < -1 * bb.LINEFOLLOW_SPEED_HIGH) Output = -1 * bb.LINEFOLLOW_SPEED_HIGH;


		bb.history0.cruiseSpeed = bb.LINEFOLLOW_SPEED_HIGH;
		bb.history0.driveDirection = DD_FORWARD;

		// If left coil outside, rotate CC until right is inside

		if (waitForRightInside == true) {
			if (srvPerSensoren.isRightInside()) {
				waitForRightInside = false;
			}
		}

		// Follow line
		if (waitForRightInside == false) {
			if (error < 0.0f) { //Set Speed Outside Perimeter

				if (srvPerSensoren.isLeftOutside() == true) {
					bb.history0.cruiseSpeed = 25;
					bb.history0.driveDirection = DD_ROTATECC;
					srvMotor.L->setSpeed(-25);
					srvMotor.R->setSpeed(25);
					waitForRightInside = true;
				}
				else if ((millis() - lastTransitionTime) > 2000) { // If more than 3.5sec Outside rotate full
					bb.history0.cruiseSpeed = 25;
					bb.history0.driveDirection = DD_ROTATECC;
					srvMotor.L->setSpeed(-25);
					srvMotor.R->setSpeed(25);
				}
				else if ((millis() - lastTransitionTime) > 1500) { // If more than 2.8sec Outside rotate more aggressive
					srvMotor.L->setSpeed((bb.history0.cruiseSpeed + Output));
					srvMotor.R->setSpeed((bb.history0.cruiseSpeed + 10));
				}
				else if ((millis() - lastTransitionTime) > 1000) { // If more than 2sec Outside rotate aggressive
					srvMotor.L->setSpeed((bb.history0.cruiseSpeed + Output));
					srvMotor.R->setSpeed((bb.history0.cruiseSpeed + 5));
				}
				else {
					srvMotor.L->setSpeed((bb.history0.cruiseSpeed + Output));
					srvMotor.R->setSpeed((bb.history0.cruiseSpeed));
				}

			}
			else { //Set Speed Inside Perimeter

				if (srvPerSensoren.isLeftOutside() == true) {
					bb.history0.cruiseSpeed = 25;
					bb.history0.driveDirection = DD_ROTATECC;
					srvMotor.L->setSpeed(-25);
					srvMotor.R->setSpeed(25);
					waitForRightInside = true;
				}
				else if ((millis() - lastTransitionTime) > 1800) { // // If more than 2sec inside rotate full
					bb.history0.cruiseSpeed = 25;
					bb.history0.driveDirection = DD_ROTATECW;
					srvMotor.L->setSpeed(25);
					srvMotor.R->setSpeed(-25);
				}
				else if ((millis() - lastTransitionTime) > 1500) { // If more than 1.5sec inside rotate more aggressive
					srvMotor.L->setSpeed((bb.history0.cruiseSpeed + 10));
					srvMotor.R->setSpeed(-35);
				}
				else if ((millis() - lastTransitionTime) > 1000) { // If more than 1sec inside rotate aggressive
					srvMotor.L->setSpeed((bb.history0.cruiseSpeed + 10)); //50+10=60
					srvMotor.R->setSpeed(-25);  // -8
				}
				else {
					srvMotor.L->setSpeed((bb.history0.cruiseSpeed + 5)); //50+10=60
					srvMotor.R->setSpeed((bb.history0.cruiseSpeed - 20)); //50-25=25

				}
			}
		}

		if ((millis() - lastTransitionTime) > 12000) {
			errorHandler.setError(F("!03,TlineFollow line crossing > 12000\r\n"));
		}
		return BH_RUNNING;
	}

	virtual void onTerminate(NodeStatus status, Blackboard& bb) {
		//errorHandler.setInfoNoLog(F("TlineFollow onTerminate \r\n"));
		//srvMotor.enableDefaultRamping();
	}
	};

*/


class TlineFollow : public Action {
private:

	unsigned long lastRun;
	unsigned long lastTransitionTime;

	unsigned long timeInState;

	float Input, Output, Setpoint, InputOld;
	int speedL, speedR;
	int16_t state;



public:

	PIDPOS pid;
	bool flagShowValues;
	bool tunePID;
	float iTermLimit;
	float speedLimit;

	TlineFollow() {

		lastRun = 0;						  //0.08	
		pid.setup(&Input, &Output, &Setpoint, 0.06, 0.02, 0.005);
		pid.SetOutputLimits(-100, 170);
		pid.Int_Improvement = true;
		flagShowValues = false;
		iTermLimit = 10.0f;
		speedLimit = 110.0f;
		tunePID = false;


	}

	virtual void onInitialize(Blackboard& bb) {

		// set PID
		Output = 0;
		pid.SetMode(AUTOMATIC);
		pid.Initialize();

		InputOld = srvPerSensoren.magnetudeR0;


		lastRun = millis() - 50ul;

		lastTransitionTime = millis();
		srvMotor.enablePerTrackRamping();
		errorHandler.setInfoNoLog(F("TlineFollow onInitialize \r\n"));


		if (tunePID) {
			state = 0;
			errorHandler.setInfoNoLog(F("Set State  0 (TUNE INIT) \r\n"));

		}
		else {
			state = -2;
			errorHandler.setInfoNoLog(F("Set State -2 (INIT) \r\n"));

		}

		THistory hist = bb.getInitialisedHistoryEntry();
		hist.cruiseSpeed = Blackboard::LINEFOLLOW_SPEED_HIGH;
		hist.driveDirection = DD_FORWARD;
		hist.distanceSoll = 40000;
		bb.addHistoryEntry(hist);

	}

	virtual NodeStatus onUpdate(Blackboard& bb) {
		float PeriCoeffAccel;

		if (millis() - lastRun < 33UL) return BH_RUNNING;
		lastRun = millis();

		//============================================
		// Calculate PID
		//============================================
		// Lowpassfilter for Input
		Input = 0.6f * InputOld + 0.4f * (float)srvPerSensoren.magnetudeR0; //important use magnetudeR0 here which includes 0 also
		// Tell the pid where to go
		if (Input < 0) {
			Setpoint = -0.5f;
		}
		else {
			Setpoint = 0.5f;
		}

		pid.Compute();

		if (sign0minus(Input) != sign0minus(InputOld)) { //sign0minus => 0 belongs to minus
			lastTransitionTime = millis();
		}
		InputOld = Input;

		//============================================
		// Statemachine
		//============================================

		switch (state) {

		case -2: // setup for  linfollowing smoothly
			Output = 0;
			pid.SetMode(AUTOMATIC);
			pid.Initialize();
			timeInState = millis();
			state = -1;
			errorHandler.setInfoNoLog(F("Set State -1 dt: %ul\r\n"), timeInState);
			pid.Compute();  //compute again because we initialised the pid new
			lastTransitionTime = millis();
			break;



		case -1:  // start linfollowing smoothly

			// important, when calling this state pid.SetMode(AUTOMATIC); must be set.
			// If PID is in manual, this state will drive forward only.
			if (pid.GetMode() == MANUAL) {
				errorHandler.setError(F("TlineFollow PID not in AUTOMATIC mode state -1\r\n"));
			}

			bb.history0.driveDirection = DD_FORWARD;

			// calculate a value to reduce the agressivity of the PID for the first 3 Seconds
			PeriCoeffAccel = (3000.0f - (millis() - timeInState)) / 1000.0f;
			if (PeriCoeffAccel < 1.0f) PeriCoeffAccel = 1.0f;

			speedL = (Blackboard::LINEFOLLOW_SPEED_HIGH - (int)Output) / PeriCoeffAccel;
			speedR = (Blackboard::LINEFOLLOW_SPEED_HIGH + (int)Output) / PeriCoeffAccel;

			//*********************
			//** State transition
			//*********************
			// if the speedL reaches the limit, rotate CW. It could be, that limit must be > 100
			if (speedL > speedLimit && Setpoint > 0.0f) {
				errorHandler.setInfoNoLog(F("speedL limit reached Set State -1->3 \r\n"));
				state = 3;
			}
			// if the iTerm of the PID reaches the limit, rotate CW
			if (fabs(pid.GetIterm()) > iTermLimit && Setpoint > 0.0f) {
				errorHandler.setInfoNoLog(F("iTermLimit reached Set State -1->3 \r\n"));
				state = 3;
			}

			// transition to state 1 if left outside
			if (srvPerSensoren.isLeftOutside() == true) {
				errorHandler.setInfoNoLog(F("left coil outside Set State -1->1 \r\n"));
				state = 1;
			}

			// after 3 second do the normal
			if (millis() - timeInState > 3000UL) {
				state = 0;
				errorHandler.setInfoNoLog(F("Set State -1->0 dt: %ul\r\n"), millis() - timeInState);
			}
			//**********************
			//** State transition end
			//**********************


			//errorHandler.setInfo(F("PCA: %f OUTPUT: %f sL: %d  sR: %d  set: %f\r\n"), PeriCoeffAccel, Output, speedL, speedR, Setpoint);
			if (flagShowValues) {
				// Show calculated values
				errorHandler.setInfo(F("state -1 sL cal: %d  sR cal: %d\r\n"), speedL, speedR);
			}

			speedL = myMax(-100, myMin(100, speedL));
			speedR = myMax(-100, myMin(100, speedR));

			srvMotor.L->setSpeed(speedL);
			srvMotor.R->setSpeed(speedR);

			break;



		case 0: // normal linfollowing

			// important, when calling this state pid.SetMode(AUTOMATIC); must be set.
			// If PID is in manual, this state will drive forward only.

			if (pid.GetMode() == MANUAL) {
				errorHandler.setError(F("TlineFollow PID not in AUTOMATIC mode state 0\r\n"));
			}

			bb.history0.driveDirection = DD_FORWARD;

			speedL = Blackboard::LINEFOLLOW_SPEED_HIGH - (int)Output;
			speedR = Blackboard::LINEFOLLOW_SPEED_HIGH + (int)Output;

			//*********************
			//** State transition
			//*********************
			if (!tunePID) { //don't change state while tuning the PID
				// if the iTerm of the PID reaches the limit, rotate CW
				if (fabs(pid.GetIterm()) > iTermLimit && Setpoint > 0.0f) {
					errorHandler.setInfoNoLog(F("iTermLimit reached Set State 0->3 \r\n"));
					state = 3;
				}
				// if the speedL reaches the limit, rotate CW
				if (speedL > speedLimit && Setpoint > 0.0f) {
					errorHandler.setInfoNoLog(F("speedL limit reached Set State -1->3 \r\n"));
					state = 3;
				}
			}

			// transition to state 1 if left outside
			if (srvPerSensoren.isLeftOutside() == true) {
				state = 1;
				errorHandler.setInfoNoLog(F("Set State 0->1 \r\n"));
			}
			//**********************
			//** State transition end
			//**********************

			if (flagShowValues) {
				// Show calculated values
				errorHandler.setInfo(F("state 0 sL cal: %d  sR cal: %d\r\n"), speedL, speedR);
			}

			speedL = myMax(-100, myMin(100, speedL));
			speedR = myMax(-100, myMin(100, speedR));

			srvMotor.L->setSpeed(speedL);
			srvMotor.R->setSpeed(speedR);

			break;

		case 1: // left coil outside => initiate rotation CC
			bb.history0.cruiseSpeed = 30;
			Output = 0;
			bb.history0.driveDirection = DD_ROTATECC;
			srvMotor.L->setSpeed(-30);
			srvMotor.R->setSpeed(30);
			pid.SetMode(MANUAL);

			// transition to state 2 to wait until right inside
			state = 2;
			errorHandler.setInfoNoLog(F("Set State 1->2 \r\n"));

			break;

		case 2: // wait for right inside
			if (srvPerSensoren.isRightInside()) {
				lastTransitionTime = millis();

				//transition to 5 right inside go stop motors
				errorHandler.setInfoNoLog(F("Set State 2->5\r\n"));
				state = 5;
			}
			break;

		case 3: // iTerm too high and coil inside -> initiate rotation CW
			bb.history0.cruiseSpeed = 30;
			Output = 0;
			bb.history0.driveDirection = DD_ROTATECW;
			srvMotor.L->setSpeed(30);
			srvMotor.R->setSpeed(-30);
			pid.SetMode(MANUAL);

			// transition to state 4 to wait right coil outside
			if (!tunePID) {
				state = 4;
				errorHandler.setInfoNoLog(F("Set State 3->4 \r\n"));
			}
			else {
				state = 0;
				pid.SetMode(AUTOMATIC);
				pid.Initialize();
				errorHandler.setInfoNoLog(F("Set State 3->0 PID Tuning \r\n"));
			}

			break;

		case 4: // wait for right outside
			if (!srvPerSensoren.isRightInside()) {
				lastTransitionTime = millis();

				//transition to 7 right inside go stop motors
				state = 7;
				errorHandler.setInfoNoLog(F("Set State 4->7 \r\n"));
			}
			break;

		case 5: // Motor Stop from CC
			srvMotor.L->stop();
			srvMotor.R->stop();
			pid.SetMode(MANUAL);

			// transition to wait for motor stoped
			errorHandler.setInfoNoLog(F("Set State 5->6 \r\n"));
			state = 6;
			break;

		case 6: // wait that motors just stopped. I use isNearStopped here, to make a smoother
			  // movment and don't wait until all wheels stopped to 0.
			if (srvMotor.L->isNearStopped() && srvMotor.R->isNearStopped()) {

				// transition to start linefollowing smoothly
				if (!tunePID) {
					state = -2;
					errorHandler.setInfoNoLog(F("Set State 6->-2 \r\n"));
				}
				else {
					state = 0;
					pid.SetMode(AUTOMATIC);
					pid.Initialize();
					errorHandler.setInfoNoLog(F("Set State 6->0 PID Tuning \r\n"));
				}
			}
			break;


		case 7: // Motor Stop from CW
			srvMotor.L->stop();
			srvMotor.R->stop();
			pid.SetMode(MANUAL);

			// transition to wait for motor stoped
			errorHandler.setInfoNoLog(F("Set State 7->8 \r\n"));
			state = 8;
			break;

		case 8: // wait that motors just stopped. I use isNearStopped here, to make a smoother
			  // movment and don't wait until all wheels stopped to 0.
			if (srvMotor.R->isNearStopped() && srvMotor.L->isNearStopped()) {

				// transition to start linefollowing smoothly
				if (!tunePID) {
					state = -2;
					errorHandler.setInfoNoLog(F("Set State 8->-2 \r\n"));
				}
				else {
					state = 0;
					pid.SetMode(AUTOMATIC);
					pid.Initialize();
					errorHandler.setInfoNoLog(F("Set State 6->0 PID Tuning \r\n"));
				}
				break;

		default:
			// do nothing
			break;
			}

		}

		if ((millis() - lastTransitionTime) > 12000) {
			errorHandler.setError(F("!03,TlineFollow line crossing > 12000\r\n"));
		}

		if (flagShowValues) {
			errorHandler.setInfo(F("sL: %d  sR: %d  set: %f , in: %f, out: %f\r\n"), speedL, speedR, Setpoint, Input, Output);
		}

		return BH_RUNNING;

	}
	virtual void onTerminate(NodeStatus status, Blackboard& bb) {
		//errorHandler.setInfoNoLog(F("TlineFollow onTerminate \r\n"));
		//srvMotor.enableDefaultRamping();
	}
};



/*
	Fuerr das finden eines Dreiecks in der Schleife wird staendig nach einer Linkskurve "Ausschau" gehalten.
	Dazu werden alle 500ms die Encoderdaten ausgelesen und ausgewertet.
	Wenn eine Linkskurve erkannt wurde wird die Zeit festgehalten und timeLeftTurnFound gesetzt.
	Wenn ein Dreieck vohanden ist, wird der Robbi nicht mehr so einfach von innen nach aussen wechseln koennen,
	da er nach der scharfen Linkskurfe bereits Ã¼ber das Ende des Dreieckes gefahren ist. Daher wird er auf jeden Fall in die if Abfrage:
	"if ( (millis()-lastTransitionTime) > 3500 )" gehen wo er rotiert. Hier wird geguckt, ob die Linkskurve innerhalb
	der letzten 5 Sek. gefunden wurde. Wenn ja, wird flagTriangleFound = true gesetzt und rotiert, bis die andere Seite des Dreiecks erreicht
	wurde. Wenn diese erreicht wurde, geht die Spule von innen nach auÃŸen. Bei diesem Ãœbergang wird geprÃ¼ft ob ein Dreieck gefunden wurde mit
	flagTriangleFound. Weiterhin muss mindesten noch eine Kurve von 80 nach rechts gefahren worden sein und die  Zeit fÃ¼r die Rechtskurve muss mindestens
	4Sek. gedauert haben. Dann werden die Zweige im BehaviourTree mit  bb.flagFollowLine = false;  bb.flagGoHome = true; umgeschaltet, so dass der Mower wieder normal fÃ¤hrt
	bis die andere Seite des Perimeters erreicht wurde.

*/

#define ENCDELTAARRAY 5  // Check curve for 2.5 sec.

class TfindTriangle : public Action {
private:
	unsigned long lastRunAngleCalculation;
	int encDeltaL[ENCDELTAARRAY]; //Array measured encoder ticks/100ms
	int encDeltaR[ENCDELTAARRAY];
	int idxL;
	int idxR;
	float angle;
	long lastEncTicksL;
	long lastEncTicksR;
	uint8_t state;


public:
	float Ki;
	bool flagShowFindTriangleStates;

	TfindTriangle() {
		Ki = 1.1f;
		flagShowFindTriangleStates = false;
	}

	virtual void onInitialize(Blackboard& bb) {
		lastRunAngleCalculation = 0;
		memset(&encDeltaL[0], 0, ENCDELTAARRAY * sizeof(int));
		memset(&encDeltaR[0], 0, ENCDELTAARRAY * sizeof(int));
		idxL = 0;
		idxR = 0;
		lastEncTicksL = srvMotor.getEncoderTickCountsL();
		lastEncTicksR = srvMotor.getEncoderTickCountsR();
		angle = 0;
		state = 0;
		//errorHandler.setInfoNoLog(F("ON INITIALIZE"));

	}

	virtual NodeStatus onUpdate(Blackboard& bb) {
		long sumL = 0;
		long sumR = 0;
		long buffL, buffR;
		float distance;

		if (CONF_DISABLE_FAST_RETURN == true) {
			return BH_RUNNING;
		}

		//============================================
		// Calculate driven angle every 500ms.
		//============================================
		if (millis() - lastRunAngleCalculation >= 500) {
			lastRunAngleCalculation = millis();

			buffL = srvMotor.getEncoderTickCountsL();
			buffR = srvMotor.getEncoderTickCountsR();
			encDeltaL[idxL++] = buffL - lastEncTicksL;
			encDeltaR[idxR++] = buffR - lastEncTicksR;
			lastEncTicksL = buffL;
			lastEncTicksR = buffR;

			if (idxL > ENCDELTAARRAY - 1) idxL = 0;
			if (idxR > ENCDELTAARRAY - 1) idxR = 0;

			/*
				for(int i=0; i<ENCDELTAARRAY; i++) {
				errorHandler.setInfoNoLog(F("%d  %d  %d\r\n"),encDeltaL[i], encDeltaR[i], encDeltaL[i] - encDeltaR[i]);
				}
				errorHandler.setInfoNoLog(F("==============\r\n"));
			*/

			for (int i = 0; i < ENCDELTAARRAY; i++) {
				sumL += encDeltaL[i];
				sumR += encDeltaR[i];
			}

			//The average angle the robot has traveled
			//theta = (LeftEncoderDistance−RightEncoderDistance) / wheelbase
			//theta = (36.8 - 27.97) / 3 = 2.95 radians
			//1 Grad = 0,017453292519943295769236907684886 Bogenmaß (radiant) 1/0,017... = 57,2957795  => angle=(360/(2Pi))*Rad
			//theta = 2.95/0.017453292 oder 2.95*57.2957795
			float cmL = srvMotor.pcL->getCMForCounts(sumL);
			float cmR = srvMotor.pcR->getCMForCounts(sumR);
			angle = (cmL - cmR) * 57.2957795f / CONF_DISTANCE_BETWEEN_WHEELS_CM;
			if (flagShowFindTriangleStates) {
				errorHandler.setInfoNoLog(F("Winkel: %f cmL %f cmR %f\r\n"), angle, cmL, cmR);
			}
		}


		// Check for left curve / right curve / second left curve
		switch (state) {
		case 0:  // search for left curve
			if (angle < -22) {
				if (flagShowFindTriangleStates) {
					errorHandler.setInfo(F("!03,s0 set state = 1 Left turn found angle: %f ms: %lu\r\n"), angle, millis());
				}
				srvMotor.startDistanceMeasurementTriangle();
				// array loeschen damit letzte linkskurve die rechts winkelmessung nicht beeinflusst
				memset(&encDeltaL[0], 0, ENCDELTAARRAY * sizeof(int));
				memset(&encDeltaR[0], 0, ENCDELTAARRAY * sizeof(int));
				state = 1; // Activate triangle searching
			}
			break;

		case 1: // search for right turn
			distance = srvMotor.getDistanceInCMForTriangle();
			if (distance > 70) {
				state = 0;
				if (flagShowFindTriangleStates) {
					errorHandler.setInfo(F("!03,s1 set state = 0 distance %f > 65 ms: %lu\r\n"), distance, millis());
				}
			}

			else if (angle > 50) {
				if (flagShowFindTriangleStates) {
					errorHandler.setInfo(F("!03,s1 set state = 2 angle %f > 50 distance %f\r\n"), angle, distance);
				}
				srvMotor.startDistanceMeasurementTriangle();
				// array loeschen damit letzte linkskurve die rechts winkelmessung nicht beeinflusst
				memset(&encDeltaL[0], 0, ENCDELTAARRAY * sizeof(int));
				memset(&encDeltaR[0], 0, ENCDELTAARRAY * sizeof(int));
				state = 2;
			}

			break;

		case 2:  // search for second left curve
			distance = srvMotor.getDistanceInCMForTriangle();
			if (distance > 77) {
				state = 0;
				if (flagShowFindTriangleStates) {
					errorHandler.setInfo(F("!03,s2 set state = 0 distance %f > 65 ms: %lu\r\n"), distance, millis());
				}
			}

			else if (angle < -25) {
				if (flagShowFindTriangleStates) {
					errorHandler.setInfo(F("!03,s2 set state = 3 angle %f distance %f < -22\r\n"), angle, distance);
				}
				srvMotor.startDistanceMeasurementTriangle();
				// array loeschen damit letzte linkskurve die rechts winkelmessung nicht beeinflusst
				memset(&encDeltaL[0], 0, ENCDELTAARRAY * sizeof(int));
				memset(&encDeltaR[0], 0, ENCDELTAARRAY * sizeof(int));
				state = 3;

			}

			break;

		case 3:

			if (flagShowFindTriangleStates) {
				errorHandler.setInfo(F("!03,s3 set state=0  Cross Lawn Activated\r\n"));
			}
			state = 0;
			//srvMotor.enableDefaultRamping();
			return BH_SUCCESS;

			break;

		default:
			errorHandler.setError(F("!03,TtrackPerimeter state %d not found\r\n"), state);
			break;
		}

		return BH_RUNNING;
	}


	virtual void onTerminate(NodeStatus status, Blackboard& bb) {
		//debug->printf("onTerminate TFLfollowLine enableDefaultRamping()\r\n" );
		//srvMotor.enableDefaultRamping();
	}

};


class TSetArc45CC : public Action    // Each task will be a class (derived from Node of course).
{
private:
public:

	TSetArc45CC() {}

	virtual void onInitialize(Blackboard& bb) {

		errorHandler.setInfo(F("!05,TSetArc45CW\r\n"));

		THistory hist = bb.getInitialisedHistoryEntry();
		hist.cruiseSpeed = bb.CRUISE_SPEED_LOW;
		hist.driveDirection = DD_ROTATECC;
		hist.distanceSoll = -45;
		bb.addHistoryEntry(hist);
		srvMotor.enableDefaultRamping();

	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		return BH_SUCCESS;
	}

};


class TcheckOutsideAgain : public Action    // Each task will be a class (derived from Node of course).
{
private:
public:

	TcheckOutsideAgain() {}

	virtual void onInitialize(Blackboard& bb) {
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {
		if (getTimeInNode() > 1000ul) {
			if (srvPerSensoren.isLeftOutside() || srvPerSensoren.isRightOutside()) {
				errorHandler.setInfo(F("!05,TcheckOutsideAgain perimeter outside\r\n"));
				bb.flagDriveCurve = false;
			}
			else {
				errorHandler.setInfo(F("!05,TcheckOutsideAgain perimeter NOT outside\r\n"));
				//bb.flagDriveCurve = true;  //has not changed, therefore I don't have to set it again
			}
			return BH_SUCCESS;
		}
		return BH_RUNNING;
	}

};


class TdriveCurve : public Action {
private:
	bool oneCoilsOutside;

public:

	TdriveCurve() {}

	virtual void onInitialize(Blackboard& bb) {

		//Check if one coil is outside when start driving
		if (srvPerSensoren.isLeftOutside() || srvPerSensoren.isRightOutside()) {
			oneCoilsOutside = true;
		}
		else {
			oneCoilsOutside = false;
		}


		THistory hist = bb.getInitialisedHistoryEntry();
		hist.cruiseSpeed = bb.CRUISE_SPEED_LOW;
		hist.driveDirection = DD_FORWARD;
		hist.distanceSoll = 1000;
		bb.addHistoryEntry(hist);


		srvMotor.pcL->rotateCM(1000, 65);
		srvMotor.pcR->rotateCM(1000, 54);
		//if (bb.flagShowRotateX) {
		errorHandler.setInfo(F("!03,TdriveCurve\r\n"));
		//}


	}

	virtual NodeStatus onUpdate(Blackboard& bb) {


		if (getTimeInNode() > 60000) { // drive no longer than 60 seconds for searching the perimeter (for security)
			errorHandler.setError(F("!03,TdriveCurve too long in state\r\n"));
			return BH_SUCCESS;
		}


		// If one coil outside while start driving, wait until both coils are inside again.
		if (oneCoilsOutside == false) {
			if (srvPerSensoren.isLeftOutside() || srvPerSensoren.isRightOutside()) {
				errorHandler.setInfo(F("!05,TdriveCurve perimeter found\r\n"));
				return BH_SUCCESS;
			}
		}
		else { // Wait that both coils inside
			if (srvPerSensoren.isLeftInside() && srvPerSensoren.isRightInside()) {
				oneCoilsOutside = false;
			}
			else {
				// If after 20cm driveway both coils are not inside then error.
				if (bb.history0.distanceIst > 20) {
					errorHandler.setError(F("!05,TdriveCurve one or both coils outside after 20cm\r\n"));
				}
			}

		}

		//Because the rigth wheel rotates slower, whe have to ask each wheel individually if its position is reached.
		//It could be, that the left wheel has reached the 10m but not the right wheel. Then the left wheel would be stopped
		//and the right wheel would drive further.
		if (srvMotor.pcL->isPositionReached() || srvMotor.pcR->isPositionReached()) {
			errorHandler.setError(F("!05,TdriveCurve position reached\r\n"));
			return BH_SUCCESS;
		}


		return BH_RUNNING;
	}

	virtual void onTerminate(NodeStatus status, Blackboard& bb) {
		//bb.flagDriveCurve = false;
		/*
			if(status != BH_ABORTED) {

			}
		*/
	}

};



#endif
