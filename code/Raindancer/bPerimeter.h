#pragma once


#ifndef _BPERIMETER_h
#define _BPERIMETER_h


class TSetflagCoilFirstOutside : public Action    // Each task will be a class (derived from Node of course).
{
private:

public:

	TSetflagCoilFirstOutside() {}

	virtual void onInitialize(Blackboard& bb) {
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {
		if (srvPerSensoren.isLeftOutside() && srvPerSensoren.isRightOutside()) {
			bb.history0.coilFirstOutside = CO_BOTH;
			sprintf(errorHandler.msg, "!03,->TsrvSetflagCoilFirstOutsideLatched = CO_BOTH\r\n");
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
		}
		else if (srvPerSensoren.isLeftOutside()) {
			bb.history0.coilFirstOutside = CO_LEFT;
			sprintf(errorHandler.msg, "!03,->TsrvSetflagCoilFirstOutsideLatched = CO_LEFT\r\n");
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
		}
		else if (srvPerSensoren.isRightOutside()) {
			bb.history0.coilFirstOutside = CO_RIGHT;
			sprintf(errorHandler.msg, "!03,->TsrvSetflagCoilFirstOutsideLatched = CO_RIGHT\r\n");
			if (bb.flagBHTShowLastNode) { errorHandler.setInfo(); }
			else { errorHandler.writeToLogOnly(); }
		}
		return BH_SUCCESS;
	}


};




class TOverRun : public Action    // Each task will be a class (derived from Node of course).
{
private:
	float weg;
public:

	TOverRun() {}

	virtual void onInitialize(Blackboard& bb) {
		srvMotor.startDistanceMeasurementCoilOut();
		srvMotor.stopPCAtPerimeter();
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (getTimeInNode() > 10000) {
			errorHandler.setError(F("!03,Runover too long in node\r\n"));
		}

		if (srvPerSensoren.isLeftOutside()) {
			srvMotor.stopDistanceMeasurementLCoilOut();
		}

		if (srvPerSensoren.isRightOutside()) {
			srvMotor.stopDistanceMeasurementRCoilOut();
		}

		//bb.history0.distanceCmIst = bb.motor.getDistanceInCM();

		if (srvMotor.isPositionReached()) {

			bb.history0.coilsOutsideAngle = srvMotor.getDistanceAngleCoilOut();
			//errorHandler.setInfo(F("bb.coilsOutsideAngle: %f\r\n"), bb.coilsOutsideAngle);


			if (srvPerSensoren.isLeftOutside() && srvPerSensoren.isRightOutside()) {
				bb.history0.coilOutsideAfterOverrun = CO_BOTH;
			}
			else if (srvPerSensoren.isLeftOutside()) {
				bb.history0.coilOutsideAfterOverrun = CO_LEFT;
			}
			else if (srvPerSensoren.isRightOutside()) {
				bb.history0.coilOutsideAfterOverrun = CO_RIGHT;
			}


			if (CONF_USE_ZONE_RECOGNITION == false) { // When not using zone recognition drive further if both coils are inside.
				if (srvPerSensoren.isLeftInside() && srvPerSensoren.isRightInside()) {
					//					return BH_FAILURE;
				}
			}

			return BH_SUCCESS;
		}
		return BH_RUNNING;
	}

};


class TRunTempService : public Action    // Each task will be a class (derived from Node of course).
{
private:

public:

	TRunTempService() {}

	virtual void onInitialize(Blackboard& bb) {  // executed each time the node is call

	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		srvDht.Run();
		return BH_SUCCESS;

	}
};


class TPerDriveBack : public Action {
private:
	long weg;
public:

	TPerDriveBack() {}

	virtual void onInitialize(Blackboard& bb) {


		// if 0 cm then do nothing
		if (CONF_PERIMETER_DRIVE_BACK_CM < 0.1f) {
			return;
		}

		srvMotor.rotateCM(-CONF_PERIMETER_DRIVE_BACK_CM, bb.CRUISE_SPEED_MEDIUM); // x cm zurueckfahren
		bb.history0.driveDirection = DD_REVERSE; //That disables the bumper branch

		/*
		// if 0 cm then do nothing
		if (CONF_PERIMETER_DRIVE_BACK_CM < 0.1f) {
			return;
		}

		if (CONF_USE_ZONE_RECOGNITION == false){
			if (bb.coilsOutsideAngle > CONF_PERIMETER_DRIVE_BACK_ANGLE) {
				return;
			}
			bb.cruiseSpeed = bb.CRUISE_SPEED_LOW;
			bb.motor.rotateCM(-CONF_PERIMETER_DRIVE_BACK_CM, bb.cruiseSpeed); // x cm zurueckfahren
			bb.driveDirection = DD_REVERSE_ESC_OBST; ; // DD_REVERSE_INSIDE;
		}

		if (CONF_USE_ZONE_RECOGNITION == true) {
			if (bb.perimeterSensoren.isLeftInside() && bb.perimeterSensoren.isRightInside()) {
				// When using zone recoginition, it could be that the mower runs over the 13cm zone boarder with both coils. If this happend, drive back 10 more cm.
				bb.cruiseSpeed = bb.CRUISE_SPEED_LOW;
				bb.motor.rotateCM(-CONF_PERIMETER_DRIVE_BACK_CM - 10, bb.cruiseSpeed); // x cm zurueckfahren
				bb.driveDirection = DD_REVERSE_ESC_OBST; ; // DD_REVERSE_INSIDE;
			}
			else {
				// if not both coils inside again, drive like  CONF_USE_ZONE_RECOGNITION == false
				if (bb.coilsOutsideAngle > CONF_PERIMETER_DRIVE_BACK_ANGLE) {
					return;
				}
				bb.cruiseSpeed = bb.CRUISE_SPEED_LOW;
				bb.motor.rotateCM(-CONF_PERIMETER_DRIVE_BACK_CM, bb.cruiseSpeed); // x cm zurueckfahren
				bb.driveDirection = DD_REVERSE_ESC_OBST; ; // DD_REVERSE_INSIDE;
			}
		}


		//bb.addHistoryEntry(bb.driveDirection, 0.0f, 0.0f, 0.0f, FRD_NONE, bb.flagCoilFirstOutside);
		*/
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {


		/*
		if (getTimeInNode() > 2000) {
			return BH_SUCCESS;
		}*/

		if (srvMotor.isPositionReached()) {
			bb.history0.restored = true;
			bb.history0.driveDirection = DD_FORWARD;
			return BH_SUCCESS;
		}

		if (getTimeInNode() > 8000) {
			errorHandler.setError(F("!03,TPerDriveBack too long in state\r\n"));
		}

		return BH_RUNNING;

		/*
				if (getTimeInNode() > 8000) {
					errorHandler.setError(F("!03,TPerDriveBack too long in state\r\n"));
				}

				//bb.history0.distanceCmIst = bb.motor.getDistanceInCM();

				// if 0 cm then do nothing
				if (CONF_PERIMETER_DRIVE_BACK_CM < 0.1f) {
					return BH_SUCCESS;
				}

				if (bb.coilsOutsideAngle > CONF_PERIMETER_DRIVE_BACK_ANGLE) {
					return BH_SUCCESS;
				}

				if (bb.motor.isPositionReached()) {
					return BH_SUCCESS;
				}

				return BH_RUNNING;
		*/
	}
};



class TMotorStop : public Action {
private:

public:

	TMotorStop() {}

	virtual void onInitialize(Blackboard& bb) {
		srvMotor.enableDefaultRamping();
		srvMotor.stopPC();
		//errorHandler.setError("!03,onInitializeTMotorStop\r\n");
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (getTimeInNode() > 10000) {
			errorHandler.setError(F("!03,motorStop too long in state\r\n"));
		}

		if (srvMotor.isPositionReached()) {
			//bb.cruiseSpeed = 0;
			//errorHandler.setInfo("!03,motorStopped\r\n");
			return BH_SUCCESS;
		}

		return BH_RUNNING;
	}
};

class TErrorBothCoilsOutside : public Action {
private:

public:

	TErrorBothCoilsOutside() {}

	virtual void onInitialize(Blackboard& bb) {
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {
		if (srvPerSensoren.isLeftOutside() && srvPerSensoren.isRightOutside()) {
			errorHandler.setError(F("!03,Perimeter Both Coils Outside\r\n"));
			return BH_FAILURE;
		}
		return BH_SUCCESS;
	}
};

class TRestoreLastRotation : public Action {
private:

public:

	TRestoreLastRotation() {}

	virtual void onInitialize(Blackboard& bb) {

		switch (bb.history0.driveDirection) {
		case DD_ROTATECW:
		case DD_ROTATECC:
			srvMotor.turnTo(-bb.history0.distanceIst, bb.history0.cruiseSpeed);
			errorHandler.setInfo(F("!03,RestHist driveDirection: %s angle: %f\r\n"), enuDriveDirectionString[bb.history0.driveDirection], -bb.history0.distanceIst);
			break;
		default:
			sprintf(errorHandler.msg, "!03,TRestoreLastRotation driveDirection not found: %s\r\n", enuDriveDirectionString[bb.history0.driveDirection]);
			errorHandler.setError();
			break;
		}
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (srvMotor.isPositionReached()) {
			bb.history0.restored = true;
			return BH_SUCCESS;
		}

		if (getTimeInNode() > 10000) {
			errorHandler.setError(F("!03,TRestoreLastRotation too long in state\r\n"));
		}

		return BH_RUNNING;
	}
};


class TInvertRotationDirection : public Action {
private:

public:

	TInvertRotationDirection() {}

	virtual void onInitialize(Blackboard& bb) {
		THistory hist;

		hist = bb.history0;

		switch (hist.driveDirection) {
		case DD_ROTATECW:
			hist.driveDirection = DD_ROTATECC;
			hist.distanceSoll = -myRandom(90, 100); // -fabs(hist.distanceSoll);
			break;
		case DD_ROTATECC:
			hist.driveDirection = DD_ROTATECW;
			hist.distanceSoll = myRandom(90, 100); //fabs(hist.distanceSoll);
			break;
		default:
			sprintf(errorHandler.msg, "!03,TInvertRotationDirection driveDirection not found: %s\r\n", enuDriveDirectionString[bb.history0.driveDirection]);
			errorHandler.setError();
			break;
		}
		bb.addHistoryEntry(hist);
	}

	virtual NodeStatus onUpdate(Blackboard& bb) {

		return BH_SUCCESS;
	}
};



#endif