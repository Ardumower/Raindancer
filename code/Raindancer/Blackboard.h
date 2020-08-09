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

#ifndef BLACKBOARD_H
#define BLACKBOARD_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif



#include "helpers.h"



//Achtung, bei Aenderung auch enuDriveDirectionString[] aendern!!!
enum enuDriveDirection {
	DD_FORWARD = 0,
	DD_REVERSE = 1, //Darf eigentlich nicht vorkommen wenn ich prÃ¼fe, dass Spule hinten rausfÃ¤hrt beim zurÃ¼ckfahren
	DD_ROTATECW = 2,
	DD_ROTATECC = 3,
	DD_SPIRAL_CW = 4,
	DD_NONE = 5        //Only used in History to init the drivedirection
};

extern const char* enuDriveDirectionString[];  //in Blackboard.cpp definiert


enum enuFlagCoilsOutside { CO_NONE, CO_RIGHT, CO_LEFT, CO_BOTH };
extern const char* enuFlagCoilsOutsideString[]; //in Blackboard.cpp definiert

enum enuFlagBumperActivated { BUM_NONE, BUM_RIGHT, BUM_LEFT, BUM_BOTH, BUM_DUINO };
extern const char* enuFlagBumperActivatedString[]; //in Blackboard.cpp definiert


enum enuFlagPeriStateChanged { PSC_NONE, PSC_IO, PSC_OI };

enum enuFlagForceRotateDirection { FRD_NONE, FRD_CC, FRD_CW };
extern const char* enuFlagForceRotateDirectionString[]; //in Blackboard.cpp definiert

enum enuBehaviour { BH_NONE, BH_MOW, BH_PERITRACK, BH_CHARGING, BH_GOTOAREA, BH_FINDPERIMETER, BH_LEAVE_HEAD_CS };


//Structure for history and current rotation data
typedef struct {
	enuDriveDirection driveDirection;		// Enthaelt Fahrtrichtung
	float  distanceSoll;                      // Enthaelt die zu fahrende istanz in cm bzw. den zu drehenden Winkel in grad
	float  distanceIst;				// Enthaelt die gerade gefahrene distanz in cm bzw. den gedrehten Winkel in grad
	enuFlagCoilsOutside   coilFirstOutside;   // which coil was first outside jet
	enuFlagCoilsOutside   coilOutsideAfterOverrun; // which coil was first outside after overrun
	bool restored;					// wurde die node restored
	unsigned long timeAdded;			// zeit, wann node erstellt wurde in ms
	enuFlagBumperActivated bumperActivated;	// Was bumper activated?
	int16_t cruiseSpeed;
	float coilsOutsideAngle;			// what was the angle of the coils regarding to the perimeter

} THistory;
#define HISTROY_BUFSIZE 15

class Node;


// We will use a Blackboard object. This object will be created by the user and will be propagated to the nodes during the Blackboard through the tick function.
class Blackboard {
public:
	Blackboard(): history0(history[0]){
		flagEnableMowing = false;
		flagEnablePerimetertracking = false;
		flagEnableCharging = false;
		flagEnableGotoAreaX = false;
		flagEnableFindPerimeter = false;
		flagEnableLeaveHeadChargingStation = false;

		flagBHTShowChanges = true;
		
	}


	// Variables used by default BehaviourTree funktions
	bool flagBHTShowChanges; // shows leaf which changes the state during a tick

	THistory& history0;
	THistory history[HISTROY_BUFSIZE];

	long randAngle;

	static const int CRUISE_SPEED_HIGH = 85;//92,90
	static const int CRUISE_SPEED_MEDIUM = 70;
	static const int CRUISE_SPEED_LOW = 50;
	static const int CRUISE_SPEED_OBSTACLE = 40;  // Lowest Speed to drive
	static const int CRUISE_ROTATE_HIGH = 40;
	static const int CRUISE_ROTATE_LOW = 25;
	static const int LINEFOLLOW_SPEED_HIGH = 50;
	static const int LINEFOLLOW_SPEED_LOW = 50;
	static const int SHORTWAYCOUNT = 3; //3
	
	
	enuFlagForceRotateDirection flagForceRotateDirection;
	

	bool flagCruiseSpiral;
	bool flagForceAngle;
	//bool flagPerimeterActivated;
	bool flagRotateAtPer;
	bool flagDriveCurve;

	int flagForceSmallRotAngle; // Anzahl, wie haeufig kleiner Winkel gedreht werden soll 


	bool flagEnableMowing;
	bool flagEnablePerimetertracking;
	bool flagEnableCharging;
	bool flagEnableGotoAreaX;
	bool flagEnableFindPerimeter;
	bool flagEnableLeaveHeadChargingStation;

	

	bool flagEnableRestoreHistory;

	bool flagGotoAreaXFirstCall;

	bool flagGoHome;

	
	

	int32_t numberOfRotations;
	unsigned long timeInMowBehaviour;


	//int arcRotateXArc;

	long areaTargetDistanceInMeter;

	unsigned long lastTimeSpiralStarted;

	bool flagBHTShowLastNode; //shows last nodes and conditions on the terminal

	bool flagShowRotateX;
	bool flagShowHistory;


	void resetBB();
	void setBehaviour(enuBehaviour b);


	void addHistoryEntry(THistory& hist);
	THistory getInitialisedHistoryEntry();
	void printHistoryEntry(int x);
	void historyUpdateDistance();
	enuDriveDirection historyGetLastRotateDirection();
	void deleteLastHistoryEntry();
	int8_t histGetThreeLastForwardDistances(float& a, float& b, float& c);

};


#endif

