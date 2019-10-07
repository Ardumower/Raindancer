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
#include "bConditions.h"
#include "bCreateTree.h"
#include "bCruise.h"
#include "bFreePerimeter.h"
#include "bRotate.h"
#include "bFreeBumper.h"
#include "bEscapeObstacle.h"
#include "bPerimeterTracking.h"
#include "bCharging.h"
#include "bGotoAreaX.h"
#include "bCheck2.h"
#include "bEscapeOOP.h"
#include "bRestoreHistory.h"
#include "bHeadStation.h"


// ************************************
// Conditions Behaviour
//*************************************





//TConFEO_BACK180 conFEO_BACK180;
//TConFEO_FWD20   conFEO_FWD20;
TConFEO_ROTCC2   conFEO_ROTCC2;
TConFEO_ROTCW2   conFEO_ROTCW2;
TConFEO_ROTCC1   conFEO_ROTCC1;
TConFEO_ROTCW1   conFEO_ROTCW1;
TConFEO_ROT     conFEO_ROT;
TConFEO_BACKINSIDE conFEO_BACKINSIDE;


TConWasDirectionFeoRotateCC conWasDirectionFeoRotateCC;
TConWasDirectionFeoRotateCW conWasDirectionFeoRotateCW;
TConWasDirectionFeoRotateCC1 conWasDirectionFeoRotateCC1;
TConWasDirectionFeoRotateCW1 conWasDirectionFeoRotateCW1;

TConWasDirectionForward conWasDirectionForward;
//TConWasDirectionForward20 conWasDirectionForward20;
TConWasDirectionForwardInside conWasDirectionForwardInside;
TConWasDirectionReverseObstacle conWasDirectionReverseObstacle;

TConWasDirectionOverrun conWasDirectionOverrun;

TConWasDirectionReverseInside conWasDirectionReverseInside;

TConWasDirectionRotateCC conWasDirectionRotateCC;
TConWasDirectionRotateCW conWasDirectionRotateCW;


TConBumperActive conBumperActive;
TConPerOutside conPerOutside;

//TconStopOvershootLeft conStopOvershootLeft;
//TconStopOvershootRight conStopOvershootRight;
TConBatLow conBatLow;
TConRaining conRaining;

TConRightCoilOutside conRightCoilOutside;
TConLeftCoilOutside  conLeftCoilOutside;
TConOneCoilOutside conOneCoilOutside;
TConBothCoilsOutside conBothCoilsOutside;

TConInDockingStation conInDockingStation;

TconAreaReached conAreaReached;

TconRotateAtPer conRotateAtPer;
TconDriveCurve conDriveCurve;
TconSecondReverse conSecondReverse;

// ************************************
// Decorator Nodes Behaviour
//*************************************

ExecuteOnTrue eotCruiseSpiral;
ExecuteOnTrue eotMowing;
ExecuteOnTrue eotPermeterTracking;
ExecuteOnTrue eotBumperActivated;
ExecuteOnTrue eotBumpPeriActivated;
ExecuteOnTrue eotCharging;
ExecuteOnTrue eotGotoAreaX;
ExecuteOnTrue eotFindPerimeter;
ExecuteOnTrue eotRestoreHistory;
ExecuteOnTrue eotLeaveHeadChargingStation;

WaitDecorator dnWaitDockingStation;
WaitDecorator dnWaitGotoArea;
WaitDecorator dnWaitFreeBumper2;



// ************************************
// Escape Obstacle Outside Perimeter
//*************************************

TsetDD_FORWARD setDD_FORWARD;
//Sequence seqEscOOPRot;
MemSequence mseqEscOOPRotCC;
MemSequence mseqEscOOPRotCW;


MemSequence mseqEscOOPReverse;
MemSelector mselEscObstacleOutside;

// ************************************
// Free Bumper Behaviour
//*************************************

TSetflagBumperActivatedLR setflagBumperActivatedLR;
TselEscapeAlgorithm selEscapeAlgorithm;
THardstop hardstop;
TFreeBumper freeBumper;
MemSequence mseqBumperActive;


// ************************************
// Free Perimeter Behaviour
//*************************************

TConditionPerimeterNotFound conditionPerimeterNotFound;

TRotateBackCW rotateBackCW;
TRotateBackCC rotateBackCC;

TSetflagCoilFirstOutsideLatched setflagCoilFirstOutsideLatched;
TSetflagCoilFirstOutside setflagCoilFirstOutside;

TSetflagForceSmallRotAngle setFlagForceSmallRotAngle;
//MemSequence mseqPerimeterForward20;

MemSequence mseqPerimeterForwardInside;

TSetArc90CW setArc90CW;
TSetArc90CC setArc90CC;
TSetArc90CW1 setArc90CW1;
TSetArc90CC1 setArc90CC1;

TSetArc20CW setArc20CW;
TSetArc20CC setArc20CC;

MemSequence mseqPRRCO;
MemSequence mseqPRLCO;
MemSelector mselPerimeterReverse;

MemSequence mseqPerimeterFeoRotCC;
MemSequence mseqPerimeterFeoRotCW;

MemSequence mseqPerimeterFeoRotCC1;
MemSequence mseqPerimeterFeoRotCW1;


//TPerRotateInsideCW perRotateInsideCW;
//TPerRotateInsideCC perRotateInsideCC;
TMotorStop motorStop;
MemSequence mseqPerimeterRotCC;
MemSequence mseqPerimeterRotCW;

TOverRun overRun;

TRunTempService runTempService;

TPerDriveBack perDriveBack;
TReverseFurtherInside reverseFurtherInside;

TForwardInside forwardInside;
TForwardInsideError forwardInsideError;

TDirectionOverrunError directionOverrunError;
TReverseInsideError reverseInsideError;



MemSequence mseqPerimeterRevInside;
MemSequence mseqPerimeterOverrun;

TRotateBothCoilsInside rotateBothCoilsInside;


MemSequence mseqPerimeterReverse;
MemSequence mseqPerimeterForward;
MemSelector mselPerimeterActive;
MemSequence mseqPerimeterAvtive;

// ************************************
// Goto Area Behaviour
//*************************************
TsetMowBehaviour setMowBehaviour;
TlineFollow lineFollow;
TARrotate90CC arRotate90CC;
MemSequence mseqGotoAreaX;
Selector selGotoAreaX;


// ************************************
// Charging Behaviour
//*************************************

Selector selCharging;
TchargeRelayOn chargeRelayOn;


// ************************************
// Find Perimeter Behaviour
//*************************************

WaitDecorator dnWaitSignalOutside;
TConIsOutsidePerimeter conIsOutsidePerimeter;
Selector selFindPerimeter;

// ************************************
// Perimeter Tracking Behaviour
//*************************************


MemSequence mseqSecondReverse;

TdriveCurve driveCurve;
TSetArc45CC setArc45CC;
MemSequence mseqTrackPerimeterEscapeObst;
MemSequence mseqTrackPerimeterEscapeObst2;

MemSequence mseqBumperActive2;
TFreeBumper2 freeBumper2;

TfindTriangle findTriangle;
Parallel parLineFollow;

TLFRotateCC105 LFRotateCC105;
MemSequence mseqTrackPerimeter;

TMotorStopFast MotorStopFast;
MemSequence mseqDockingStation;
TperTrackChargingStationReached perTrackChargingStationReached;
TcheckOutsideAgain checkOutsideAgain;

//TLFRotateCW LFRotateCW;

//TFLRotateCC FLRotateCC;



//Failer    FLFailer;
//MemSequence mseqOvershootLeft;
//MemSequence mseqOvershootRight;
//Selector selPerTracking;
//MemSelector mselFollowLine;


// ************************************
// Escape Obstacle Inside Perimeter
//*************************************

//MemSequence mseqEscRotate;

TConditionFEONotFound conditionFEONotFound;
TEscRotateCW escRotateCW;
MemSequence mseqEscRotateCW1;
MemSequence mseqEscRotateCW2;


TEscRotateCC escRotateCC;
MemSequence mseqEscRotateCC1;
MemSequence mseqEscRotateCC2;


//TForward20 forward20;
//MemSequence mseqEscForward;


TSecondReverse  secondReverse;
TSecondReverse2  secondReverse2;
TSecondReverse3  secondReverse3;
MemSequence mseqEscBackward;

MemSelector mselEscabeObstacle;

Selector selEscabeObstacle1;


// ************************************
// Perimeter Outside Behaviour
//*************************************


TPreUpdateHistory preUpdateHistory;
TCalcAngle calcAngle;
TRotateDriveBackInside rotateDriveBackInside;
//TPostUpdateHistory postUpdateHistory;
//TRotatePer rotatePer;



MemSequence mseqRotatePer;
MemSelector mselRotatePer1;



// ************************************
// Rotate 
//*************************************


TSetArcFEO_ROT    setArcFEO_ROT;
TSetArcFEO_ROTCW1 setArcFEO_ROTCW1;
TSetArcFEO_ROTCC1 setArcFEO_ROTCC1;
TSetArcFEO_ROTCW2 setArcFEO_ROTCW2;
TSetArcFEO_ROTCC2 setArcFEO_ROTCC2;

TPreUpdateHistoryBump preUpdateHistoryBump;
TRotateX    rotateX;
MemSequence mseqRotateBump;

// ************************************
// BatLow and raining Behaviour
//*************************************

TCruiseBatLow cruiseBatLow;
TCruiseRaining cruiseRaining;
Sequence seqMowBatLow;
Sequence seqRaining;

// ************************************
// Check2 Behaviour
//*************************************

Selector selCheck2;

TCheck2CoilSignalAreaX Check2CoilSignalAreaX;

TCheck2LeftCoilSignal Check2LeftCoilSignal;
TCheck2RightCoilSignal Check2RightCoilSignal;

TCheck2PerSignal Check2PerSignal;
TCheck2AllCoilsOutside Check2AllCoilsOutside;

// ************************************
// Cruise Behaviour
//*************************************

TCruiseSpiral cruiseSpiral;

TCruiseStartMowMotor CruiseStartMowMotor;

TCruiseRotCW CruiseRotCW;
TCruiseStopped CruiseStopped;
TCruiseToPerimeter CruiseToPerimeter;


Sequence seqFindPerimeter;
MemSequence mseqFindPerimeter;

TCruiseSpeedToMotor CruiseSpeedToMotor;
TCruiseObstacleNear CruiseObstacleNear;
TCruisePerimeterNear CruisePerimeterNear;
TCruiseHighSpeed CruiseHighSpeed;
Selector selCruiseSpeed;
Sequence seqCruise;
ExecuteOnTrue eotSetbbShortWayCounter;


Selector selPerimeterTracking;
Selector selMowing;

// ************************************
// Restore History Behaviour
//*************************************
Selector selRestoreHist;
TRestoreHistory restoreHistory;

// ************************************
// Leave Head Charging Station Behaviour
//*************************************
Selector selLeaveHeadCS;
MemSequence mseqLeaveHeadCS;
TdriveBackXCS driveBackXCS;
TdriveForwardXCS driveForwardXCS;
TSetArcHeadStation_ROT setArcHeadStation_ROT;


void TBehaviour::reset() {
	errorHandler.setInfo(F("TBehaviour::reset\r\n"));
	bb.resetBB();
	behaviorTree.reset(bb);
	bb.setBehaviour(BH_NONE);
}

void TBehaviour::setup() {
	reset();


	// ************************************
	// Conditions Behaviour
	//*************************************

	    //conFEO_BACK180.m_nodeName = (char*)"conFEO_BACK180";
	    //conFEO_FWD20.m_nodeName = (char*)"conFEO_FWD20";
	conFEO_ROTCC2.m_nodeName = (char*)"conFEO_ROTCC2";
	conFEO_ROTCW2.m_nodeName = (char*)"conFEO_ROTCW2";
	conFEO_ROTCC1.m_nodeName = (char*)"conFEO_ROTCC1";
	conFEO_ROTCW1.m_nodeName = (char*)"conFEO_ROTCW1";
	conFEO_ROT.m_nodeName = (char*)"conFEO_ROT";
	conFEO_BACKINSIDE.m_nodeName = (char*)"conFEO_BACKINSIDE";


	conWasDirectionFeoRotateCC.m_nodeName = (char*)"conWasDirectionFeoRotateCC";
	conWasDirectionFeoRotateCW.m_nodeName = (char*)"conWasDirectionFeoRotateCW";
	conWasDirectionFeoRotateCC1.m_nodeName = (char*)"conWasDirectionFeoRotateCC1";
	conWasDirectionFeoRotateCW1.m_nodeName = (char*)"conWasDirectionFeoRotateCW1";

	conWasDirectionForward.m_nodeName = (char*)"conWasDirectionForward";
	//conWasDirectionForward20.m_nodeName = (char*)"conWasDirectionForward20";
	conWasDirectionForwardInside.m_nodeName = (char*)"conWasDirectionForwardInside";
	conWasDirectionOverrun.m_nodeName = (char*)"conWasDirectionOverrun";
	conWasDirectionReverseObstacle.m_nodeName = (char*)"conWasDirectionReverseObstacle";
	conWasDirectionReverseInside.m_nodeName = (char*)"conWasDirectionReverseInside";
	conWasDirectionRotateCC.m_nodeName = (char*)"conWasDirectionRotateCC";
	conWasDirectionRotateCW.m_nodeName = (char*)"conWasDirectionRotateCW";
	//conNOTflagPerimeterActivated.m_nodeName = "conNOTflagPerimeterActivated";

	conBumperActive.m_nodeName = (char*)"conBumperActive";
	conPerOutside.m_nodeName = (char*)"conPerOutside"; //Condition

	//conStopOvershootLeft.m_nodeName = "conStopOvershootLeft";
	//conStopOvershootRight.m_nodeName = "conStopOvershootRight";
	conBatLow.m_nodeName = (char*)"conBatLow";
	conRaining.m_nodeName = (char*)"conRaining";

	conRightCoilOutside.m_nodeName = (char*)"conRightCoilOutside";
	conLeftCoilOutside.m_nodeName = (char*)"conLeftCoilOutside";

	conAreaReached.m_nodeName = (char*)"conAreaReached";

	conRotateAtPer.m_nodeName = (char*)"conRotateAtPer";
	conDriveCurve.m_nodeName = (char*)"conDriveCurve";
	conSecondReverse.m_nodeName = (char*)"conSecondReverse";

	// ************************************
	// Decorator Nodes Behaviour
	//*************************************

	eotCruiseSpiral.m_nodeName = (char*)"eotCruiseSpiral";
	eotCruiseSpiral.setFlag(&bb.flagCruiseSpiral);
	eotCruiseSpiral.setChild(&cruiseSpiral);

	eotBumpPeriActivated.m_nodeName = (char*)"eotBumpPeriActivated";
	eotBumpPeriActivated.setFlag(&bb.flagBumperOutsidePerActivated);
	eotBumpPeriActivated.setFlagToFalseOnTerminate();
	eotBumpPeriActivated.setChild(&mselEscObstacleOutside);


	eotMowing.m_nodeName = (char*)"eotMowing";
	eotMowing.setFlag(&bb.flagEnableMowing);
	eotMowing.setChild(&selMowing);

	eotPermeterTracking.m_nodeName = (char*)"eotPermeterTracking";
	eotPermeterTracking.setFlag(&bb.flagEnablePerimetertracking);
	eotPermeterTracking.setChild(&selPerimeterTracking);

	eotCharging.m_nodeName = (char*)"eotCharging";
	eotCharging.setFlag(&bb.flagEnableCharging);
	eotCharging.setChild(&selCharging);

	eotGotoAreaX.m_nodeName = (char*)"eotGotoAreaX";
	eotGotoAreaX.setFlag(&bb.flagEnableGotoAreaX);
	eotGotoAreaX.setChild(&selGotoAreaX);

	eotFindPerimeter.m_nodeName = (char*)"eotFindPerimeter";
	eotFindPerimeter.setFlag(&bb.flagEnableFindPerimeter);
	eotFindPerimeter.setChild(&selFindPerimeter);

	conInDockingStation.m_nodeName = (char*)"conInDockingStation";

	dnWaitDockingStation.m_nodeName = (char*)"dnWaitDockingStation";
	dnWaitDockingStation.setChild(&conInDockingStation);
	dnWaitDockingStation.setWaitMillis(2000);


	dnWaitFreeBumper2.m_nodeName = (char*)"dnWaitFreeBumper2";
	dnWaitFreeBumper2.setChild(&freeBumper2);
	dnWaitFreeBumper2.setWaitMillis(1000);

	eotRestoreHistory.m_nodeName = (char*)"eotRestoreHistory";
	eotRestoreHistory.setFlag(&bb.flagEnableRestoreHistory);
	eotRestoreHistory.setChild(&selRestoreHist);

	dnWaitGotoArea.m_nodeName = (char*)"dnWaitGotoArea";
	dnWaitGotoArea.setChild(&lineFollow);
	dnWaitGotoArea.setWaitMillis(2000);

	eotLeaveHeadChargingStation.m_nodeName = (char*)"eotLeaveHeadChargingStation";
	eotLeaveHeadChargingStation.setFlag(&bb.flagEnableLeaveHeadChargingStation);
	eotLeaveHeadChargingStation.setChild(&selLeaveHeadCS);

	// ************************************
	// Escape Obstacle Outside Perimeter
	//*************************************


	mseqEscOOPReverse.m_nodeName = (char*)"mseqEscOOPReverse";
	mselEscObstacleOutside.m_nodeName = (char*)"mselEscObstacleOutside";
	setDD_FORWARD.m_nodeName = (char*)"setDD_FORWARD";
	//seqEscOOPRot.m_nodeName = (char*)"seqEscOOPRot";
	mseqEscOOPRotCC.m_nodeName = (char*)"mseqEscOOPRotCC";
	mseqEscOOPRotCW.m_nodeName = (char*)"mseqEscOOPRotCW";

	//seqEscOOPRot.addChildren(&mseqEscRotate,&setDD_FORWARD);
	mseqEscOOPRotCC.addChildren(&conFEO_ROTCC2, &secondReverse2, &setArc90CC, &mseqRotatePer, &setDD_FORWARD);
	mseqEscOOPRotCW.addChildren(&conFEO_ROTCW2, &secondReverse2, &setArc90CW, &mseqRotatePer, &setDD_FORWARD);

	mseqEscOOPReverse.addChildren(&conFEO_BACKINSIDE, &calcAngle, &mseqRotatePer);
	//mseqEscOOPReverse.addChildren(&conFEO_BACKINSIDE,&reverseInside,&reverseFurtherInside,&mseqRotatePer);
	mselEscObstacleOutside.addChildren(&mseqEscOOPReverse, &mseqEscOOPRotCW, &mseqEscOOPRotCC, &conditionFEONotFound);


	// ************************************
	// Bumper Behaviour
	//*************************************

	setflagBumperActivatedLR.m_nodeName = (char*)"setflagBumperActivatedLR";
	selEscapeAlgorithm.m_nodeName = (char*)"selEscapeAlgorithm";
	hardstop.m_nodeName = (char*)"hardstop";

	freeBumper.m_nodeName = (char*)"freeBumper";
	mseqBumperActive.m_nodeName = (char*)"mseqBumperActive";

	//mseqBumperActive.addChildren(&conBumperActive,&hardstop,&freeBumper,&selEscapeAlgorithm );
	mseqBumperActive.addChildren(&conBumperActive, &setflagBumperActivatedLR, &MotorStopFast, &freeBumper, &selEscapeAlgorithm);


	// ************************************
	// Free Perimeter Behaviour
	//*************************************

	runTempService.m_nodeName = (char*)"runTempService";

	conditionPerimeterNotFound.m_nodeName = (char*)"conditionPerimeterNotFound";

	rotateBackCW.m_nodeName = (char*)"rotateBackCW";
	rotateBackCC.m_nodeName = (char*)"rotateBackCC";


	setflagCoilFirstOutsideLatched.m_nodeName = (char*)"setflagCoilFirstOutsideLatched";
	setflagCoilFirstOutside.m_nodeName = (char*)"setflagCoilFirstOutside";
	setFlagForceSmallRotAngle.m_nodeName = (char*)"flagForceSmallRotAngle";
	//mseqPerimeterForward20.m_nodeName = (char*)"mseqPerimeterForward20";

	mseqPerimeterForwardInside.m_nodeName = (char*)"mseqPerimeterForwardInside";

	setArc90CW.m_nodeName = (char*)"setArc90CW";
	setArc90CC.m_nodeName = (char*)"setArc90CC";
	setArc90CW1.m_nodeName = (char*)"setArc90CW1";
	setArc90CC1.m_nodeName = (char*)"setArc90CC1";
	mseqPRRCO.m_nodeName = (char*)"mseqPRRCO";
	mseqPRLCO.m_nodeName = (char*)"mseqPRLCO";
	mselPerimeterReverse.m_nodeName = (char*)"mseqPRLCO";


	motorStop.m_nodeName = (char*)"motorStop";

	mseqPerimeterFeoRotCC.m_nodeName = (char*)"mseqPerimeterFeoRotCC";
	mseqPerimeterFeoRotCW.m_nodeName = (char*)"mseqPerimeterFeoRotCW";
	mseqPerimeterFeoRotCC1.m_nodeName = (char*)"mseqPerimeterFeoRotCC1";
	mseqPerimeterFeoRotCW1.m_nodeName = (char*)"mseqPerimeterFeoRotCW1";
	//    perRotateInsideCW.m_nodeName = (char*)"perRotateInsideCW";
	//    perRotateInsideCC.m_nodeName = (char*)"perRotateInsideCC";
	setArc20CW.m_nodeName = (char*)"setArc20CW";
	setArc20CC.m_nodeName = (char*)"setArc20CC";
	mseqPerimeterRotCC.m_nodeName = (char*)"mseqPerimeterRotCC";
	mseqPerimeterRotCW.m_nodeName = (char*)"mseqPerimeterRotCW";


	overRun.m_nodeName = (char*)"overRun";

	reverseFurtherInside.m_nodeName = (char*)"reverseFurtherInside";
	perDriveBack.m_nodeName = (char*)"perDriveBack";

	conOneCoilOutside.m_nodeName = (char*)"conOneCoilOutside";
	conBothCoilsOutside.m_nodeName = (char*)"conBothCoilsOutside";

	reverseInsideError.m_nodeName = (char*)"reverseInsideError";
	mseqPerimeterRevInside.m_nodeName = (char*)"mseqPerimeterRevInside";

	directionOverrunError.m_nodeName = (char*)"directionOverrunError";
	mseqPerimeterOverrun.m_nodeName = (char*)"seqPerimeterOverrun";

	mseqPerimeterForward.m_nodeName = (char*)"mseqPerimeterForward";
	mselPerimeterActive.m_nodeName = (char*)"mselPerimeterActive";
	mseqPerimeterAvtive.m_nodeName = (char*)"mseqPerimeterAvtive";

	forwardInside.m_nodeName = (char*)"forwardInside";
	forwardInsideError.m_nodeName = (char*) "forwardInsideError";
	mseqPerimeterReverse.m_nodeName = (char*)"mseqPerimeterReverse";

	rotateBothCoilsInside.m_nodeName = (char*)"rotateBothCoilsInside";



	mseqPerimeterRevInside.addChildren(&conWasDirectionReverseInside, &motorStop, &reverseInsideError);

	mseqPerimeterOverrun.addChildren(&conWasDirectionOverrun, &motorStop, &directionOverrunError);

	mseqPerimeterForwardInside.addChildren(&conWasDirectionForwardInside, &motorStop, &forwardInsideError);

	//Reverse
	mseqPRRCO.addChildren(&conRightCoilOutside, &motorStop, &setArc90CC, &mseqRotatePer, &forwardInside);
	mseqPRLCO.addChildren(&conLeftCoilOutside, &motorStop, &setArc90CW, &mseqRotatePer, &forwardInside);
	mselPerimeterReverse.addChildren(&mseqPRLCO, &mseqPRRCO, &forwardInside);

	mseqPerimeterReverse.addChildren(&conWasDirectionReverseObstacle, &motorStop, &mselPerimeterReverse);

	mseqPerimeterFeoRotCW.addChildren(&conWasDirectionFeoRotateCW, &motorStop, &rotateBackCC, &setArc90CC1, &mseqRotatePer);
	mseqPerimeterFeoRotCC.addChildren(&conWasDirectionFeoRotateCC, &motorStop, &rotateBackCW, &setArc90CW1, &mseqRotatePer);

	mseqPerimeterFeoRotCW1.addChildren(&conWasDirectionFeoRotateCW1, &motorStop, &rotateBackCC, &secondReverse2, &setArc90CC, &mseqRotatePer);
	mseqPerimeterFeoRotCC1.addChildren(&conWasDirectionFeoRotateCC1, &motorStop, &rotateBackCW, &secondReverse2, &setArc90CW, &mseqRotatePer);

	mseqPerimeterRotCC.addChildren(&conWasDirectionRotateCC, &motorStop, &setArc20CC, &mseqRotatePer, &setDD_FORWARD);
	mseqPerimeterRotCW.addChildren(&conWasDirectionRotateCW, &motorStop, &setArc20CW, &mseqRotatePer, &setDD_FORWARD);

	//mseqPerimeterRotCC.addChildren(&conWasDirectionRotateCC,&motorStop,&reverseInside,&reverseFurtherInside, &seqRotatePer);
	//mseqPerimeterRotCW.addChildren(&conWasDirectionRotateCW,&motorStop,&reverseInside,&reverseFurtherInside, &seqRotatePer);

  //	mseqPerimeterForward.addChildren(&conWasDirectionForward, &motorStop, &mseqReverseInside, &reverseFurtherInside, &seqRotatePer);
  //	mseqPerimeterForward.addChildren(&conWasDirectionForward, &overRun, &mselPerimeterForward, &seqRotatePer);


	mseqPerimeterForward.addChildren(&conWasDirectionForward, &overRun, &runTempService, &perDriveBack, &calcAngle, &mseqRotatePer);
	mselPerimeterActive.addChildren(&mseqPerimeterForward, &mseqPerimeterFeoRotCC, &mseqPerimeterFeoRotCW, &mseqPerimeterFeoRotCW1, &mseqPerimeterFeoRotCC1, &mseqPerimeterRotCC, &mseqPerimeterRotCW, &mseqPerimeterReverse, &mseqPerimeterForwardInside, &mseqPerimeterOverrun, &mseqPerimeterRevInside, &conditionPerimeterNotFound);

	mseqPerimeterAvtive.addChildren(&conPerOutside, &setflagCoilFirstOutside, &mselPerimeterActive);


	// ************************************
	// GoTo Area Behaviour
	//*************************************
	selGotoAreaX.m_nodeName = (char*)"selGotoAreaX";
	mseqGotoAreaX.m_nodeName = (char*)"mseqGotoAreaX";
	lineFollow.m_nodeName = (char*)"lineFollow";
	arRotate90CC.m_nodeName = (char*)"arRotate90CC";
	setMowBehaviour.m_nodeName = (char*)"setMowBehaviour";

	mseqGotoAreaX.addChildren(&conAreaReached, &motorStop, &arRotate90CC, &setMowBehaviour);
	selGotoAreaX.addChildren(&CruiseStartMowMotor, &mseqBumperActive2, &mseqSecondReverse, &mseqTrackPerimeterEscapeObst, &mseqTrackPerimeterEscapeObst2, &mseqGotoAreaX, &dnWaitGotoArea);

	// ************************************
	// Charging Behaviour
	//*************************************

	selCharging.m_nodeName = (char*)"selCharging";
	chargeRelayOn.m_nodeName = (char*)"chargeRelayOn";
	selCharging.addChildren(&chargeRelayOn);



	// ************************************
	// Find Perimeter Behaviour
	//*************************************

	selFindPerimeter.m_nodeName = (char*)"selCharging";
	seqFindPerimeter.m_nodeName = (char*)"seqFindPerimeter",
		mseqFindPerimeter.m_nodeName = (char*)"mseqFindPerimeter";
	conIsOutsidePerimeter.m_nodeName = (char*)"conIsOutsidePerimeter";

	dnWaitSignalOutside.m_nodeName = (char*)"dnWaitSignalOutside";
	dnWaitSignalOutside.setChild(&conIsOutsidePerimeter);
	dnWaitSignalOutside.setWaitMillis(2000);

	seqFindPerimeter.addChildren(&selCruiseSpeed, &mseqFindPerimeter);
	mseqFindPerimeter.addChildren(&CruiseToPerimeter, &CruiseStopped, &dnWaitSignalOutside, &CruiseRotCW, &CruiseStopped);
	selFindPerimeter.addChildren(&mseqBumperActive, &eotBumpPeriActivated, &eotBumperActivated, &seqFindPerimeter, &mseqFindPerimeter);

	// ************************************
	// Perimeter Tracking
	//*************************************

	mseqBumperActive2.m_nodeName = (char*)"mseqBumperActive2";
	freeBumper2.m_nodeName = (char*)"freeBumper2";

	LFRotateCC105.m_nodeName = (char*)"LFRotateCC105";
	mseqTrackPerimeter.m_nodeName = (char*)"mseqTrackPerimeter";

	MotorStopFast.m_nodeName = (char*)"motorStopFast";
	mseqDockingStation.m_nodeName = (char*)"mseqDockingStation";
	perTrackChargingStationReached.m_nodeName = (char*)"perTrackChargingStationReached";
	checkOutsideAgain.m_nodeName = (char*)"checkOutsideAgain";

	driveCurve.m_nodeName = (char*)"driveCurve";
	setArc45CC.m_nodeName = (char*)"setArc45CC";
	mseqTrackPerimeterEscapeObst.m_nodeName = (char*)"mseqTrackPerimeterEscapeObst";
	mseqTrackPerimeterEscapeObst2.m_nodeName = (char*)"mseqTrackPerimeterEscapeObst2";

	mseqSecondReverse.m_nodeName = (char*)"mseqSecondReverse";


	mseqSecondReverse.addChildren(&conSecondReverse, &secondReverse3, &motorStop);

	mseqBumperActive2.addChildren(&conBumperActive, &MotorStopFast, &dnWaitFreeBumper2, &motorStop);

	mseqTrackPerimeterEscapeObst.addChildren(&conRotateAtPer, &setArc45CC, &mseqRotatePer);
	mseqTrackPerimeterEscapeObst2.addChildren(&conDriveCurve, &driveCurve, &motorStop, &checkOutsideAgain);


	//FLRotateCC.m_nodeName = "fLRotateCC";
	//LFRotateCW.m_nodeName = (char*)"LFRotateCW";
	findTriangle.m_nodeName = (char*)"findTriangle";
	parLineFollow.m_nodeName = (char*)"parLineFollow";


	//FLFailer.m_nodeName = (char*)"fLFailer";
	//mseqOvershootLeft.m_nodeName = "mseqOvershootLeft";
	//mseqOvershootRight.m_nodeName = "mseqOvershootRight";
	//selPerTracking.m_nodeName = (char*)"selPerTracking";
	//mselFollowLine.m_nodeName = "mselFollowLine";

	parLineFollow.addChildren(&lineFollow, &findTriangle);

	//mseqTrackPerimeter.addChildren(&parLineFollow, &motorStop, &LFRotateCW, &motorStop, &LFRotateCC105);

	mseqTrackPerimeter.addChildren(&parLineFollow, &motorStop, &LFRotateCC105);
	mseqDockingStation.addChildren(&conInDockingStation, &MotorStopFast, &dnWaitDockingStation, &perTrackChargingStationReached);

	//FLFailer.setChild(&motorStop);
	//mseqOvershootLeft.addChildren(&conStopOvershootLeft,&motorStop,&FLRotateCW,&FLFailer );
	//mseqOvershootRight.addChildren(&conStopOvershootRight,&motorStop,&FLRotateCC,&FLFailer );
	//mselFollowLine.addChildren(&mseqOvershootLeft,&mseqOvershootRight);
	//selFollowLine.addChildren(&mselFollowLine,&trackPerimeter );
	//selPerTracking.addChildren(&trackPerimeter );

	selPerimeterTracking.m_nodeName = (char*)"selPerimeterTracking";
	selPerimeterTracking.addChildren(&mseqDockingStation, &mseqBumperActive2, &mseqSecondReverse, &mseqTrackPerimeterEscapeObst, &mseqTrackPerimeterEscapeObst2, &mseqTrackPerimeter);


	// ************************************
	// Escape Obstacle Inside Perimeter
	//*************************************

	    //mseqEscRotate.m_nodeName = (char*)"mseqEscRotate";

	conditionFEONotFound.m_nodeName = (char*)"conditionFEONotFound";

	setArcFEO_ROT.m_nodeName = (char*)"setArcFEO_ROT";
	setArcFEO_ROTCW1.m_nodeName = (char*)"setArcFEO_ROTCW2";
	setArcFEO_ROTCC1.m_nodeName = (char*)"setArcFEO_ROTCC2";
	setArcFEO_ROTCW2.m_nodeName = (char*)"setArcFEO_ROTCW2";
	setArcFEO_ROTCC2.m_nodeName = (char*)"setArcFEO_ROTCC2";

	escRotateCC.m_nodeName = (char*)"escRotateCC";
	mseqEscRotateCW1.m_nodeName = (char*)"mseqEscRotateCW1";
	mseqEscRotateCW2.m_nodeName = (char*)"mseqEscRotateCW2";


	escRotateCW.m_nodeName = (char*)"escRotateCW";
	mseqEscRotateCC1.m_nodeName = (char*)"mseqEscRotateCC1";
	mseqEscRotateCC2.m_nodeName = (char*)"mseqEscRotateCC2";



	//forward20.m_nodeName = (char*)"forward20";
	//mseqEscForward.m_nodeName = (char*)"mseqEscForward";

	secondReverse.m_nodeName = (char*)"secondReverse";
	secondReverse2.m_nodeName = (char*)"secondReverse2";
	secondReverse3.m_nodeName = (char*)"secondReverse3";
	mseqEscBackward.m_nodeName = (char*)"mseqEscBackward";

	mselEscabeObstacle.m_nodeName = (char*)"mselEscabeObstacle";
	selEscabeObstacle1.m_nodeName = (char*)"selEscabeObstacle1";

	eotBumperActivated.m_nodeName = (char*)"eotBumperActivated";
	eotBumperActivated.setFlag(&bb.flagBumperInsidePerActivated);
	eotBumperActivated.setFlagToFalseOnTerminate();
	eotBumperActivated.setChild(&selEscabeObstacle1);


	//mseqEscRotate.addChildren(&conFEO_ROT,&mseqRotatePer);

	//mseqEscRotateCW.addChildren(&conFEO_ROTCW,&escRotateCW);
	//mseqEscRotateCC.addChildren(&conFEO_ROTCC,&escRotateCC);
	  //mseqEscForward.addChildren(&conFEO_FWD20,&forward20, &mseqRotatePer);

	mseqEscRotateCW1.addChildren(&conFEO_ROTCW1, &setArcFEO_ROTCW1, &mseqRotateBump);
	mseqEscRotateCC1.addChildren(&conFEO_ROTCC1, &setArcFEO_ROTCC1, &mseqRotateBump);

	mseqEscRotateCW2.addChildren(&conFEO_ROTCW2, &secondReverse, &setArcFEO_ROTCW2, &mseqRotateBump);
	mseqEscRotateCC2.addChildren(&conFEO_ROTCC2, &secondReverse, &setArcFEO_ROTCC2, &mseqRotateBump);
	mseqEscBackward.addChildren(&conFEO_ROT, &setArcFEO_ROT, &mseqRotateBump);
	mselEscabeObstacle.addChildren(&mseqEscBackward, &mseqEscRotateCC1, &mseqEscRotateCW1, &mseqEscRotateCC2, &mseqEscRotateCW2, &mseqPerimeterForwardInside, &conditionFEONotFound);
	selEscabeObstacle1.addChildren(&mselEscabeObstacle);


	// ************************************
	// Perimeter Outside Behaviour
	//*************************************


	preUpdateHistory.m_nodeName = (char*)"preUpdateHistory";
	calcAngle.m_nodeName = (char*)"calcAngle";
	rotateDriveBackInside.m_nodeName = (char*)"rotateDriveBackInside";
	//postUpdateHistory.m_nodeName = (char*)"postUpdateHistory";
    //rotatePer.m_nodeName = (char*)"rotatePer";


	mseqRotatePer.m_nodeName = (char*)"mseqRotatePer";
	mselRotatePer1.m_nodeName = (char*)"mselRotatePer1";   


	//mseqRotatePer.addChildren(&preUpdateHistory, &calcAngle, &rotateBothCoilsInside, &rotatePer, &postUpdateHistory);
	mselRotatePer1.addChildren(&rotateBothCoilsInside, &rotateDriveBackInside);
	mseqRotatePer.addChildren(&preUpdateHistory, &mselRotatePer1, &rotateX);


	// ************************************
	// Rotate
	//*************************************



	preUpdateHistoryBump.m_nodeName = (char*)"preUpdateHistoryBump";
	rotateX.m_nodeName = (char*)"rotateX";
	mseqRotateBump.m_nodeName = (char*)"mseqRotateBump";

	mseqRotateBump.addChildren(&preUpdateHistoryBump, &rotateX);

	// ************************************
	// BatLow and Raining Behaviour
	//*************************************

	cruiseBatLow.m_nodeName = (char*)"cruiseBatLow";
	cruiseRaining.m_nodeName = (char*)"cruiseRaining";

	seqMowBatLow.m_nodeName = (char*)"seqMowBatLow";
	seqRaining.m_nodeName = (char*)"seqRaining";

	seqMowBatLow.addChildren(&conBatLow, &cruiseBatLow);
	seqRaining.addChildren(&conRaining, &cruiseRaining);

	// ************************************
	// Check2 Behaviour
	//*************************************

	selCheck2.m_nodeName = (char*)"selCheck2";

	Check2CoilSignalAreaX.m_nodeName = (char*)"Check2CoilSignalAreaX";
	Check2LeftCoilSignal.m_nodeName = (char*)"Check2LeftCoilSignal";
	Check2RightCoilSignal.m_nodeName = (char*)"Check2RightCoilSignal";

	Check2PerSignal.m_nodeName = (char*)"check2PerSignal";
	Check2AllCoilsOutside.m_nodeName = (char*)"check2AllCoilsOutside";
	selCheck2.addChildren(&Check2CoilSignalAreaX, &Check2PerSignal, &Check2LeftCoilSignal, &Check2RightCoilSignal);
	//selCheck2.addChildren(&Check2PerSignal, &Check2AllCoilsOutside, &Check2BackCoilSignalAreaX, &Check2LeftCoilSignal, &Check2RightCoilSignal, &Check2BackCoilSignal);

    // ************************************
    // Cruise Behaviour
    //*************************************

	cruiseSpiral.m_nodeName = (char*)"cruiseSpiral";

	CruiseStartMowMotor.m_nodeName = (char*)"cruiseStartMowMotor";
	CruiseRotCW.m_nodeName = (char*)"cruiseRotCW";
	CruiseStopped.m_nodeName = (char*)"cruiseStopped";
	CruiseToPerimeter.m_nodeName = (char*)"cruiseToPerimeter";

	CruiseObstacleNear.m_nodeName = (char*)"cruiseObstacleNear";
	CruisePerimeterNear.m_nodeName = (char*)"cruisePerimeterNear";
	CruiseHighSpeed.m_nodeName = (char*)"cruiseHighSpeed";
	CruiseSpeedToMotor.m_nodeName = (char*)"cruiseSpeedToMotor";
	selCruiseSpeed.m_nodeName = (char*)"selCruiseSpeed";
	seqCruise.m_nodeName = (char*)"seqCruise";
	

	//selCruiseSpeed.addChildren(&CruiseObstacleNear, &CruiseHighSpeed);
	selCruiseSpeed.addChildren(&CruiseObstacleNear, &CruisePerimeterNear, &CruiseHighSpeed);
	seqCruise.addChildren(&selCruiseSpeed, &CruiseSpeedToMotor);

	eotSetbbShortWayCounter.m_nodeName = (char*)"eotSetbbShortWayCounter";
	eotSetbbShortWayCounter.setFlag(&bb.flagShortWayCounter);
	eotSetbbShortWayCounter.setChild(&seqCruise);

	// ************************************
	// Mowing
	//*************************************

	selMowing.m_nodeName = (char*)"selMowing";

	selMowing.addChildren(&CruiseStartMowMotor, &setflagCoilFirstOutsideLatched, &eotRestoreHistory, &mseqBumperActive, &eotBumpPeriActivated, &mseqPerimeterAvtive, &eotBumperActivated, &seqRaining, &seqMowBatLow, &Check2AllCoilsOutside, &eotCruiseSpiral, &eotSetbbShortWayCounter);

	// ************************************
	// Restore history
	//*************************************
	selRestoreHist.m_nodeName = (char*)"selRestoreHist";
	restoreHistory.m_nodeName = (char*)"restoreHistory";
	selRestoreHist.addChildren(&restoreHistory);


	// ************************************
	// Leave Head Charging Station Behaviour
	//*************************************
	selLeaveHeadCS.m_nodeName = (char*)"selLeaveHeadCS";
	mseqLeaveHeadCS.m_nodeName = (char*)"mseqLeaveHeadCS";
	driveBackXCS.m_nodeName = (char*)"driveBackXCS";
	driveForwardXCS.m_nodeName = (char*)"driveForwardXCS";
	setArcHeadStation_ROT.m_nodeName = (char*)"setArcHeadStation_ROT";

	mseqLeaveHeadCS.addChildren(&driveBackXCS, &arRotate90CC, &driveForwardXCS, &setArcHeadStation_ROT, &mseqRotateBump, &setMowBehaviour);
	selLeaveHeadCS.addChildren(&mseqLeaveHeadCS);

	// ************************************
	// Root
	//*************************************

	selRoot.m_nodeName = (char*)"rootSel";
	//selRoot.addChildren(&eotCharging, &selCheck2, &eotGotoAreaX,&eotPermeterTracking,&eotFindPerimeter,&eotMowing, &eotLeaveHeadChargingStation);
	selRoot.addChildren(&eotCharging, &eotLeaveHeadChargingStation, &selCheck2, &eotGotoAreaX, &eotPermeterTracking, &eotFindPerimeter, &eotMowing);
	//selRoot.addChildren(&eotCharging,&selCheck2,&eotGotoAreaX,&eotPermeterTracking,&eotFindPerimeter,&eotMowing);

	behaviorTree.setRootNode(&selRoot);

	// Run the setup of all nodes which have a setup function.
	behaviorTree.onSetup(bb);
}

void TBehaviour::loop() {
	behaviorTree.tick(bb);

}

