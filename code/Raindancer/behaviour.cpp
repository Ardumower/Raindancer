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
#include "bConditions.h"
#include "bDecorators.h"
#include "behaviour.h"
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
TConWasDirectionReverseObstacle conWasDirectionReverseObstacle ;

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

TdnCruiseSpiral dnCruiseSpiral;
TdnMowing dnMowing;
TdnPermeterTracking dnPermeterTracking;
TdnBumpPeriActivated dnBumpPeriActivated;
TdnCharging dnCharging;
TdnGotoAreaX dnGotoAreaX;
TdnFindPerimeter dnFindPerimeter;

TdnRestoreHistory dnRestoreHistory;

TdnLeaveHeadChargingStation dnLeaveHeadChargingStation;

WaitDecorator dnWaitDockingStation;
WaitDecorator dnWaitGotoArea;



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

TdnBumperActivated dnBumperActivated;
// ************************************
// Perimeter Outside Behaviour
//*************************************


TPreUpdateHistory preUpdateHistory;
TCalcAngle calcAngle;
//TPostUpdateHistory postUpdateHistory;
//TRotatePer rotatePer;



MemSequence mseqRotatePer;


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

TCruiseBatLow CruiseBatLow;
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
TdnSetbbShortWayCounter dnSetbbShortWayCounter;


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


void TBehaviour::reset()
{
	errorHandler.setInfo(F("TBehaviour::reset\r\n"));
	bb.resetBB();
    behaviorTree.reset(bb);
	bb.setBehaviour(BH_NONE);
}

void TBehaviour::setup()
{
    reset();


// ************************************
// Conditions Behaviour
//*************************************

    //conFEO_BACK180.nodeName = (char*)"conFEO_BACK180";
    //conFEO_FWD20.nodeName = (char*)"conFEO_FWD20";
    conFEO_ROTCC2.nodeName = (char*)"conFEO_ROTCC2";
    conFEO_ROTCW2.nodeName = (char*)"conFEO_ROTCW2";
	conFEO_ROTCC1.nodeName = (char*)"conFEO_ROTCC1";
	conFEO_ROTCW1.nodeName = (char*)"conFEO_ROTCW1";
    conFEO_ROT.nodeName = (char*)"conFEO_ROT";
    conFEO_BACKINSIDE.nodeName = (char*)"conFEO_BACKINSIDE";


	conWasDirectionFeoRotateCC.nodeName = (char*)"conWasDirectionFeoRotateCC";
	conWasDirectionFeoRotateCW.nodeName = (char*)"conWasDirectionFeoRotateCW";
	conWasDirectionFeoRotateCC1.nodeName = (char*)"conWasDirectionFeoRotateCC1";
	conWasDirectionFeoRotateCW1.nodeName = (char*)"conWasDirectionFeoRotateCW1";

    conWasDirectionForward.nodeName = (char*)"conWasDirectionForward";
    //conWasDirectionForward20.nodeName = (char*)"conWasDirectionForward20";
    conWasDirectionForwardInside.nodeName = (char*)"conWasDirectionForwardInside";
    conWasDirectionOverrun.nodeName = (char*)"conWasDirectionOverrun";
    conWasDirectionReverseObstacle.nodeName = (char*)"conWasDirectionReverseObstacle";
    conWasDirectionReverseInside.nodeName = (char*)"conWasDirectionReverseInside";
    conWasDirectionRotateCC.nodeName = (char*)"conWasDirectionRotateCC";
    conWasDirectionRotateCW.nodeName = (char*)"conWasDirectionRotateCW";
    //conNOTflagPerimeterActivated.nodeName = "conNOTflagPerimeterActivated";

    conBumperActive.nodeName = (char*)"conBumperActive";
    conPerOutside.nodeName = (char*)"conPerOutside"; //Condition

    //conStopOvershootLeft.nodeName = "conStopOvershootLeft";
    //conStopOvershootRight.nodeName = "conStopOvershootRight";
    conBatLow.nodeName= (char*)"conBatLow";
	conRaining.nodeName = (char*)"conRaining";

    conRightCoilOutside.nodeName= (char*)"conRightCoilOutside";
    conLeftCoilOutside.nodeName= (char*)"conLeftCoilOutside";

    conAreaReached.nodeName= (char*)"conAreaReached";

	conRotateAtPer.nodeName = (char*)"conRotateAtPer";
	conDriveCurve.nodeName = (char*)"conDriveCurve";
	conSecondReverse.nodeName = (char*)"conSecondReverse";

// ************************************
// Decorator Nodes Behaviour
//*************************************

    dnCruiseSpiral.nodeName= (char*)"dnCruiseSpiral";
    dnCruiseSpiral.setChild(&cruiseSpiral);
    
    dnBumpPeriActivated.nodeName= (char*)"dnBumpPeriActivated";

    dnBumpPeriActivated.setChild(&mselEscObstacleOutside);


    dnMowing.nodeName= (char*)"dnMowing";
    dnMowing.setChild(&selMowing);

    dnPermeterTracking.nodeName= (char*)"dnPermeterTracking";
    dnPermeterTracking.setChild(&selPerimeterTracking);

    dnCharging.nodeName= (char*)"dnCharging";
    dnCharging.setChild(&selCharging);

    dnGotoAreaX.nodeName= (char*)"dnGotoAreaX";
    dnGotoAreaX.setChild(&selGotoAreaX);

    dnFindPerimeter.nodeName= (char*)"dnFindPerimeter";
    dnFindPerimeter.setChild(&selFindPerimeter);

	conInDockingStation.nodeName = (char*)"conInDockingStation";

    dnWaitDockingStation.nodeName= (char*)"dnWaitDockingStation";
    dnWaitDockingStation.setChild(&conInDockingStation);
    dnWaitDockingStation.setWaitMillis(2000);

	dnRestoreHistory.nodeName = (char*)"dnRestoreHistory";
	dnRestoreHistory.setChild(&selRestoreHist);

    dnWaitGotoArea.nodeName= (char*)"dnWaitGotoArea";
    dnWaitGotoArea.setChild(&lineFollow);
    dnWaitGotoArea.setWaitMillis(2000);

	dnLeaveHeadChargingStation.nodeName = (char*)"dnLeaveHeadChargingStation";
	dnLeaveHeadChargingStation.setChild(&selLeaveHeadCS);

// ************************************
// Escape Obstacle Outside Perimeter
//*************************************


    mseqEscOOPReverse.nodeName = (char*)"mseqEscOOPReverse";
    mselEscObstacleOutside.nodeName = (char*)"mselEscObstacleOutside";
    setDD_FORWARD.nodeName = (char*)"setDD_FORWARD";
    //seqEscOOPRot.nodeName = (char*)"seqEscOOPRot";
    mseqEscOOPRotCC.nodeName = (char*)"mseqEscOOPRotCC";
    mseqEscOOPRotCW.nodeName = (char*)"mseqEscOOPRotCW";

    //seqEscOOPRot.addChildren(&mseqEscRotate,&setDD_FORWARD);
    mseqEscOOPRotCC.addChildren(&conFEO_ROTCC2, &secondReverse2, &setArc90CC,&mseqRotatePer, &setDD_FORWARD);
    mseqEscOOPRotCW.addChildren(&conFEO_ROTCW2, &secondReverse2, &setArc90CW, &mseqRotatePer, &setDD_FORWARD);

	mseqEscOOPReverse.addChildren(&conFEO_BACKINSIDE, &calcAngle, &mseqRotatePer);
    //mseqEscOOPReverse.addChildren(&conFEO_BACKINSIDE,&reverseInside,&reverseFurtherInside,&mseqRotatePer);
    mselEscObstacleOutside.addChildren(&mseqEscOOPReverse,&mseqEscOOPRotCW ,&mseqEscOOPRotCC, &conditionFEONotFound);


// ************************************
// Bumper Behaviour
//*************************************

	setflagBumperActivatedLR.nodeName = (char*)"setflagBumperActivatedLR";
    selEscapeAlgorithm.nodeName = (char*)"selEscapeAlgorithm";
    hardstop.nodeName = (char*)"hardstop";

    freeBumper.nodeName = (char*)"freeBumper";
    mseqBumperActive.nodeName = (char*)"mseqBumperActive";

    //mseqBumperActive.addChildren(&conBumperActive,&hardstop,&freeBumper,&selEscapeAlgorithm );
	mseqBumperActive.addChildren(&conBumperActive, &setflagBumperActivatedLR, &MotorStopFast, &freeBumper, &selEscapeAlgorithm);


// ************************************
// Free Perimeter Behaviour
//*************************************

	runTempService.nodeName = (char*)"runTempService";

	conditionPerimeterNotFound.nodeName = (char*)"conditionPerimeterNotFound";

	rotateBackCW.nodeName = (char*)"rotateBackCW";
	rotateBackCC.nodeName = (char*)"rotateBackCC";
	

	setflagCoilFirstOutsideLatched.nodeName = (char*)"setflagCoilFirstOutsideLatched";
	setflagCoilFirstOutside.nodeName = (char*)"setflagCoilFirstOutside";
    setFlagForceSmallRotAngle.nodeName = (char*)"flagForceSmallRotAngle";
    //mseqPerimeterForward20.nodeName = (char*)"mseqPerimeterForward20";

    mseqPerimeterForwardInside.nodeName = (char*)"mseqPerimeterForwardInside";

    setArc90CW.nodeName = (char*)"setArc90CW";
    setArc90CC.nodeName = (char*)"setArc90CC";
	setArc90CW1.nodeName = (char*)"setArc90CW1";
	setArc90CC1.nodeName = (char*)"setArc90CC1";
    mseqPRRCO.nodeName = (char*)"mseqPRRCO";
    mseqPRLCO.nodeName = (char*)"mseqPRLCO";
    mselPerimeterReverse.nodeName = (char*)"mseqPRLCO";


    motorStop.nodeName = (char*)"motorStop";

	mseqPerimeterFeoRotCC.nodeName = (char*)"mseqPerimeterFeoRotCC";
	mseqPerimeterFeoRotCW.nodeName = (char*)"mseqPerimeterFeoRotCW";
	mseqPerimeterFeoRotCC1.nodeName = (char*)"mseqPerimeterFeoRotCC1";
	mseqPerimeterFeoRotCW1.nodeName = (char*)"mseqPerimeterFeoRotCW1";
//    perRotateInsideCW.nodeName = (char*)"perRotateInsideCW";
//    perRotateInsideCC.nodeName = (char*)"perRotateInsideCC";
	setArc20CW.nodeName = (char*)"setArc20CW";
	setArc20CC.nodeName = (char*)"setArc20CC";
    mseqPerimeterRotCC.nodeName = (char*)"mseqPerimeterRotCC";
    mseqPerimeterRotCW.nodeName = (char*)"mseqPerimeterRotCW";


    overRun.nodeName = (char*)"overRun";

    reverseFurtherInside.nodeName = (char*)"reverseFurtherInside";
	perDriveBack.nodeName = (char*)"perDriveBack";

	conOneCoilOutside.nodeName = (char*)"conOneCoilOutside";
	conBothCoilsOutside.nodeName = (char*)"conBothCoilsOutside";
    
	reverseInsideError.nodeName = (char*)"reverseInsideError";
	mseqPerimeterRevInside.nodeName = (char*)"mseqPerimeterRevInside";

	directionOverrunError.nodeName = (char*)"directionOverrunError";
    mseqPerimeterOverrun.nodeName = (char*)"seqPerimeterOverrun";
	
    mseqPerimeterForward.nodeName = (char*)"mseqPerimeterForward";
    mselPerimeterActive.nodeName = (char*)"mselPerimeterActive";
    mseqPerimeterAvtive.nodeName = (char*)"mseqPerimeterAvtive";

    forwardInside.nodeName = (char*)"forwardInside";
	forwardInsideError.nodeName = (char*) "forwardInsideError";
    mseqPerimeterReverse.nodeName = (char*)"mseqPerimeterReverse";

	rotateBothCoilsInside.nodeName = (char*)"rotateBothCoilsInside";



    mseqPerimeterRevInside.addChildren(&conWasDirectionReverseInside, &motorStop,&reverseInsideError);

    mseqPerimeterOverrun.addChildren(&conWasDirectionOverrun, &motorStop, &directionOverrunError);

    mseqPerimeterForwardInside.addChildren(&conWasDirectionForwardInside,&motorStop,&forwardInsideError);

    //Reverse
    mseqPRRCO.addChildren(&conRightCoilOutside,&motorStop, &setArc90CC, &mseqRotatePer, &forwardInside);
    mseqPRLCO.addChildren(&conLeftCoilOutside,&motorStop, &setArc90CW, &mseqRotatePer, &forwardInside);
    mselPerimeterReverse.addChildren(&mseqPRLCO,&mseqPRRCO, &forwardInside);

    mseqPerimeterReverse.addChildren(&conWasDirectionReverseObstacle,&motorStop,&mselPerimeterReverse);

	mseqPerimeterFeoRotCW.addChildren(&conWasDirectionFeoRotateCW, &motorStop, &rotateBackCC, &setArc90CC1, &mseqRotatePer);
	mseqPerimeterFeoRotCC.addChildren(&conWasDirectionFeoRotateCC, &motorStop, &rotateBackCW, &setArc90CW1, &mseqRotatePer);

	mseqPerimeterFeoRotCW1.addChildren(&conWasDirectionFeoRotateCW1, &motorStop, &rotateBackCC, &secondReverse2, &setArc90CC,&mseqRotatePer);
	mseqPerimeterFeoRotCC1.addChildren(&conWasDirectionFeoRotateCC1, &motorStop, &rotateBackCW, & secondReverse2, &setArc90CW, &mseqRotatePer);

    mseqPerimeterRotCC.addChildren(&conWasDirectionRotateCC,&motorStop,&setArc20CC, &mseqRotatePer, &setDD_FORWARD);
    mseqPerimeterRotCW.addChildren(&conWasDirectionRotateCW,&motorStop,&setArc20CW, &mseqRotatePer, &setDD_FORWARD);

    //mseqPerimeterRotCC.addChildren(&conWasDirectionRotateCC,&motorStop,&reverseInside,&reverseFurtherInside, &seqRotatePer);
    //mseqPerimeterRotCW.addChildren(&conWasDirectionRotateCW,&motorStop,&reverseInside,&reverseFurtherInside, &seqRotatePer);

//	mseqPerimeterForward.addChildren(&conWasDirectionForward, &motorStop, &mseqReverseInside, &reverseFurtherInside, &seqRotatePer);
//	mseqPerimeterForward.addChildren(&conWasDirectionForward, &overRun, &mselPerimeterForward, &seqRotatePer);


  mseqPerimeterForward.addChildren(&conWasDirectionForward,&overRun,  &runTempService, &perDriveBack, &calcAngle,&mseqRotatePer);
	mselPerimeterActive.addChildren(&mseqPerimeterForward, &mseqPerimeterFeoRotCC, &mseqPerimeterFeoRotCW, &mseqPerimeterFeoRotCW1, &mseqPerimeterFeoRotCC1, &mseqPerimeterRotCC,&mseqPerimeterRotCW,&mseqPerimeterReverse,&mseqPerimeterForwardInside, &mseqPerimeterOverrun, &mseqPerimeterRevInside,&conditionPerimeterNotFound);

  mseqPerimeterAvtive.addChildren(&conPerOutside,&setflagCoilFirstOutside, &mselPerimeterActive);


// ************************************
// GoTo Area Behaviour
//*************************************
    selGotoAreaX.nodeName = (char*)"selGotoAreaX";
    mseqGotoAreaX.nodeName = (char*)"mseqGotoAreaX";
	lineFollow.nodeName = (char*)"lineFollow";
    arRotate90CC.nodeName = (char*)"arRotate90CC";
    setMowBehaviour.nodeName = (char*)"setMowBehaviour";

    mseqGotoAreaX.addChildren(&conAreaReached,&motorStop,&arRotate90CC,&setMowBehaviour);
    selGotoAreaX.addChildren(&CruiseStartMowMotor,&mseqBumperActive2, &mseqSecondReverse, &mseqTrackPerimeterEscapeObst, &mseqTrackPerimeterEscapeObst2, &mseqGotoAreaX,&dnWaitGotoArea);

// ************************************
// Charging Behaviour
//*************************************

    selCharging.nodeName = (char*)"selCharging";
    chargeRelayOn.nodeName = (char*)"chargeRelayOn";
    selCharging.addChildren(&chargeRelayOn);



// ************************************
// Find Perimeter Behaviour
//*************************************

    selFindPerimeter.nodeName = (char*)"selCharging";
	seqFindPerimeter.nodeName = (char*)"seqFindPerimeter",
    mseqFindPerimeter.nodeName= (char*)"mseqFindPerimeter";
	conIsOutsidePerimeter.nodeName = (char*)"conIsOutsidePerimeter";

	dnWaitSignalOutside.nodeName = (char*)"dnWaitSignalOutside";
	dnWaitSignalOutside.setChild(&conIsOutsidePerimeter);
	dnWaitSignalOutside.setWaitMillis(2000);

	seqFindPerimeter.addChildren(&selCruiseSpeed, &mseqFindPerimeter);
    mseqFindPerimeter.addChildren(&CruiseToPerimeter,&CruiseStopped, &dnWaitSignalOutside, &CruiseRotCW, &CruiseStopped);
    selFindPerimeter.addChildren(&mseqBumperActive, &dnBumpPeriActivated, &dnBumperActivated, &seqFindPerimeter, &mseqFindPerimeter);

// ************************************
// Perimeter Tracking
//*************************************

	mseqBumperActive2.nodeName = (char*)"mseqBumperActive2";
	freeBumper2.nodeName = (char*)"freeBumper2";
	
	LFRotateCC105.nodeName = (char*)"LFRotateCC105";
	mseqTrackPerimeter.nodeName = (char*)"mseqTrackPerimeter";

    MotorStopFast.nodeName = (char*)"motorStopFast";
    mseqDockingStation.nodeName = (char*)"mseqDockingStation";
    perTrackChargingStationReached.nodeName = (char*)"perTrackChargingStationReached";
	checkOutsideAgain.nodeName = (char*)"checkOutsideAgain";

	driveCurve.nodeName = (char*)"driveCurve";
	setArc45CC.nodeName = (char*)"setArc45CC";
	mseqTrackPerimeterEscapeObst.nodeName = (char*)"mseqTrackPerimeterEscapeObst";
	mseqTrackPerimeterEscapeObst2.nodeName = (char*)"mseqTrackPerimeterEscapeObst2";

	mseqSecondReverse.nodeName = (char*)"mseqSecondReverse";


	mseqSecondReverse.addChildren(&conSecondReverse, &secondReverse3, &motorStop);

	mseqBumperActive2.addChildren(&conBumperActive, &MotorStopFast, &freeBumper2, &motorStop);

	mseqTrackPerimeterEscapeObst.addChildren(&conRotateAtPer,&setArc45CC,&mseqRotatePer);
	mseqTrackPerimeterEscapeObst2.addChildren(&conDriveCurve,&driveCurve, &motorStop,&checkOutsideAgain);
	

    //FLRotateCC.nodeName = "fLRotateCC";
    //LFRotateCW.nodeName = (char*)"LFRotateCW";
	findTriangle.nodeName = (char*)"findTriangle";
	parLineFollow.nodeName = (char*)"parLineFollow";


    //FLFailer.nodeName = (char*)"fLFailer";
    //mseqOvershootLeft.nodeName = "mseqOvershootLeft";
    //mseqOvershootRight.nodeName = "mseqOvershootRight";
    //selPerTracking.nodeName = (char*)"selPerTracking";
    //mselFollowLine.nodeName = "mselFollowLine";

	parLineFollow.addChildren(&lineFollow,&findTriangle);

	//mseqTrackPerimeter.addChildren(&parLineFollow, &motorStop, &LFRotateCW, &motorStop, &LFRotateCC105);

	mseqTrackPerimeter.addChildren(&parLineFollow, &motorStop, &LFRotateCC105);
	mseqDockingStation.addChildren(&conInDockingStation,&MotorStopFast,&dnWaitDockingStation, &perTrackChargingStationReached);

    //FLFailer.setChild(&motorStop);
    //mseqOvershootLeft.addChildren(&conStopOvershootLeft,&motorStop,&FLRotateCW,&FLFailer );
    //mseqOvershootRight.addChildren(&conStopOvershootRight,&motorStop,&FLRotateCC,&FLFailer );
    //mselFollowLine.addChildren(&mseqOvershootLeft,&mseqOvershootRight);
    //selFollowLine.addChildren(&mselFollowLine,&trackPerimeter );
    //selPerTracking.addChildren(&trackPerimeter );

    selPerimeterTracking.nodeName = (char*)"selPerimeterTracking";
    selPerimeterTracking.addChildren(&mseqBumperActive2,&mseqDockingStation, &mseqSecondReverse, &mseqTrackPerimeterEscapeObst,&mseqTrackPerimeterEscapeObst2,&mseqTrackPerimeter);


// ************************************
// Escape Obstacle Inside Perimeter
//*************************************

    //mseqEscRotate.nodeName = (char*)"mseqEscRotate";

	conditionFEONotFound.nodeName = (char*)"conditionFEONotFound";

	setArcFEO_ROT.nodeName = (char*)"setArcFEO_ROT";
	setArcFEO_ROTCW1.nodeName = (char*)"setArcFEO_ROTCW2";
	setArcFEO_ROTCC1.nodeName = (char*)"setArcFEO_ROTCC2";
	setArcFEO_ROTCW2.nodeName = (char*)"setArcFEO_ROTCW2";
	setArcFEO_ROTCC2.nodeName = (char*)"setArcFEO_ROTCC2";

    escRotateCC.nodeName = (char*)"escRotateCC";
	mseqEscRotateCW1.nodeName = (char*)"mseqEscRotateCW1";
	mseqEscRotateCW2.nodeName = (char*)"mseqEscRotateCW2";


    escRotateCW.nodeName = (char*)"escRotateCW";
	mseqEscRotateCC1.nodeName = (char*)"mseqEscRotateCC1";
    mseqEscRotateCC2.nodeName = (char*)"mseqEscRotateCC2";



    //forward20.nodeName = (char*)"forward20";
    //mseqEscForward.nodeName = (char*)"mseqEscForward";

	secondReverse.nodeName = (char*)"secondReverse";
	secondReverse2.nodeName = (char*)"secondReverse2";
	secondReverse3.nodeName = (char*)"secondReverse3";
    mseqEscBackward.nodeName = (char*)"mseqEscBackward";

    mselEscabeObstacle.nodeName = (char*)"mselEscabeObstacle";
    selEscabeObstacle1.nodeName = (char*)"selEscabeObstacle1";
    dnBumperActivated.nodeName = (char*)"dnBumperActivated";


    //mseqEscRotate.addChildren(&conFEO_ROT,&mseqRotatePer);

    //mseqEscRotateCW.addChildren(&conFEO_ROTCW,&escRotateCW);
    //mseqEscRotateCC.addChildren(&conFEO_ROTCC,&escRotateCC);
	//mseqEscForward.addChildren(&conFEO_FWD20,&forward20, &mseqRotatePer);

	mseqEscRotateCW1.addChildren(&conFEO_ROTCW1, &setArcFEO_ROTCW1, &mseqRotateBump);
	mseqEscRotateCC1.addChildren(&conFEO_ROTCC1, &setArcFEO_ROTCC1, &mseqRotateBump);

	mseqEscRotateCW2.addChildren(&conFEO_ROTCW2, &secondReverse, &setArcFEO_ROTCW2, &mseqRotateBump);
	mseqEscRotateCC2.addChildren(&conFEO_ROTCC2, &secondReverse, &setArcFEO_ROTCC2, &mseqRotateBump);
    mseqEscBackward.addChildren(&conFEO_ROT, &setArcFEO_ROT, &mseqRotateBump);
    mselEscabeObstacle.addChildren(&mseqEscBackward, &mseqEscRotateCC1, &mseqEscRotateCW1, &mseqEscRotateCC2 ,&mseqEscRotateCW2, &mseqPerimeterForwardInside,&conditionFEONotFound) ;
	selEscabeObstacle1.addChildren(&mselEscabeObstacle);
    dnBumperActivated.setChild(&selEscabeObstacle1);

    // ************************************
    // Perimeter Outside Behaviour
    //*************************************


	preUpdateHistory.nodeName = (char*)"preUpdateHistory";
	calcAngle.nodeName = (char*)"calcAngle";
	//postUpdateHistory.nodeName = (char*)"postUpdateHistory";
    //rotatePer.nodeName = (char*)"rotatePer";


    mseqRotatePer.nodeName = (char*)"mseqRotatePer";


    //mseqRotatePer.addChildren(&preUpdateHistory, &calcAngle, &rotateBothCoilsInside, &rotatePer, &postUpdateHistory);
	mseqRotatePer.addChildren(&preUpdateHistory, &rotateBothCoilsInside, &rotateX);


	// ************************************
	// Rotate
	//*************************************



	preUpdateHistoryBump.nodeName = (char*)"preUpdateHistoryBump";
	rotateX.nodeName = (char*)"rotateX";
	mseqRotateBump.nodeName = (char*)"mseqRotateBump";
	
	mseqRotateBump.addChildren(&preUpdateHistoryBump,&rotateX);

    // ************************************
    // BatLow and Raining Behaviour
    //*************************************

    CruiseBatLow.nodeName= (char*)"cruiseBatLow";
    seqMowBatLow.nodeName= (char*)"seqMowBatLow";
	seqRaining.nodeName = (char*)"seqRaining";

    seqMowBatLow.addChildren( &conBatLow, &CruiseBatLow);
	seqRaining.addChildren(&conRaining, &CruiseBatLow);

    // ************************************
    // Check2 Behaviour
    //*************************************

    selCheck2.nodeName = (char*)"selCheck2";

	Check2CoilSignalAreaX.nodeName = (char*)"Check2CoilSignalAreaX";
    Check2LeftCoilSignal.nodeName = (char*)"Check2LeftCoilSignal";
    Check2RightCoilSignal.nodeName = (char*)"Check2RightCoilSignal";

    Check2PerSignal.nodeName = (char*)"check2PerSignal";
    Check2AllCoilsOutside.nodeName = (char*)"check2AllCoilsOutside";
    selCheck2.addChildren( &Check2PerSignal, &Check2CoilSignalAreaX, &Check2LeftCoilSignal,&Check2RightCoilSignal);
	//selCheck2.addChildren(&Check2PerSignal, &Check2AllCoilsOutside, &Check2BackCoilSignalAreaX, &Check2LeftCoilSignal, &Check2RightCoilSignal, &Check2BackCoilSignal);

    // ************************************
    // Cruise Behaviour
    //*************************************

    cruiseSpiral.nodeName = (char*)"cruiseSpiral";
    
    CruiseStartMowMotor.nodeName = (char*)"cruiseStartMowMotor";
    CruiseRotCW.nodeName= (char*)"cruiseRotCW";
    CruiseStopped.nodeName= (char*)"cruiseStopped";
    CruiseToPerimeter.nodeName= (char*)"cruiseToPerimeter";

    CruiseObstacleNear.nodeName= (char*)"cruiseObstacleNear";
    CruisePerimeterNear.nodeName= (char*)"cruisePerimeterNear";
    CruiseHighSpeed.nodeName= (char*)"cruiseHighSpeed";
    CruiseSpeedToMotor.nodeName= (char*)"cruiseSpeedToMotor";
    selCruiseSpeed.nodeName= (char*)"selCruiseSpeed";
    seqCruise.nodeName= (char*)"seqCruise";
    dnSetbbShortWayCounter.nodeName= (char*)"dnSetbbShortWayCounter";

	//selCruiseSpeed.addChildren(&CruiseObstacleNear, &CruiseHighSpeed);
    selCruiseSpeed.addChildren( &CruiseObstacleNear, &CruisePerimeterNear, &CruiseHighSpeed);
    seqCruise.addChildren(&selCruiseSpeed,&CruiseSpeedToMotor);
    dnSetbbShortWayCounter.setChild(&seqCruise);

    // ************************************
    // Mowing
    //*************************************

    selMowing.nodeName = (char*)"selMowing";

    selMowing.addChildren(&CruiseStartMowMotor, &setflagCoilFirstOutsideLatched, &dnRestoreHistory, &mseqBumperActive, &dnBumpPeriActivated,  &mseqPerimeterAvtive,  &dnBumperActivated, &seqRaining, &seqMowBatLow, &Check2AllCoilsOutside, &dnCruiseSpiral, &dnSetbbShortWayCounter);

	// ************************************
	// Restore history
	//*************************************
	selRestoreHist.nodeName = (char*)"selRestoreHist";
	restoreHistory.nodeName = (char*)"restoreHistory";
	selRestoreHist.addChildren(&restoreHistory);


	// ************************************
	// Leave Head Charging Station Behaviour
	//*************************************
	selLeaveHeadCS.nodeName = (char*)"selLeaveHeadCS";
	mseqLeaveHeadCS.nodeName = (char*)"mseqLeaveHeadCS";
	driveBackXCS.nodeName = (char*)"driveBackXCS";
	driveForwardXCS.nodeName = (char*)"driveForwardXCS";
	setArcHeadStation_ROT.nodeName = (char*)"setArcHeadStation_ROT";

	mseqLeaveHeadCS.addChildren(&driveBackXCS, &arRotate90CC, &driveForwardXCS, &setArcHeadStation_ROT, &mseqRotateBump, &setMowBehaviour);
	selLeaveHeadCS.addChildren(&mseqLeaveHeadCS);
	
    // ************************************
    // Root
    //*************************************

    selRoot.nodeName = (char*)"rootSel";
	//selRoot.addChildren(&dnCharging, &selCheck2, &dnGotoAreaX,&dnPermeterTracking,&dnFindPerimeter,&dnMowing, &dnLeaveHeadChargingStation);
    selRoot.addChildren(&dnCharging, &dnLeaveHeadChargingStation, &selCheck2, &dnGotoAreaX,&dnPermeterTracking,&dnFindPerimeter,&dnMowing );
	//selRoot.addChildren(&dnCharging,&selCheck2,&dnGotoAreaX,&dnPermeterTracking,&dnFindPerimeter,&dnMowing);

    behaviorTree.setRootNode (&selRoot);
}

void TBehaviour::loop()
{
    behaviorTree.tick(bb);

}

