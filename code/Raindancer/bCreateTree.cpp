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


#include "BehaviourTree.h"
#include "bCreateTree.h"
#include "errorhandler.h"
#include "bServices.h"
#include "bCruise.h"
#include "bConditions.h"
#include "bPerimeter.h"
#include "bRotate.h"
#include "bBumper.h"
#include "bCharging.h"
#include "bCheck2.h"
#include "bHeadStation.h"
#include "bPerimeterTracking.h"

class TConAlwaysFailure : public Condition    // Condition
{
private:

public:

	TConAlwaysFailure() {
		m_nodeName = const_cast<char*>("conAlwaysFailure");
	}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {
		return BH_FAILURE;
	}
};


class TConAlwaysSuccess : public Condition    // Condition
{
private:

public:

	TConAlwaysSuccess() {
		m_nodeName = const_cast<char*>("conAlwaysSuccess");
	}

	virtual NodeStatus onCheckCondition(Blackboard& bb) {
		return BH_SUCCESS;
	}
};

TConAlwaysFailure alwaysFailure;
TConAlwaysSuccess alwaysSuccess;


// ************************************
// Conditions 
//*************************************

TConPerOutside conPerOutside;
TConIsDirectionForward conIsDirectionForward;
TConBothCoilsOutside conBothCoilsOutside;
TConRightCoilOutside conRightCoilOutside;
TConLeftCoilOutside conLeftCoilOutside;
TConIsDirectionRotateCC conIsDirectionRotateCC;
TConIsDirectionRotateCW conIsDirectionRotateCW;
TConBumperActive conBumperActive;
TConditionBumperNotFound  conditionBumperNotFound;
TConditionPerimeterNotFound conditionPerimeterNotFound;
TConIsDirectionRotateCCorCW conIsDirectionRotateCCorCW;
TconLCO_and_DDCC conLCO_and_DDCC;
TconRCO_and_DDCW conRCO_and_DDCW;
TConIsOutsidePerimeter conIsOutsidePerimeter;
TConInDockingStation conInDockingStation;
TconAreaReached conAreaReached;

// ************************************
// Services
//*************************************

TsrvUpdateHistory srvUpdateHistory;


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
// Decorator Nodes Behaviour
//*************************************

ExecuteOnTrue eotCruiseSpiral;
ExecuteOnTrue eotMowing;
ExecuteOnTrue eotPermeterTracking;
ExecuteOnTrue eotCharging;
ExecuteOnTrue eotGotoAreaX;
ExecuteOnTrue eotFindPerimeter;
ExecuteOnTrue eotRestoreHistory;
ExecuteOnTrue eotLeaveHeadChargingStation;

WaitDecorator dnWaitDockingStation;
WaitDecorator dnWaitGotoArea;
WaitDecorator dnWaitFreeBumper2;

// ************************************
// Selector Nodes 
//*************************************
Selector selMowing;



// ************************************
// Find Perimeter Behaviour
//*************************************
TCruiseRotCW CruiseRotCW;
WaitDecorator dnWaitSignalOutside;
TCheckChargStationDisabled checkChargStationDisabled;
Inverter invConPerOutside;
Selector selFindPerimeter2;
MemSequence mseqFindPerimeter3;
Monitor monFindPerimeter1;
MemSelector mselFindPerimeter0;


// ************************************
// Perimeter Tracking
//*************************************

TcheckOutsideAgain checkOutsideAgain;
TdriveCurve driveCurve;
TSetArc45CC setArc45CC;
Monitor monPTBumperActive3;
MemSequence mseqPTBumperActive6;

Inverter invConBumperActive;

MemSequence mseqPTBumperActive5;
MemSequence mseqPTBumperActive4;
MemSelector mselPTBumperActive2;
MemSequence mseqPTBumperActive1;

TfindTriangle findTriangle;
TlineFollow lineFollow;
TLFRotateCC105 LFRotateCC105;
Parallel parLineFollow;
MemSequence mseqTrackPerimeter;
Monitor monTrackPerimeter;

TperTrackChargingStationReached perTrackChargingStationReached;
MemSequence mseqDockingStation;
Selector selPerimeterTracking;


// ************************************
// Goto Area Behaviour
//*************************************

MemSequence mseqGotoAreaX;
Selector selGotoAreaX1;
Monitor monGotoAreaX;
Selector selGotoAreaX;

// ************************************
// Rotate 
//*************************************
TDriveHist0    driveHist0;
TDriveBumperFree driveBumperFree;


// ************************************
// BatLow and raining Behaviour
//*************************************
TacCruiseBatLow acCruiseBatLow;
TacCruiseRaining acCruiseRaining;

// ************************************
// Cruise Behaviour
//*************************************

TCheckOscillating checkOscillating;
TcruiseSpeedToMotor cruiseSpeedToMotor;
TcruiseStartMowMotor cruiseStartMowMotor;


// ************************************
// Bumper Behaviour
//*************************************

MemSequence mseqRotateBump8;
MemSequence mseqRotateBump7;
Selector selFreeBumper6;
MemSequence mseqRotateBump5;
TCalcBumpAngleSameDirection calcBumpAngleSameDir;
MemSequence mseqRotateBump4;
Selector selFreeBumper2;

TBumperDriveBack bumperDriveBack;
TCalcBumpAngleOpposDir calcBumpAngleOpposDir;
MemSequence mseqFreeBumper3;
MemSequence mseqFreeBumper1;
MemSelector mselFreeBumper;
TSetflagBumperActivatedLR setflagBumperActivatedLR;
TMotorStopFast MotorStopFast;
MemSequence mseqBumperActive;



// ************************************
// Perimeter Behaviour
//*************************************
TSetflagCoilFirstOutside setflagCoilFirstOutside;
TInvertRotationDirection invertRotationDirection;
TRestoreLastRotation restoreLastRotation;
TErrorBothCoilsOutside errorBothCoilsOutside;
TMotorStop motorStop;
TCalcAngle calcAngle;
TPerDriveBack perDriveBack;
TOverRun overRun;
TRunTempService runTempService;

MemSequence mseqRotatePer4;
MemSequence mseqRotatePer3;
MemSequence mseqRotatePer2;
MemSequence mseqRotatePer1;
Selector selRotatePer;
MemSequence mseqPerimeterForward;
MemSequence mseqPerimeterActive;
MemSelector mselPerimeterActive;


// ************************************
// Charging Behaviour
//*************************************

Selector selCharging;
TchargeRelayOn chargeRelayOn;


// ************************************
// Leave Head Charging Station Behaviour
//*************************************
Selector selLeaveHeadCS;
MemSequence mseqLeaveHeadCS;
TdriveBackXCS driveBackXCS;
TRrotate90CC rotate90CC;
TdriveForwardXCS driveForwardXCS;
TSetArcHeadStation_ROT setArcHeadStation_ROT;
TsetMowBehaviour setMowBehaviour;

void TBehaviour::reset() {
	errorHandler.setInfo(F("TBehaviour::reset\r\n"));
	bb.resetBB();
	behaviorTree.reset(bb);
	bb.setBehaviour(BH_NONE);
}

void TBehaviour::setup() {
	reset();



	// ************************************
	// Conditions 
	//*************************************
	conPerOutside.m_nodeName = const_cast<char*>("conPerOutside"); //Condition
	conIsDirectionForward.m_nodeName = (char*)"conWasDirectionForward";
	conBothCoilsOutside.m_nodeName = (char*)"conBothCoilsOutside";
	conRightCoilOutside.m_nodeName = (char*)"conRightCoilOutside";
	conLeftCoilOutside.m_nodeName = (char*)"conLeftCoilOutside";
	conIsDirectionRotateCC.m_nodeName = (char*)"conIsDirectionRotateCC";
	conIsDirectionRotateCW.m_nodeName = (char*)"conIsDirectionRotateCW";
	conBumperActive.m_nodeName = (char*)"conBumperActive";
	conditionBumperNotFound.m_nodeName = (char*)"conditionBumperNotFound";
	conditionPerimeterNotFound.m_nodeName = (char*)"conditionPerimeterNotFound";
	conIsDirectionRotateCCorCW.m_nodeName = (char*)"conIsDirectionRotateCCorCW";
	conLCO_and_DDCC.m_nodeName = (char*)"conLRCO_and_DDCC";
	conRCO_and_DDCW.m_nodeName = (char*)"conRCO_and_DDCW";
	conIsOutsidePerimeter.m_nodeName = (char*)"conIsOutsidePerimeter";
	conInDockingStation.m_nodeName = (char*)"conInDockingStation";
	conAreaReached.m_nodeName = (char*)"conAreaReached";

	// ************************************
	// Services
	//*************************************
	srvUpdateHistory.setInterval(0);

	// ************************************
	// Decorator Nodes Behaviour
	//*************************************

	eotCruiseSpiral.m_nodeName = (char*)"eotCruiseSpiral";
	eotCruiseSpiral.setFlag(&bb.flagCruiseSpiral);
	eotCruiseSpiral.setChild(&alwaysFailure);

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
	eotFindPerimeter.setChild(&mselFindPerimeter0);


	eotLeaveHeadChargingStation.m_nodeName = (char*)"eotLeaveHeadChargingStation";
	eotLeaveHeadChargingStation.setFlag(&bb.flagEnableLeaveHeadChargingStation);
	eotLeaveHeadChargingStation.setChild(&selLeaveHeadCS);


	dnWaitDockingStation.m_nodeName = (char*)"dnWaitDockingStation";
	dnWaitDockingStation.setChild(&conInDockingStation);
	dnWaitDockingStation.setWaitMillis(2000);

	dnWaitGotoArea.m_nodeName = (char*)"dnWaitGotoArea";
	dnWaitGotoArea.setChild(&lineFollow);
	dnWaitGotoArea.setWaitMillis(2000);


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


	// ************************************
	// BatLow and Raining Behaviour
	//*************************************
	checkOscillating.m_nodeName = (char*)"acCruiseBatLow";
	acCruiseBatLow.m_nodeName = (char*)"acCruiseBatLow";
	acCruiseRaining.m_nodeName = (char*)"acCruiseRaining";

	// ************************************
	// Cruise Behaviour
	//*************************************
	cruiseSpeedToMotor.m_nodeName = (char*)"cruiseSpeedToMotor";
	cruiseStartMowMotor.m_nodeName = (char*)"cruiseStartMowMotor";


	// ************************************
	// Bumper Behaviour
	//*************************************
	mseqRotateBump8.m_nodeName = (char*)"mseqRotateBump8";
	mseqRotateBump7.m_nodeName = (char*)"mseqRotateBump7";
	calcBumpAngleSameDir.m_nodeName = (char*)"calcBumpAngleSameDir";
	selFreeBumper6.m_nodeName = (char*)"selFreeBumper6";
	mseqRotateBump5.m_nodeName = (char*)"mseqRotateBump5";
	mseqRotateBump4.m_nodeName = (char*)"mseqRotateBump4";
	selFreeBumper2.m_nodeName = (char*)"selFreeBumper2";
	bumperDriveBack.m_nodeName = (char*)"bumperDriveBack";
	calcBumpAngleOpposDir.m_nodeName = (char*)"calcBumpAngleOpposDir";
	mseqFreeBumper3.m_nodeName = (char*)"mseqFreeBumper3";
	mseqFreeBumper1.m_nodeName = (char*)"mseqFreeBumper1";
	mselFreeBumper.m_nodeName = (char*)"mselFreeBumper";
	setflagBumperActivatedLR.m_nodeName = (char*)"setflagBumperActivatedLR";
	MotorStopFast.m_nodeName = (char*)"motorStopFast";
	mseqBumperActive.m_nodeName = (char*)"mseqBumperActive";

	mseqRotateBump8.addChildren(&conPerOutside, &setflagCoilFirstOutside, &motorStop, &restoreLastRotation, &calcBumpAngleOpposDir, &driveHist0);
	mseqRotateBump7.addChildren(&conPerOutside, &setflagCoilFirstOutside, &motorStop, &restoreLastRotation, &bumperDriveBack, &calcBumpAngleOpposDir, &driveHist0);
	mseqRotateBump5.addChildren(&conBumperActive, &setflagBumperActivatedLR, &MotorStopFast, &restoreLastRotation, &calcBumpAngleOpposDir, &selFreeBumper2);
	selFreeBumper6.addChildren(&mseqRotateBump5, &mseqRotateBump8, &driveHist0);
	mseqRotateBump4.addChildren(&conBumperActive, &setflagBumperActivatedLR, &MotorStopFast, &restoreLastRotation, &bumperDriveBack, &calcBumpAngleOpposDir, &driveHist0);
	mseqFreeBumper3.addChildren(&conIsDirectionForward, &setflagBumperActivatedLR, &MotorStopFast, &bumperDriveBack, &calcBumpAngleSameDir, &selFreeBumper6);
	selFreeBumper2.addChildren(&mseqRotateBump4, &mseqRotateBump7, &driveHist0);
	mseqFreeBumper1.addChildren(&conIsDirectionRotateCCorCW, &setflagBumperActivatedLR, &MotorStopFast, &restoreLastRotation, &calcBumpAngleOpposDir, &driveBumperFree, &selFreeBumper2);
	mselFreeBumper.addChildren(&mseqFreeBumper1, &mseqFreeBumper3, &conditionBumperNotFound);
	mseqBumperActive.addChildren(&conBumperActive, &mselFreeBumper);

	// ************************************
	// Find Perimeter Behaviour
	//*************************************

	CruiseRotCW.m_nodeName = (char*)"CruiseRotCW";
	dnWaitSignalOutside.m_nodeName = (char*)"dnWaitSignalOutside";
	checkChargStationDisabled.m_nodeName = (char*)"checkChargStationDisabled";
	invConPerOutside.m_nodeName = (char*)"invConPerOutside";
	selFindPerimeter2.m_nodeName = (char*)"selFindPerimeter2";
	mseqFindPerimeter3.m_nodeName = (char*)"mseqFindPerimeter3";
	monFindPerimeter1.m_nodeName = (char*)"monFindPerimeter1";
	mselFindPerimeter0.m_nodeName = (char*)"mselFindPerimeter0";

	dnWaitSignalOutside.m_nodeName = (char*)"dnWaitSignalOutside";
	dnWaitSignalOutside.setChild(&conIsOutsidePerimeter);
	dnWaitSignalOutside.setWaitMillis(2000);

	invConPerOutside.setChild(&conPerOutside);

	mseqFindPerimeter3.addChildren(&motorStop, &checkChargStationDisabled, &dnWaitSignalOutside, &CruiseRotCW);
	selFindPerimeter2.addChildren(&mseqBumperActive, &cruiseSpeedToMotor);
	monFindPerimeter1.addChildren(&invConPerOutside, &selFindPerimeter2);

	mselFindPerimeter0.addServices(&srvUpdateHistory);
	mselFindPerimeter0.addChildren(&monFindPerimeter1, &mseqFindPerimeter3);

	// ************************************
	// Perimeter Tracking
	//*************************************

	monTrackPerimeter.m_nodeName = (char*)"v";
	checkOutsideAgain.m_nodeName = (char*)"checkOutsideAgain";
	driveCurve.m_nodeName = (char*)"driveCurve";
	setArc45CC.m_nodeName = (char*)"setArc45CC";
	monPTBumperActive3.m_nodeName = (char*)"monPTBumperActive3";
	mseqPTBumperActive6.m_nodeName = (char*)"mseqPTBumperActive6";
	invConBumperActive.m_nodeName = (char*)"invConBumperActive";
	mseqPTBumperActive5.m_nodeName = (char*)"mseqPTBumperActive5";
	mseqPTBumperActive4.m_nodeName = (char*)"mseqPTBumperActive4";
	mselPTBumperActive2.m_nodeName = (char*)"mselPTBumperActive2";
	mseqPTBumperActive1.m_nodeName = (char*)"mseqPTBumperActive1";
	findTriangle.m_nodeName = (char*)"findTriangle";
	lineFollow.m_nodeName = (char*)"lineFollow";
	LFRotateCC105.m_nodeName = (char*)"LFRotateCC105";
	parLineFollow.m_nodeName = (char*)"parLineFollow";
	mseqTrackPerimeter.m_nodeName = (char*)"mseqTrackPerimeter";
	perTrackChargingStationReached.m_nodeName = (char*)"perTrackChargingStationReached";
	mseqDockingStation.m_nodeName = (char*)"mseqDockingStation";
	selPerimeterTracking.m_nodeName = (char*)"selPerimeterTracking";


	mseqPTBumperActive6.addChildren(&setArc45CC, &driveHist0, &driveCurve, &motorStop, &checkOutsideAgain);
	monPTBumperActive3.addChildren(&invConBumperActive, &mseqPTBumperActive6);
	invConBumperActive.setChild(&conBumperActive);

	mseqPTBumperActive5.addChildren(&conIsDirectionForward, &setflagBumperActivatedLR, &MotorStopFast, &restoreLastRotation, &bumperDriveBack);
	mseqPTBumperActive4.addChildren(&conIsDirectionForward, &setflagBumperActivatedLR, &MotorStopFast, &bumperDriveBack);
	mselPTBumperActive2.addChildren(&mseqPTBumperActive4, &mseqPTBumperActive5);
	mseqPTBumperActive1.addChildren(&conBumperActive, &mselPTBumperActive2, &monPTBumperActive3);

	monTrackPerimeter.addChildren(&invConBumperActive, &mseqTrackPerimeter);
	parLineFollow.addChildren(&lineFollow, &findTriangle);
	mseqTrackPerimeter.addChildren(&parLineFollow, &motorStop, &LFRotateCC105);
	mseqDockingStation.addChildren(&conInDockingStation, &MotorStopFast, &dnWaitDockingStation, &perTrackChargingStationReached);
	selPerimeterTracking.addServices(&srvUpdateHistory);
	selPerimeterTracking.addChildren(&mseqDockingStation, &mseqPTBumperActive1, &monTrackPerimeter);



	// ************************************
	// Goto Area Behaviour
	//*************************************
	
	mseqGotoAreaX.m_nodeName = (char*)"mseqGotoAreaX";
	selGotoAreaX1.m_nodeName = (char*)"selGotoAreaX1";
	monGotoAreaX.m_nodeName = (char*)"monGotoAreaX";
	selGotoAreaX.m_nodeName = (char*)"selGotoAreaX";

	mseqGotoAreaX.addChildren(&conAreaReached,&motorStop,&rotate90CC,&driveHist0,&setMowBehaviour);
	selGotoAreaX1.addChildren(&mseqGotoAreaX, &dnWaitGotoArea);
	monGotoAreaX.addChildren(&invConBumperActive,&selGotoAreaX1);
	selGotoAreaX.addServices(&srvUpdateHistory);
	selGotoAreaX.addChildren(&cruiseStartMowMotor, &mseqPTBumperActive1, &monGotoAreaX);

	// *******************s*****************
	// Rotate
	//*************************************
	driveHist0.m_nodeName = (char*)"driveHist0";
	driveBumperFree.m_nodeName = (char*)"driveBumperFree";


	// ************************************
	// Perimeter Behaviour
	//*************************************
	setflagCoilFirstOutside.m_nodeName = (char*)"setflagCoilFirstOutside";
	invertRotationDirection.m_nodeName = (char*)"invertRotationDirection";
	restoreLastRotation.m_nodeName = (char*)"restoreLastRotation";
	motorStop.m_nodeName = (char*)"motorStop";
	mseqRotatePer4.m_nodeName = (char*)"mseqRotatePer4";
	mseqRotatePer3.m_nodeName = (char*)"mseqRotatePer3";
	mseqRotatePer2.m_nodeName = (char*)"mseqRotatePer2";
	mseqRotatePer1.m_nodeName = (char*)"mseqRotatePer1";
	selRotatePer.m_nodeName = (char*)"selRotatePer";
	calcAngle.m_nodeName = (char*)"calcAngle";
	perDriveBack.m_nodeName = (char*)"perDriveBack";
	overRun.m_nodeName = (char*)"overRun";
	runTempService.m_nodeName = (char*)"runTempService";
	mseqPerimeterForward.m_nodeName = (char*)"mseqPerimeterForward";
	mseqPerimeterActive.m_nodeName = (char*)"mseqPerimeterActive";
	mselPerimeterActive.m_nodeName = (char*)"mselPerimeterActive";


	mseqRotatePer4.addChildren(&conIsDirectionRotateCCorCW, &setflagCoilFirstOutside, &motorStop, &restoreLastRotation, &invertRotationDirection, &driveHist0);
	mseqRotatePer3.addChildren(&conLCO_and_DDCC, &setflagCoilFirstOutside, &motorStop, &restoreLastRotation, &invertRotationDirection, &driveHist0);
	mseqRotatePer2.addChildren(&conRCO_and_DDCW, &setflagCoilFirstOutside, &motorStop, &restoreLastRotation, &invertRotationDirection, &driveHist0);
	mseqRotatePer1.addChildren(&conBothCoilsOutside, &motorStop, &errorBothCoilsOutside);
	selRotatePer.addChildren(&mseqRotatePer2, &mseqRotatePer3, &driveHist0);
	mseqPerimeterForward.addChildren(&conIsDirectionForward, &setflagCoilFirstOutside, &overRun, &runTempService, &perDriveBack, &errorBothCoilsOutside, &calcAngle, &selRotatePer);
	mselPerimeterActive.addChildren(&mseqPerimeterForward, &mseqRotatePer4, &conditionPerimeterNotFound);
	mseqPerimeterActive.addChildren(&conPerOutside, &mselPerimeterActive);


	// ************************************
	// Mowing
	//*************************************
	selMowing.m_nodeName = (char*)"selMowing";
	selMowing.addServices(&srvUpdateHistory);
	selMowing.addChildren(&cruiseStartMowMotor, &checkOscillating, &mseqBumperActive, &mseqPerimeterActive, &acCruiseRaining, &acCruiseBatLow, &Check2AllCoilsOutside, &cruiseSpeedToMotor);

	// ************************************
	// Charging Behaviour
	//*************************************
	selCharging.m_nodeName = (char*)"selCharging";
	chargeRelayOn.m_nodeName = (char*)"chargeRelayOn";
	selCharging.addChildren(&chargeRelayOn);

	// ************************************
	// Leave Head Charging Station Behaviour
	//*************************************
	selLeaveHeadCS.m_nodeName = (char*)"selLeaveHeadCS";
	mseqLeaveHeadCS.m_nodeName = (char*)"mseqLeaveHeadCS";
	driveBackXCS.m_nodeName = (char*)"driveBackXCS";
	driveForwardXCS.m_nodeName = (char*)"driveForwardXCS";
	setArcHeadStation_ROT.m_nodeName = (char*)"setArcHeadStation_ROT";
	rotate90CC.m_nodeName = (char*)"rotate90CC";
	setMowBehaviour.m_nodeName = (char*)"setMowBehaviour";

	mseqLeaveHeadCS.addChildren(&driveBackXCS, &driveHist0, &rotate90CC, &driveHist0, &driveForwardXCS, &driveHist0, &setArcHeadStation_ROT, &driveHist0, &setMowBehaviour);
	selLeaveHeadCS.addServices(&srvUpdateHistory);
	selLeaveHeadCS.addChildren(&mseqLeaveHeadCS);

	// ************************************
	// Root
	//*************************************

	selRoot.m_nodeName = (char*)"selRoot";
	selRoot.addChildren(&eotCharging, &eotLeaveHeadChargingStation, &selCheck2, &eotGotoAreaX, &eotPermeterTracking, &eotFindPerimeter, &eotMowing);

	//selRoot.addChildren(&eotCharging, &eotLeaveHeadChargingStation, &selCheck2, &eotGotoAreaX, &eotPermeterTracking, &eotFindPerimeter, &eotMowing);

	behaviorTree.setRootNode(&selRoot);

	// Run the setup of all nodes which have a setup function.
	behaviorTree.onSetup(bb);
}

void TBehaviour::loop() {
	behaviorTree.tick(bb);

}


void TBehaviour::print() {
	behaviorTree.print();

}


