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


#ifndef PERIMETER_H
#define PERIMETER_H
// EmpfÃ¤ngt die Perimeterdaten Ã¼ber die SerDueMot read Schnittstelle vom due.

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

//#include "BufferedSerial.h"
#include "Thread.h"
#include "helpers.h"
#include "PerimeterCoil.h"
#include "RunningMedian.h"


enum EPerSignal {
    SIGNAL_INSIDE = -1, // optional, -1 is the initial state of the fsm
    SIGNAL_NA = 0,
    SIGNAL_OUTSIDE = 1
};

enum EPerReceiveState {
    SPR_OFF = -1, // optional, -1 is the initial state of the fsm
	SPR_WAIT_COILL_SAMPLE_COMPLETED,
	SPR_WAIT_COILR_SAMPLE_COMPLETED,
	SPR_COILL_CALCULATE,
	SPR_COILR_CALCULATE

};



class TPerimeterThread: public Thread, public FSM<EPerReceiveState>
{
private:


    float arcToPerimeter;
    

    virtual void UpdateState( EPerReceiveState t );

    // Werte hier negative fÃ¼r inside
    int32_t _magnetudeL;
	int32_t _magnetudeR;
	int32_t curMaxL, curMaxR;
	FastRunningMedian<int32_t, 16,0> medianMagL;
	FastRunningMedian<int32_t, 16,0> medianMagR;

    // Werte hier positiv fÃ¼r inside
    void CaluculateInsideOutsideL(int32_t magl);
	void CaluculateInsideOutsideR(int32_t magr);

public:

	PerimeterCoil coilL;
	PerimeterCoil coilR;
	//-----------------------------------------------------
    // Empfangsdaten von Perimetersensoren Links und Rechts
    //-----------------------------------------------------

    //  Achtung, vor dem Verwenden  der Daten erstmal prÃ¼fen ob neueDatenEmpfangen == true ist. Wenn Daten verarbeitet wurden, neueDatenEmpfangen=false setzen.
    // Funktioniert nicht, wenn mehrer Prozesse gleichzeitig auf die Daten zugreifen
    //  checksumError gibt an, ob die empfangenen Daten richtig sind oder fehlerhaft. Die Daten werden in die Variablen eingetragen, neueDatenEmpfangen wird aber auf false gesetzt.
    //  seriellesSignalHeaderError gibt an, dass Daten auf der Seriellen Leitung emfangen wurden, aber der Header nicht gefunden wurde. Variablen Daten werden nicht verÃ¤ndert.


    // Achtung, Werte sind positive fÃ¼r inside!!!
    int magnetudeL;  // nimmt nur beim start 0 an. danach wird immer der letzte wert gelatched, wenn signal verloren
    int magnetudeR;
	int magnetudeL0; // nimmt zusätzlich 0 an wenn signal ungültig. Wird für Linefollowing verwendet verwendet.
	int magnetudeR0;

	int16_t signalCounterL;    // 2 outside >=  wert  >=-2 inside
	int16_t signalCounterR;
	int16_t signalCounterLFast;    // 2 outside >=  wert  >=-2 inside
	int16_t signalCounterRFast;
    
	unsigned long lastTimeSignalReceivedL;
    unsigned long lastTimeSignalReceivedR;
    
    int count;

	int magMax;
    
    // Achtung, Linefollowing arbeitet mit positiven werten!!!
    //long magLineFollow;
    //long magLineFollowMin;
    //long magLineFollowMax;
       
    bool showValuesOnConsole;

    bool isNearPerimeter();

    bool isLeftInside();
    bool isRightInside();
	//bool isRightInsideMag();
    bool isLeftOutside();
    bool isRightOutside();
	//bool isRightOutsideMag();

    void setup();
    virtual void run(void);
	void showConfig();
};

#endif

