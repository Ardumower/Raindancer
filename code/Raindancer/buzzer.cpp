#include "buzzer.h"
#include "config.h"
#include <Arduino.h>
#ifndef __AVR__
#include "DueTimer.h"
#endif





#ifndef __AVR__
static boolean tone_pin_state = false;

void toneHandler() {
	doBuzzer.write(tone_pin_state = !tone_pin_state);
}
#endif 



void BuzzerClass::sound(SoundSelect idx) {
	soundIdx = idx;
	toneIdx = 0;
	interval = 0;
	Restart(); //enable thread!!!
	Run();
}

bool BuzzerClass::Run() {

	if (!IsRunning()) return true;

	if(millis() - last_run < interval) return true;
	last_run = millis();

	switch (soundIdx) {
	case SND_START:
		switch (toneIdx) {
		case 0: tone(2000);  interval = 100; break;
		case 1: tone(3000);  interval = 100; break;
		case 2: tone(2000);  interval = 100; break;
		case 3: tone(3000);  interval = 100; break;
		case 4: tone(2000);  interval = 100; break;
		case 5: tone(3000);  interval = 100; break;
		case 6: noTone(); Stop();   break; //disable thread because we are done!!!
		}
		break;
	case SND_READY:
		switch (toneIdx) {
		case 0: tone(4200);  interval = 250; break;
		case 1: noTone();    interval = 250; break;
		case 2: tone(4200);  interval = 250; break;
		case 3: noTone();    interval = 250; break;
		case 4: tone(4200);  interval = 250; break;
		case 5: noTone();    interval = 250; break;
		case 6: tone(4200);  interval = 250; break;
		case 7: noTone();  Stop();   break; //disable thread because we are done!!!
		}
		break;
	case SND_CHARGERELAYON:
		switch (toneIdx) {
		case 0: tone(440);   interval = 200; break;
		case 1: noTone();    interval = 200; break;
		case 2: tone(600);   interval = 200; break;
		case 3: noTone();    interval = 200; break;
		case 4: tone(880);   interval = 200; break;
		case 5: noTone();    interval = 200; break;
		case 6: tone(1320);  interval = 200; break;
		case 7: noTone();  Stop();   break; //disable thread because we are done!!!
		}
		break;
	case SND_CHARGERELAYOFF:
		switch (toneIdx) {
		case 0: tone(1320);  interval = 200; break;
		case 1: noTone();    interval = 200; break;
		case 2: tone(880);   interval = 200; break;
		case 3: noTone();    interval = 200; break;
		case 4: tone(600);   interval = 200; break;
		case 5: noTone();    interval = 200; break;
		case 6: tone(440);   interval = 200; break;
		case 7: noTone();  Stop();   break; //disable thread because we are done!!!
		}
		break;
	case SND_PROGRESS:
		break;
	case SND_OVERCURRENT:
		break;
	case SND_TILT:
		break;
	case SND_PERIMETER_TIMEOUT:
		break;
	}
	toneIdx++;

	return true;
}

void BuzzerClass::setup()
{
	doBuzzer = LOW;
	toneIdx = 0;
	Stop();;
	interval = 0;
}


void BuzzerClass::tone(uint16_t  freq)
{
	Timer1.attachInterrupt(toneHandler).setFrequency(freq).start();
}


void BuzzerClass::noTone() {
	Timer1.stop();
	doBuzzer = LOW;
}





