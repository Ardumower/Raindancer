#ifndef BUZZER_H
#define BUZZER_H


#include <inttypes.h>
#include "Thread.h"
#include "hardware.h"

enum SoundSelect { SND_START, SND_CHARGERELAYON, SND_CHARGERELAYOFF, SND_READY, SND_PROGRESS, SND_OVERCURRENT, SND_TILT, SND_PERIMETER_TIMEOUT };

class BuzzerClass : public Thread {
public:
	void setup();
	void sound(SoundSelect idx);
	void run();
	void tone(uint16_t freq);
	void noTone();
protected:
	SoundSelect soundIdx;
	int toneIdx;

};



#endif
