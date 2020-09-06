#ifndef BUZZER_H
#define BUZZER_H


#include <inttypes.h>
#include "Protothread.h"
#include "hardware.h"

enum SoundSelect { SND_START, SND_CHARGERELAYON, SND_CHARGERELAYOFF, SND_READY, SND_PROGRESS, SND_OVERCURRENT, SND_TILT, SND_PERIMETER_TIMEOUT };

class BuzzerClass : public Protothread {
public:
	void setup();
	void sound(SoundSelect idx);
	bool Run();
	void tone(uint16_t freq);
	void noTone();
protected:
	SoundSelect soundIdx;
	int toneIdx;

};



#endif
