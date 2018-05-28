// adcman.h

/*
Note: requires Arduino Due (optimized for Due DMA)
Problem: you have multiple analog inputs, some need only to be sampled once, other need
a fixed sample rate.

Solution:
Arduino ADC manager (ADC0-ADC9)
- can capture multiple pins one after the other (example ADC0: 1000 samples, ADC1: 100 samples, ADC2: 1 sample etc.)
- can capture more than one sample into buffers (fixed sample rate)
- runs in background: interrupt-based (free-running)
- two types of ADC capture:
1) free-running ADC capturing (for certain sample count) (8 bit signed - zero = VCC/2)
2) ordinary ADC sampling (one-time sampling) (10 bit unsigned)
- WARNING: never use any 'analogRead()' in your code when using this class!

How to use it:
------example for one sampling-------
1. Initialize ADC:  ADCMan.begin();
2. Set ADC pin:     ADCMan.setupChannel(pinMotorMowSense, 1, true);
3. Program loop:    while (true){
ADCMan.run();
if (ADCMan.isConvComplete(pinMotorMowSense)){
int value = ADCMan.getValue(pinMotorMowSense);
}
}
------example for multiple samplings-------
1. Initialize ADC:  ADCMan.begin();
2. Set ADC pin:     ADCMan.setupChannel(pinPerimeterLeft, 255, true);
3. Program loop:    while (true){
ADCMan.run();
if (ADCMan.isConvComplete(pinPerimeterLeft)){
int16_t sampleCount = ADCMan.getSampleCount(pinPerimeterLeft);
int8_t *samples = ADCMan.getSamples(pinPerimeterLeft);
}
}

*/


#ifndef ADCMAN_H
#define ADCMAN_H


#include <Arduino.h>

#define ADC_BITS  12    // 12 bit ADC  
#define ADC_REF  3.3f  // 3.3 Volt reference

// sample rates
enum {
	SRATE_9615,
	SRATE_19231,
	SRATE_38462
};

#define ADC_CHANNEL_COUNT_MAX 12

// one channel data
struct ADCStruct {
	int sampleCount;
	byte pin;
	int32_t *samples;
	bool convComplete;

};


class ADCManager
{
public:
	bool showValuesOnConsole;
	ADCManager();
	int sampleRate;
	void begin();
	void run();
	void setupChannel(byte pin, int samplecount);
	int32_t* getSamples(byte pin);
	int getSampleCount(byte pin);
	int32_t getValue(byte pin);
	bool isConvComplete(byte pin);
	void restartConv(byte pin);
	void printInfo();
	int getConvCounter();
	//virtual void calibrate();
	int32_t measureOffset(byte pin);
private:
	int convCounter;
	byte chCurr;
	byte chNext;
	void setSampleCount(byte ch, int samplecount);
	void init(byte ch);
	ADCStruct channels[ADC_CHANNEL_COUNT_MAX];
	//boolean loadCalib();
	//void loadSaveCalib(boolean readflag);
	//void saveCalib();
};

extern ADCManager ADCMan;


#endif



