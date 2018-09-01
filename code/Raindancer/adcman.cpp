// 
// 
// 

/* NOTE: requires Arduino Due
Continouesly performs ADC conversion for all configured ADC channels one after the other using Arduino Due DMA transfer.
*/

#include <chip.h>
#include <Arduino.h>
#include <limits.h>
#include "adcman.h"
#include "errorhandler.h"


#define ADC_SAMPLE_COUNT_MAX 512
#define INVALID_CHANNEL 99

#define NO_CHANNEL 255

volatile int16_t dmaData[ADC_SAMPLE_COUNT_MAX];



ADCManager::ADCManager() {
	showValuesOnConsole = false;
	convCounter = 0;
	chNext = 0;
	chCurr = INVALID_CHANNEL;
	for (int i = 0; i < ADC_CHANNEL_COUNT_MAX; i++) {
		channels[i].sampleCount = 0;
		//channels[i].samples[0] = 0; Existiert hier noch gar nicht
		channels[i].convComplete = false;
	}
	sampleRate = SRATE_38462;   // sampling frequency 38462 Hz
}


void ADCManager::begin() {
	/*pinMode(A0, INPUT);
	while(true){
	DEBUGLN(analogRead(A0));
	}
	*/
	// free running ADC mode, f = ( adclock / 21 cycles per conversion )
	// example f = 19231  Hz:  using ADCCLK=405797 will result in a adcclock=403846 (due to adc_init internal conversion)
	uint32_t adcclk;
	switch (sampleRate) {
	case SRATE_38462: adcclk = 811595; break;
	case SRATE_19231: adcclk = 405797; break;
	case SRATE_9615: adcclk = 202898; break;
	default: adcclk = 811595; break;
	}
	pmc_enable_periph_clk(ID_ADC); // To use peripheral, we must enable clock distributon to it
	adc_init(ADC, SystemCoreClock, adcclk, ADC_STARTUP_FAST); // startup=768 clocks
	adc_disable_interrupt(ADC, 0xFFFFFFFF);
	adc_set_resolution(ADC, ADC_12_BITS);
	adc_configure_power_save(ADC, ADC_MR_SLEEP_NORMAL, ADC_MR_FWUP_OFF); // Disable sleep
	adc_configure_timing(ADC, 0, ADC_SETTLING_TIME_3, 1);  // tracking=0, settling=17, transfer=1      
	adc_set_bias_current(ADC, 1); // Bias current - maximum performance over current consumption
	adc_disable_tag(ADC);  // it has to do with sequencer, not using it 
	adc_disable_ts(ADC);   // disable temperature sensor 
	adc_stop_sequencer(ADC);  // not using it
	adc_disable_all_channel(ADC);
	adc_configure_trigger(ADC, ADC_TRIG_SW, 1); // triggering from software, freerunning mode      
	adc_start(ADC);

	/* // test conversion
	setupChannel(A0, 1, false);
	setupChannel(A1, 3, false);
	while(true){
	DEBUG("test A0=");
	DEBUG(getVoltage(A0));
	DEBUG("  A1=");
	DEBUGLN(getVoltage(A1));
	DEBUG("  cnvs=");
	DEBUGLN(getConvCounter());
	run();
	delay(500);
	}*/
	//	loadCalib();
}

void ADCManager::printInfo() {
	errorHandler.setInfoNoLog(F("---ADC---\r\nconversions = %d sampleRate="), convCounter);
	switch (sampleRate) {
	case SRATE_38462: errorHandler.setInfoNoLog(F("38462\r\n")); break;
	case SRATE_19231: errorHandler.setInfoNoLog(F("19231\r\n")); break;
	case SRATE_9615: errorHandler.setInfoNoLog(F("9615\r\n")); break;
	}
	for (int ch = 0; ch < ADC_CHANNEL_COUNT_MAX; ch++) {
		if (channels[ch].sampleCount != 0) {
				errorHandler.setInfoNoLog(F("AD%d\tsampleCount=%d\r\n"), ch, channels[ch].sampleCount);
				errorHandler.setInfoNoLog(F("\r\n"));
		}
	}
}

int ADCManager::getConvCounter() {
	int res = convCounter;
	convCounter = 0;
	return res;
}

void ADCManager::setupChannel(byte pin, int samplecount) {
	byte ch = pin - A0;
    errorHandler.setInfoNoLog(F("adcman setup pin: AD%d\r\n"), ch);
	pinMode(pin, INPUT);
	channels[ch].pin = pin;
	channels[ch].convComplete = false;
	setSampleCount(ch, samplecount);
}

void ADCManager::setSampleCount(byte ch, int samplecount) {
	samplecount = min(samplecount, ADC_SAMPLE_COUNT_MAX);
	channels[ch].samples = (int32_t *)realloc(channels[ch].samples, samplecount * sizeof(int32_t));
	//channels[ch].samples = (int32_t *)malloc(samplecount * sizeof(int32_t));

	if (channels[ch].samples == NULL) {
		errorHandler.setError(F("ADC malloc could not allocate memory\r\n"));
	}

	channels[ch].sampleCount = samplecount;
}

bool ADCManager::isConvComplete(byte pin) {
	byte ch = pin - A0;
	return channels[ch].convComplete;
}


void ADCManager::restartConv(byte pin) {
	byte ch = pin - A0;
	channels[ch].convComplete = false;
}

int32_t* ADCManager::getSamples(byte pin) {
	byte ch = pin - A0;
	return channels[ch].samples;
}

int ADCManager::getSampleCount(byte pin) {
	byte ch = pin - A0;
	return channels[ch].sampleCount;
}

int32_t ADCManager::getValue(byte pin) {
	byte ch = pin - A0;
	channels[ch].convComplete = false;
	return channels[ch].samples[0];
}


void ADCManager::init(byte ch) {
	//adc_disable_channel_differential_input(ADC, (adc_channel_num_t)g_APinDescription[ channels[ch].pin ].ulADCChannelNumber );
	// configure Peripheral DMA  
	adc_enable_channel(ADC, (adc_channel_num_t)g_APinDescription[channels[ch].pin].ulADCChannelNumber);
	delayMicroseconds(100);
	PDC_ADC->PERIPH_RPR = (uint32_t)dmaData; // address of buffer
	PDC_ADC->PERIPH_RCR = channels[ch].sampleCount;
	PDC_ADC->PERIPH_PTCR = PERIPH_PTCR_RXTEN; // enable receive      
}

// start another conversion
void ADCManager::run() {
	if ((adc_get_status(ADC) & ADC_ISR_ENDRX) == 0) return; // conversion busy
			
	// post-process sampling data
	if (chCurr != INVALID_CHANNEL) {
		adc_disable_channel(ADC, (adc_channel_num_t)g_APinDescription[channels[chCurr].pin].ulADCChannelNumber);

		// --------transfer DMA samples----------
		for (int i = 0; i < channels[chCurr].sampleCount; i++) {
			//if (channels[chCurr].sampleCount > 1) {
			//	errorHandler.setInfoNoLog(F("%d, "), dmaData[i]);
			//}
			channels[chCurr].samples[i] = (int32_t)dmaData[i];
		}

		if (showValuesOnConsole) {
			errorHandler.setInfoNoLog(F("AD%d: "), chCurr);
			for (int i = 0; i < channels[chCurr].sampleCount; i++) {
				errorHandler.setInfoNoLog(F("%d, "), channels[chCurr].samples[i]);
			}
			errorHandler.setInfoNoLog(F("\r\n"));
		}


		channels[chCurr].convComplete = true;
		chCurr = INVALID_CHANNEL;
		convCounter++;
	}
	// start next channel sampling    
	for (int i = 0; i < ADC_CHANNEL_COUNT_MAX; i++) {
		chNext++;
		if (chNext == ADC_CHANNEL_COUNT_MAX) chNext = 0;
		if (channels[chNext].sampleCount != 0) {
			if (!channels[chNext].convComplete) {
				chCurr = chNext;
				init(chCurr);
				break;
			}
		}
	}
}



int32_t ADCManager::measureOffset(byte pin) {
	byte ch = pin - A0;
	while (!channels[ch].convComplete) {
		run();
	}
	return getValue(pin);

}


/*

void ADCManager::calibrate() {
	DEBUG(F("ADC calibration..."));
	for (int ch = 0; ch < ADC_CHANNEL_COUNT_MAX; ch++) {
		if (channels[ch].autoCalibrate) {
			channels[ch].zeroOfs = 0;
		}
	}
	for (int ch = 0; ch < ADC_CHANNEL_COUNT_MAX; ch++) {
		if (channels[ch].autoCalibrate) {
			while (!channels[ch].convComplete) {
				run();
			}
			channels[ch].zeroOfs = channels[ch].value;
		}
	}
	saveCalib();
	calibrationAvail = true;
}
*/



