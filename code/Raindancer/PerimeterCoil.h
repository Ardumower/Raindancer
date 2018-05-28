// Perimeter.h

#ifndef _PERIMETER_h
#define _PERIMETER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "DSP.h"
#include "config.h"

#ifdef CONF_USE128BIT_PER_SIGNAL
#define REFERENCE_SIGNAL_SIZE 128
#define SIDELOBE_CELLS 20
#endif // CONF_USE128BIT_PER_SIGNAL

#ifdef CONF_USE64BIT_PER_SIGNAL
#define REFERENCE_SIGNAL_SIZE 64
#define SIDELOBE_CELLS 15
#endif // CONF_USE64BIT_PER_SIGNAL

#ifdef CONF_USE32BIT_PER_SIGNAL
#define REFERENCE_SIGNAL_SIZE 32
#define SIDELOBE_CELLS 10
#endif // CONF_USE128BIT_PER_SIGNAL


class PerimeterCoil {
public:
  int8_t state = 0;

  int shift=0;

  bool showCorrelation = false;
  bool showCorrelationSQ = false;
  bool showADCWithoutOffset = false;
  bool showMatchedFilter = false;
  bool showValuesResults = false;
  bool showPSNRFunction = false;

  int32_t adcOffset = 0;

  DSP_TYPE magnetude = 0;
  DSP_TYPE peakValue = 0;
  int peakIdx = 0;
  DSP_TYPE peakValue2 = 0;
  int peakIdx2 = 0;
  int64_t corrSum = 0;
  float MSE = 0;
  float psnr=0;
  float psnr2=0;
  float ratio;
  
	PerimeterCoil();
	bool isSignalValid();
	void printSignal(char *text, DSP_TYPE *samples, int sz);
	void printSignalValues(DSP_TYPE *samples, int sz);
	void setup(DSP_TYPE *adc_sampling_r);
	void run(void);

  static int8_t  referenceSignal_r[REFERENCE_SIGNAL_SIZE];
	static DSP_TYPE  referenceSignalFFT_r[FFT_SIZE];
	static DSP_TYPE  referenceSignalFFT_i[FFT_SIZE];

	static DSP_TYPE correlation_r[FFT_SIZE];
	static DSP_TYPE correlation_i[FFT_SIZE];

  
	// ----- sampled signal --------
	DSP_TYPE  *sampling_r; // [FFT_SIZE];
	DSP_TYPE  sampling_i[FFT_SIZE];


private:
  static bool setup_firsttime;
  
};


#endif

