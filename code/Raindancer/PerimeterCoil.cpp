//
//
//

#include "PerimeterCoil.h"
#include "errorhandler.h"

#define GENERATE_DIFF_SIGNAL 1
#define OVERSAMPLING 4

static DSP dsp;

#ifdef CONF_USE128BIT_PER_SIGNAL
int8_t  PerimeterCoil::referenceSignal_r[REFERENCE_SIGNAL_SIZE] =
{ 1, 1, -1, -1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, 1, -1, 1, 1, -1, -1, 1, 1, -1, 1, 1, -1, 1, -1, 1, -1, -1, 1, -1, -1, 1, -1, 1, 1, -1, 1, -1, 1, -1, 1, -1, 1, 1, -1, -1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1, 1, -1, -1, 1, -1,
-1, 1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, 1, -1, 1, 1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1, 1, -1, -1, 1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1, 1, 1, -1, -1, 1
};
#endif // CONF_USE128BIT_PER_SIGNAL

#ifdef CONF_USE64BIT_PER_SIGNAL
int8_t  PerimeterCoil::referenceSignal_r[REFERENCE_SIGNAL_SIZE] =
//{ 1,-1,1,-1,1,1,-1,-1,1,-1,1,-1,1,-1,1,1,-1,-1,1,1,-1,-1,1,1,-1,1,-1,1,-1,-1,1,1,-1,1,-1,-1,1,-1,1,-1,1,1,-1,1,-1,1,-1,1,-1,1,-1,-1,1,-1,1,1,-1,1,-1,-1,1,1,-1,-1 }; /*Part of Bosch Indego Signal */
{ -1, 1, 1, -1, 1, -1, -1, 1, 1, -1, -1, 1, 1, -1, -1, 1, 1, -1, -1, 1, -1, 1, 1, -1, -1, 1, 1, -1, -1, 1, 1, -1, 1, -1, -1, 1, -1, 1, 1, -1, -1, 1, 1, -1, 1, -1, -1, 1, 1, -1, -1, 1, -1, 1, 1, -1, 1, -1, -1, 1, -1, 1, -1, 1 };
#endif // CONF_USE64BIT_PER_SIGNAL

#ifdef CONF_USE32BIT_PER_SIGNAL
int8_t  PerimeterCoil::referenceSignal_r[REFERENCE_SIGNAL_SIZE] =
{ 1,-1,-1,1,1,-1,-1,1,1,-1,1,-1,1,-1,-1,1,1,-1,1,-1,-1,1,-1,1,-1,1,1,-1,1,-1,1,-1 };
#endif // CONF_USE32BIT_PER_SIGNAL



DSP_TYPE  PerimeterCoil::referenceSignalFFT_r[FFT_SIZE] = { 0 };
DSP_TYPE  PerimeterCoil::referenceSignalFFT_i[FFT_SIZE] = { 0 };

DSP_TYPE PerimeterCoil::correlation_r[FFT_SIZE] = { 0 };
DSP_TYPE PerimeterCoil::correlation_i[FFT_SIZE] = { 0 };

bool PerimeterCoil::setup_firsttime = false;

PerimeterCoil::PerimeterCoil() {

}

#ifdef CONF_USE128BIT_PER_SIGNAL
bool PerimeterCoil::isSignalValid() {
	if ((ratio > 2.3f) && (psnr > 25) && (abs(magnetude) > 25)) {  // use if you don't square the correlation array
	//if (((psnr > 25 && ratio > 8.0f) || (psnr > 50)) && (abs(magnetude) > 25)) {  // use if you square the correlation array
		return true;
	}
	return false;
}
#endif


#ifdef CONF_USE64BIT_PER_SIGNAL
bool PerimeterCoil::isSignalValid() {
	//	if (((psnr > 10 && ratio > 2.3f) || (psnr > 25)) && (abs(magnetude) > 25)) {  // use if you don't square the correlation array
																					  //if (((psnr > 25 && ratio > 8.0f) || (psnr > 50)) && (abs(magnetude) > 25)) {  // use if you square the correlation array
	if ((ratio > 3f) && (psnr > 25)) && (abs(magnetude) > 25)) {  // use if you don't square the correlation array

	return true;
	}
	return false;
}
#endif

#ifdef CONF_USE32BIT_PER_SIGNAL
bool PerimeterCoil::isSignalValid() {
	if (((psnr > 10 && ratio > 2.0f) || (psnr > 25)) && (abs(magnetude) > 25)) {  // use if you don't square the correlation array
																				  //if (((psnr > 25 && ratio > 8.0f) || (psnr > 50)) && (abs(magnetude) > 25)) {  // use if you square the correlation array
		return true;
	}
	return false;
}
#endif

void PerimeterCoil::printSignal(char *text, DSP_TYPE *samples, int sz) {
	if (showMatchedFilter) {
		errorHandler.setInfoNoLog(F("%s\t%d:\r\n"), text, sz);

		int nonZeroCounter = 0;
		for (int i = 0; i < sz; i++) {
			errorHandler.setInfoNoLog(F("%d"), samples[i]);
			if (i < sz - 1)	errorHandler.setInfoNoLog(F("\t"));
			if (samples[i] != 0) nonZeroCounter++;
		}
		errorHandler.setInfoNoLog(F("\r\n"));
	}
}

void PerimeterCoil::printSignalValues(DSP_TYPE *samples, int sz) {
	for (int i = 0; i < 20; i++) {
		errorHandler.setInfoNoLog(F("%d\r\n"), -200);
	}
	for (int i = 0; i < sz; i++) {
		errorHandler.setInfoNoLog(F("%d\r\n"), samples[i]);
	}
}

void PerimeterCoil::setup(DSP_TYPE *adc_sampling_r) {

	sampling_r = adc_sampling_r; // save real sampling array address

	// Call setup only once in order referenceSignal_r will be changed.
	if (setup_firsttime) {
		return;
	}
	setup_firsttime = true;

	if (showMatchedFilter) {
		errorHandler.setInfoNoLog(F("REFERENCE_SIGNAL:"));
		for (int i = 0; i < REFERENCE_SIGNAL_SIZE; i++) {
			errorHandler.setInfoNoLog(F("%d,"), referenceSignal_r[i]);
		}
		errorHandler.setInfoNoLog(F("\r\n"));
	}

#ifdef GENERATE_DIFF_SIGNAL

	if (showMatchedFilter) {
		errorHandler.setInfoNoLog(F("GENERATE_DIFF_SIGNAL:"));
	}
	int8_t lastValue = referenceSignal_r[REFERENCE_SIGNAL_SIZE - 1];
	int8_t value;
	for (int i = 0; i < REFERENCE_SIGNAL_SIZE; i++) {
		value = referenceSignal_r[i];
		if (value == lastValue) referenceSignal_r[i] = 0;
		lastValue = value;
	}
	if (showMatchedFilter) {
		for (int i = 0; i < REFERENCE_SIGNAL_SIZE; i++) {
			errorHandler.setInfoNoLog(F("%d,"), referenceSignal_r[i]);
		}
		errorHandler.setInfoNoLog(F("\r\n"));
	}
#endif

	// ------ generate oversampled signal -----
	for (int i = 0; i < FFT_SIZE; i++) {
		DSP_TYPE value = referenceSignal_r[((int)(i / OVERSAMPLING))]; // oversampling 4;
		referenceSignalFFT_r[i] = value * 10;  // Scale in order numbers 1/-1 are too low for fft. There would be 0 values where normaly the values where <1.
		referenceSignalFFT_i[i] = 0;
	}

	printSignal((char*)"referenceSignalFFT_r Oversample", referenceSignalFFT_r, FFT_SIZE);
	printSignal((char*)"referenceSignalFFT_i Oversample", referenceSignalFFT_i, FFT_SIZE);

	dsp.fft(referenceSignalFFT_r, referenceSignalFFT_i, LOG2_FFT, false);

	printSignal((char*)"referenceSignalFFT_r after FFT", referenceSignalFFT_r, FFT_SIZE);
	printSignal((char*)"referenceSignalFFT_i after FFT", referenceSignalFFT_i, FFT_SIZE);

	// Calculate amplitude spectrum
	DSP_TYPE referenceSignalAmplitude[FFT_SIZE];
	DSP_TYPE maxAmplitude = 0;

	for (int i = 0; i < FFT_SIZE; i++) {
		referenceSignalAmplitude[i] = sqrt((referenceSignalFFT_r[i] * referenceSignalFFT_r[i]) + (referenceSignalFFT_i[i] * referenceSignalFFT_i[i]));
		if (referenceSignalAmplitude[i] > maxAmplitude) {
			maxAmplitude = referenceSignalAmplitude[i];
		}

		/*
		if (referenceSignalAmplitude[i] < 10) {
			referenceSignalFFT_r[i] = 0;
			referenceSignalFFT_i[i] = 0;
		}
		*/


	}
	printSignal((char*)"referenceSignal Amplitude Spectrum", referenceSignalAmplitude, FFT_SIZE);

	//printSignal((char*)"referenceSignalFFT_r after Amplitude Correction", referenceSignalFFT_r, FFT_SIZE);
	//printSignal((char*)"referenceSignalFFT_i after Amplitude Correction", referenceSignalFFT_i, FFT_SIZE);


}


void PerimeterCoil::run(void) {

	int32_t sum;
	int32_t countNumbers;
	int32_t delta = 0;
	int32_t idxSearchEnd, idxSearchStart;
	float percentage;


	switch (state) {

	case 0: state++;
		// ---- compute offset of real part  ---------
		sum = 0;
		// Mittelwert bzw. Offset berechnen
		for (int i = 0; i < FFT_SIZE; i++) {
			sum += sampling_r[i];
		}
		adcOffset = sum / FFT_SIZE; // Offset is the average
		break;

	case 1: state++;
		// Offset subtrahieren von empfsngssignal
		for (int i = 0; i < FFT_SIZE; i++) {
			sampling_r[i] -= adcOffset;
		}
		if (showADCWithoutOffset) {
			printSignalValues(sampling_r, FFT_SIZE);
		}
		break;

	case 2: state++;
		// ---- delete imaginary part  ---------
		for (int i = 0; i < FFT_SIZE; i++) {
			sampling_i[i] = 0;
		}
		printSignal((char*)"ADC-Offset_r", sampling_r, FFT_SIZE);
		printSignal((char*)"ADC-Offset_i", sampling_i, FFT_SIZE);
		break;

	case 3: state++;
		// ---- compute FFT of sampling  ---------
		dsp.fft1(sampling_r, sampling_i, LOG2_FFT, false);
		break;

	case 4: state++;
		// ---- compute FFT of sampling  ---------
		dsp.fft2(sampling_r, sampling_i, LOG2_FFT, false);
		break;

	case 5: state++;
		// ---- compute FFT of sampling  ---------
		dsp.fft3(sampling_r, sampling_i, LOG2_FFT, false);
		break;

	case 6: state++;
		// ---- compute FFT of sampling  ---------
		dsp.fft4(sampling_r, sampling_i, LOG2_FFT, false);
		printSignal((char*)"FFT_r ", sampling_r, FFT_SIZE);
		printSignal((char*)"FFT_i ", sampling_i, FFT_SIZE);
		break;

	case 7: state++;
		// ---- compute complex conjugate of sampling ---------
		for (int i = 0; i < FFT_SIZE; i++) {
			sampling_i[i] *= -1;
		}
		printSignal((char*)"conj", sampling_i, FFT_SIZE);
		break;

	case 8: state++;
		// ---- complex multiply of refence signal FFT and sampling FFT ---------
		dsp.mult_complex(referenceSignalFFT_r, referenceSignalFFT_i, sampling_r, sampling_i, correlation_r, correlation_i, FFT_SIZE);
		printSignal((char*)"mul_r", correlation_r, FFT_SIZE);
		printSignal((char*)"mul_i", correlation_i, FFT_SIZE);
		break;

	case 9: state++;
		// ---- compute inverse FFT (iFFT) of multiplication result = correlation ---------
		dsp.fft1(correlation_r, correlation_i, LOG2_FFT, true);
		break;

	case 10: state++;
		// ---- compute inverse FFT (iFFT) of multiplication result = correlation ---------
		dsp.fft2(correlation_r, correlation_i, LOG2_FFT, true);
		break;

	case 11: state++;
		// ---- compute inverse FFT (iFFT) of multiplication result = correlation ---------
		dsp.fft3(correlation_r, correlation_i, LOG2_FFT, true);
		break;

	case 12: state++;
		// ---- compute inverse FFT (iFFT) of multiplication result = correlation ---------
		dsp.fft4(correlation_r, correlation_i, LOG2_FFT, true);
		break;

	case 13: state++;
		// ---- find peak ---------
		peakValue = 0;
		peakIdx = 0;
		for (int i = 0; i < FFT_SIZE; i++) {
			if (abs(correlation_r[i]) > abs(peakValue)) {
				peakValue = correlation_r[i]; //maximal Signalintensität
				peakIdx = i;
			}
		}

		// find second right peak instead of real peak. For testing peakSLL code only!
		//errorHandler.setInfoNoLog(F("real peak: %d @%d\r\n"), peakValue, peakIdx);
		/*

		peakIdx2 = peakIdx;
		if (peakValue > 0) {
			peakValue = 0;
			// search for negative max
			for (int i = 0; i < FFT_SIZE; i++) {
				if (i != peakIdx2) { // skip the real peak.
					if ( correlation_r[i] < peakValue) {
						peakValue = correlation_r[i];
						peakIdx = i;
					}
				}
			}
		}
		else {
			peakValue = 0;
			// search for positive max
			for (int i = 0; i < FFT_SIZE; i++) {
				if (i != peakIdx2) { // skip the real peak.
					if (correlation_r[i] > peakValue) {
						peakValue = correlation_r[i]; 
						peakIdx = i;
					}
				}
			}
		}
		//errorHandler.setInfoNoLog(F("second peak: %d @%d\r\n"), peakValue, peakIdx);
		*/


		// Save magetude which is used by perimeter.cpp
		magnetude = peakValue;

		if (showCorrelation) {
			printSignalValues(correlation_r, FFT_SIZE);
		}
		printSignal((char*)"iFFT_r", correlation_r, FFT_SIZE);
		printSignal((char*)"iFFT_i", correlation_i, FFT_SIZE);

		break;

		// ================================================================================================================
		// Now we have the magnitude and its sign
		// The following code from now on is only needed to determin if the received signal is valid or has too much errors
		// ================================================================================================================

		/*
			case 14: state++; // This state will not be called anymore
				// ---- square correlation array ---------

				peakValue = peakValue * peakValue;
				for (int i = 0; i < FFT_SIZE; i++) {
				//	if (correlation_r[i] < 0)
				//		correlation_r[i] = -sq(correlation_r[i]);
				//	else
						correlation_r[i] = sq(correlation_r[i]);
				}
				// From here on the correlation signal is only positive!!!

				if (showCorrelationSQ) {
					printSignalValues(correlation_r, FFT_SIZE);
				}
				break;
				*/

	case 14: state++;
		// --- get correlation sum (without max peak and nearby coils) and peakValue2 value -------
		//quadratische Mittelwert QMW - mean squared error
		corrSum = 0;
		peakValue2 = 0;
		peakIdx2 = 0;
		countNumbers = 0;

		if (peakIdx > (FFT_SIZE - SIDELOBE_CELLS)) {
			for (int i = SIDELOBE_CELLS; i < (peakIdx - SIDELOBE_CELLS); i++) {
				corrSum += (int64_t)sq(correlation_r[i]);
				countNumbers++;
				if (abs(correlation_r[i]) > abs(peakValue2)) {
					peakValue2 = correlation_r[i]; //maximal Signalintensität
					peakIdx2 = i;
				}
			}
		}
		else  if (peakIdx < SIDELOBE_CELLS) {
			for (int i = peakIdx + SIDELOBE_CELLS; i < (FFT_SIZE - SIDELOBE_CELLS); i++) {
				corrSum += (int64_t)sq(correlation_r[i]);
				countNumbers++;
				if (abs(correlation_r[i]) > abs(peakValue2)) {
					peakValue2 = correlation_r[i]; //maximal Signalintensität
					peakIdx2 = i;
				}
			}
		}
		else {
			for (int i = 0; i < FFT_SIZE; i++) {
				if (i < (peakIdx - SIDELOBE_CELLS) || i >(peakIdx + SIDELOBE_CELLS)) {
					corrSum += (int64_t)sq(correlation_r[i]);
					countNumbers++;
					if (abs(correlation_r[i]) > abs(peakValue2)) {
						peakValue2 = correlation_r[i]; //maximal Signalintensität
						peakIdx2 = i;
					}
				}
			}
		}


		MSE = (float)corrSum / ((float)countNumbers);


		if (showPSNRFunction) {
			if (MSE > 0.000001f) {
				for (int i = 0; i < FFT_SIZE; i++) {
					psnr = (sq(((float)(correlation_r[i]))) / MSE);
					errorHandler.setInfoNoLog(F("%f\r\n"), psnr);
				}
			}
			else {
				for (int i = 0; i < FFT_SIZE; i++) {
					errorHandler.setInfoNoLog(F("%f\r\n"), 0);
				}
			}

		}
		break;

	case 15: state++;

		// ---- find peakSLL (peak Side Lobe Left)
		// Search for left sidelobe of other sign. Because if receiver overdrives then it could be,
		// that the second sidelobe is higher than the normal matched filter peak which is the first.

		peakValueSLL = 0;
		peakIdxSLL = 0;
		delta = peakIdx - 8;
		idxSearchStart = peakIdx-2;
		idxSearchEnd = idxSearchStart - 8;
		if (idxSearchEnd < 0) {
			idxSearchEnd = -1;
		}


		if (peakValue > 0) { // Peak1 positive -> Search SLL for negative values
			for (int i = idxSearchStart; i > idxSearchEnd; i--) {
				if (correlation_r[i] < peakValueSLL) {
					peakValueSLL = correlation_r[i];
					peakIdxSLL = i;
				}
			}

			// If delta < 0 search at the end of the array
			if (delta < 0) {
				idxSearchStart = FFT_SIZE - 1;
				idxSearchEnd = idxSearchStart - abs(delta);
				for (int i = idxSearchStart; i > idxSearchEnd; i--) {
					if (correlation_r[i] < peakValueSLL) {
						peakValueSLL = correlation_r[i];
						peakIdxSLL = i;
					}
				}
			}
		}
		else { // Peak1 negative -> Search SLL for positive values

			for (int i = idxSearchStart; i > idxSearchEnd; i--) {
				if (correlation_r[i] > peakValueSLL) {
					peakValueSLL = correlation_r[i];
					peakIdxSLL = i;
				}
			}

			// If delta < 0 search at the end of the array
			if (delta < 0) {
				idxSearchStart = FFT_SIZE - 1;
				idxSearchEnd = idxSearchStart - abs(delta);
				for (int i = idxSearchStart; i > idxSearchEnd; i--) {
					if (correlation_r[i] > peakValueSLL) {
						peakValueSLL = correlation_r[i];
						peakIdxSLL = i;
					}
				}
			}
		}


		//https://de.wikipedia.org/wiki/Signal-Rausch-Verh%C3%A4ltnis

		if (MSE > 0.000001f) {
			psnr = (sq(((float)(peakValue))) / MSE);
			psnr2 = (sq(((float)(peakValue2))) / MSE);
			psnrSLL = (sq(((float)(peakValueSLL))) / MSE);
		}
		else {
			psnr = 1;
			psnr2 = 1;
			psnrSLL = 1;

		}

		if (psnr2 < 0.000001f) {
			psnr2 = 0.000001f;
		}
		ratio = psnr / psnr2;

		//float snr = dsp.snr(signal_r, peakIdx, sampling_r, 1.0f/4095.0f, FFT_SIZE);

		// Check if sidelobe left is near found signal
		if (isSignalValid()) {
			overdriveDetected = false;
			percentage = (psnrSLL * 100.0f) / psnr;
			if (percentage > 95.0f) {
				//peakValue = -1 * peakValue;
				magnetude = -1 * magnetude;
				overdriveDetected = true;
			}
		}

		if (showValuesResults) {
			errorHandler.setInfoNoLog(F("mag: %4d  peak @ %3d : %4d   peak2 @ %3d : %4d   peakSLL @ %3d : %4d   MSE: %8.3f   psnr: %8.3f   psnr2: %8.3f  psnrSLL: %8.3f  ratio: %8.3f"),
				magnetude, peakIdx, peakValue, peakIdx2, peakValue2, peakIdxSLL, peakValueSLL, MSE, psnr, psnr2, psnrSLL, ratio);

			//			errorHandler.setInfoNoLog(F("  peak @ %3d : %4d^2= %4d  peak2 @ %3d : %4d   MSE: %8.3f   psnr: %8.3f   psnr2: %8.3f   ratio: %8.3f"),
			//				peakIdx, magnetude, peakValue, peakIdx2, peakValue2, MSE, psnr, psnr2, ratio);


			if (isSignalValid()) {
				if (overdriveDetected) {
					errorHandler.setInfoNoLog(F("    ODR"));
				}
			}
			else {
				errorHandler.setInfoNoLog(F("    BAD"));
			}

			errorHandler.setInfoNoLog(F("\r\n"));
		}

		break;

	default:  state = 0;
		break;
	}






}
