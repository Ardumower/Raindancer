// RunningMedian.h

// many adds seldom getMedian() => RunningMedian O(n^2) //If you add a lot of values and retrieve the median only seldom there is maybe a speed advantage with RunningMedian
// few adds() and many getMedian() => FastRunningMedian O(n)
// adds() and then getMedian() => FastRunningMedian  (O(n)


// running median filter
//
// usage:
//   RunningMedian<unsigned int,32> myMedian;
//   if (myMedian.getStatus() == myMedian.OK)  myMedian.getMedian(_median);

#ifndef _RUNNINGMEDIAN_h
#define _RUNNINGMEDIAN_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

//
//    FILE: RunningMedian.h
//  AUTHOR: Rob dot Tillaart at gmail dot com
// PURPOSE: RunningMedian library for Arduino
// VERSION: 0.2.00 - template edition
//     URL: http://arduino.cc/playground/Main/RunningMedian
// HISTORY: 0.2.00 first template version by Ronny
//          0.2.01 added getAverage(uint8_t nMedians, float val)
//
// Released to the public domain
//

#include <inttypes.h>

template <typename T, int N> class RunningMedian {

public:

	enum STATUS { OK = 0, NOK = 1 };

	RunningMedian() {
		_size = N;
		clear();
	};

	void clear() {
		_cnt = 0;
		_idx = 0;
		_sorted = false;
	};

	void add(T value) {
		_ar[_idx++] = value;
		if (_idx >= _size) _idx = 0; // wrap around
		if (_cnt < _size) _cnt++;
		_sorted = false;
	};

	STATUS getMedian(T& value) {
		if (_cnt > 0) {
			if (_sorted == false) sort();
			value = _as[_cnt / 2];
			return OK;
		}
		return NOK;
	};

	STATUS getAverage(float &value) {
		if (_cnt > 0) {
			float sum = 0;
			for (uint8_t i = 0; i < _cnt; i++) sum += _ar[i];
			value = sum / _cnt;
			return OK;
		}
		return NOK;
	};

	STATUS getAverage(uint8_t nMedians, float &value) {
		if ((_cnt > 0) && (nMedians > 0))
		{
			if (_cnt < nMedians) nMedians = _cnt;     // when filling the array for first time
			uint8_t start = ((_cnt - nMedians) / 2);
			uint8_t stop = start + nMedians;
			if (_sorted == false) sort();
			float sum = 0;
			for (uint8_t i = start; i < stop; i++) sum += _as[i];
			value = sum / nMedians;
			return OK;
		}
		return NOK;
	}

	STATUS getHighest(T& value) {
		if (_cnt > 0) {
			if (_sorted == false) sort();
			value = _as[_cnt - 1];
			return OK;
		}
		return NOK;
	};

	STATUS getLowest(T& value) {
		if (_cnt > 0) {
			if (_sorted == false) sort();
			value = _as[0];
			return OK;
		}
		return NOK;
	};

	unsigned getSize() {
		return _size;
	};

	unsigned getCount() {
		return _cnt;
	}

	STATUS getStatus() {
		return (_cnt > 0 ? OK : NOK);
	};

private:
	uint8_t _size;
	uint8_t _cnt;
	uint8_t _idx;
	bool _sorted;
	T _ar[N];
	T _as[N];
	void sort() {
		// copy
		for (uint8_t i = 0; i < _cnt; i++) _as[i] = _ar[i];

		// sort all
		for (uint8_t i = 0; i < _cnt - 1; i++) {
			uint8_t m = i;
			for (uint8_t j = i + 1; j < _cnt; j++) {
				if (_as[j] < _as[m]) m = j;
			}
			if (m != i) {
				T t = _as[m];
				_as[m] = _as[i];
				_as[i] = t;
			}
		}
		_sorted = true;
	};
};

#endif


//
// FILE: FastRunningMedian.h
// AUTHOR: rkaul
// PURPOSE: RunningMedian library for Arduino
// VERSION: 0.1.01
// HISTORY:
// 0.1.00 rkaul initial version
// 0.1.01 rob.tillaart -> insertion sort
// 0.1.02 xdeschain -> insertion sort fixed bug, binary search, getHighest, getLowest, getAverage
// Released to the public domain
//
// Remarks:
// This is a lean but fast version. 
// Initially, the buffer is filled with a "default_value". To get real median values
// you have to fill the object with N values, where N is the size of the sliding window.
// For example: for(int16_t i=0; i < 32; i++) myMedian.addValue(readSensor());
//
// Constructor:
// FastRunningMedian<datatype_of_content, size_of_sliding_window, default_value>
// maximim size_of_sliding_window is 32000
// Methods:
// addValue(val) adds a new value to the buffers (and kicks the oldest)
// getMedian() returns the current median value
//
//
// Usage:
// #include "FastRunningMedian.h"
// FastRunningMedian<unsigned int,32, 0> myMedian;
// ....
// myMedian.addValue(value); // adds a value
// m = myMedian.getMedian(); // retieves the median
//

#include "Arduino.h"

#ifndef FastRunningMedian_h
#define FastRunningMedian_h

#include <inttypes.h>

template <typename T, int16_t N, T default_value> class FastRunningMedian {

public:
	FastRunningMedian() {
		_buffer_ptr = N;
		_window_size = N;
		_median_ptr = N / 2;

		// Init buffers
		int16_t i = _window_size;
		while (i > 0) {
			i--;
			_inbuffer[i] = default_value;
			_sortbuffer[i] = default_value;
		}
	};

	T getMedian() {
		// buffers are always sorted.
		return _sortbuffer[_median_ptr];
	}

	T getHighest() {
		// buffers are always sorted.
		return _sortbuffer[_window_size - 1];
	}

	T getLowest() {
		// buffers are always sorted.
		return _sortbuffer[0];
	}

	float getAverage() {
		float sum = 0;

		for (int16_t i = 0; i < _window_size; i++) {
			sum += _sortbuffer[i];
		}
		float value = sum / _window_size;
		return value;
	};


	float getAverage(int16_t nMedians) {
		if (nMedians > 0)
		{
			int16_t start = ((_window_size - nMedians) / 2);
			int16_t stop = start + nMedians;
			float sum = 0;
			for (int16_t i = start; i < stop; i++) {
				sum += _sortbuffer[i];
			}
			float value = sum / nMedians;
			return value;
		}
		return 0.0f;
	}

	void addValue(T new_value) {
		// comparision with 0 is fast, so we decrement _buffer_ptr
		if (_buffer_ptr == 0)
			_buffer_ptr = _window_size;

		_buffer_ptr--;

		T old_value = _inbuffer[_buffer_ptr]; // retrieve the old value to be replaced
		if (new_value == old_value) 		      // if the value is unchanged, do nothing
			return;

		_inbuffer[_buffer_ptr] = new_value;  // fill the new value in the cyclic buffer

											 // search the old_value in the sorted buffer with binary search
		int16_t _start = 0, _end = _window_size - 1;
		int16_t _mid = 0; //middle
		while (_start <= _end)
		{
			_mid = _end + ((_start - _end) / 2);    // average value...
			if (old_value == _sortbuffer[_mid]) { // If the middle element is the value, mid is the index of the found value
				break;
			}
			else if (old_value < _sortbuffer[_mid]) { // If the value is lesser than the  middle element, then the element must be in the left most region
				_end = _mid - 1;                      // pick the left half
			}
			else {                  // If the value is greater than the  middle element, then the element must be in the right most region
				_start = _mid + 1;  // pick the right half
			}
		}

		//Serial.print("found "); Serial.print(old_value); Serial.print("at location"); Serial.println(_mid);

		if (_start > _end) {
			Serial.print("ERROR old_value not found: "); Serial.println(old_value);
		}

		int16_t i = _mid;

		// insert in _sortbuffer
		if (new_value > old_value)
		{
			int16_t j = i + 1;
			while (j < _window_size && new_value > _sortbuffer[j])
			{
				_sortbuffer[j - 1] = _sortbuffer[j];
				j++;
			}
			_sortbuffer[j - 1] = new_value;
		}
		else
		{
			int16_t j = i - 1;
			while (j > -1 && new_value < _sortbuffer[j])
			{
				_sortbuffer[j + 1] = _sortbuffer[j];
				j--;
			}
			_sortbuffer[j + 1] = new_value;
		}
	}


private:
	// Pointer to the last added element in _inbuffer
	int16_t _buffer_ptr;
	// sliding window size
	int16_t _window_size;
	// position of the median value in _sortbuffer
	int16_t _median_ptr;

	// cyclic buffer for incoming values
	T _inbuffer[N];
	// sorted buffer
	T _sortbuffer[N];
};


#endif
