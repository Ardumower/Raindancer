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

#include "helpers.h"
#include "hardware.h"



long mapl(long x, long in_min, long in_max, long out_min, long out_max)
{
	// if input is smaller/bigger than expected return the min/max out ranges value
	if (x < in_min)
		return out_min;
	else if (x > in_max)
		return out_max;

	// map the input to the output range.
	// round up if mapping bigger ranges to smaller ranges
	else  if ((in_max - in_min) > (out_max - out_min))
		return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
	// round down if mapping smaller ranges to bigger ranges
	else
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
	// if input is smaller/bigger than expected return the min/max out ranges value
	if (x < in_min)
		return out_min;
	else if (x > in_max)
		return out_max;

	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


#include "hardware.h"


long myRandom(long min, long max, bool init)  // if init == true, seedvalue will be initialised
{

	extern AnalogIn aiRandomIn;

	//static bool first_time = true;

	//if (first_time) {
	if (init) {
		//---------------------------------
		// Initialize random numbers
		//---------------------------------
		// if analog input pin is unconnected, random analog
		// noise will cause the call to randomSeed() to generate
		// different seed numbers each time i>59.
		// srand() will then shuffle the random function.
		// create a 32 bit number out of 32 LSBs from the ADC
		uint32_t seedValue = 0;
		uint16_t value;
		uint8_t counter;


		for (counter = 0; counter < 32; counter++) {
			seedValue = seedValue << 1;
			value = (uint16_t)aiRandomIn.read_int32(); // reads a 12 bit ADC normalised to 16 bits. Max=FFFF Min>10
										   //debug->printf("randomAnalogIn: %2X\r\n",value);
			//debug.serial.print("randomAnalogIn: ");
			//debug.serial.println(value);
			if (value & 0x0001)          // LSB of ADC output = 1
				seedValue++;
		}


		errorHandler.setInfo(F("Initialise Seed Value: %u\r\n"), seedValue);

		//debug.serial.print("seedValue: ");
		//debug.serial.println(seedValue);

		srand(seedValue);     // seed the random generator with the background noise of an analog input
		//first_time = false;
	}


	return min + rand() / (RAND_MAX / (max - min + 1) + 1);




	/*
	//http://www.learncpp.com/cpp-tutorial/59-random-number-generation/
	static const double fraction = 1.0 / (static_cast<double>(RAND_MAX) + 1.0);  // static used for efficiency, so we only calculate this value once
	// evenly distribute the random number across our range
	return static_cast<long>(rand() * fraction * (max - min + 1) + min);
	*/
}





