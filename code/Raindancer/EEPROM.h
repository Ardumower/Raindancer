// EEPROM.h
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
#ifndef _EEPROM_h
#define _EEPROM_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "Protothread.h"
#include "hardware.h"
#include "errorhandler.h"





#define EEP_PAGESIZE 32			// pagesize is 32byte

#define EEPADR_INC 4			// increment adresses by 4 

#define EEPADR_MOWTIME			0
#define EEPADR_MOWDIRVENWAY		(1*EEPADR_INC)
#define EEPADR_CHARGINGCOUNT	(2*EEPADR_INC)
#define EEPADR_ROTATIONCOUNT	(3*EEPADR_INC)

#define EEPADR_ERASE_PAGES  3 // first pages will be erased erase(). No need to erase more at the moment. These are 24 4Byte Values.


class TEEPROM : public Protothread {
public:
	TEEPROM();
	void setup();
	virtual bool Run();
	void writeu8t(uint16_t address, uint8_t data);
	void writeFloat(uint16_t address, float data);
	void write32t(uint16_t address, int32_t data);
	void write16t(uint16_t address, int16_t data);
	void writeChars(unsigned int address, char *data, int length);

	uint8_t readu8t(uint16_t address);
	int16_t read16t(uint16_t address);
	int32_t  read32t(uint16_t address);
	float readFloat(uint16_t address);
	void readChars(unsigned int address, char *data, int n);

	void erase();
	void showConfig();

protected:
	void write(unsigned int address, uint8_t *data, int n);
	void read(unsigned int address, byte *data, int n);
private:

};

#endif

