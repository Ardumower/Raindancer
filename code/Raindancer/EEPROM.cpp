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

/*

https://forum.arduino.cc/index.php?topic=403038.0

You do understand that this device has a 32 byte write page size?
This means that any time you write to the device, 32 bytes are actually erased, written.
And you have to take care when writing multiple bytes that all of the bytes you send fit on one 32 byte page?

This means that the chip is organized in multiple 32 byte pages:
Code: [Select]

page    address Range
0           0..31
1          32..63
2          64..95


If you use code like this to write a multibyte buffer to the EEPROM:
Code: [Select]

void writeBlock(uint8_t i2cAddress, uint16_t addr, char *ch, uint8_t len){
Wire.beginTransmission(i2cAddress);
Wire.write(highByte(addr));
Wire.write(lowByte(addr));
Wire.write(ch,len);
Wire.endTransmission();
}


It will not correctly store the data if (((addr+len)/32) != (addr/32)).  This is because of the 32 byte write buffer.

The way I2C EEPROMS store data is something like this.

The Chip receives the Write command, it decodes the address, finding which of the 32Byte pages you will be changing.
Then it fills the page write buffer with the current contents of it's EEPROM page.
Then it starts overwriting the page write buffer with the data coming in the I2C buss.
It has a 5bit index register (0..31) that indicates where in the buffer the next byte is to be stored.
After each byte is received from the I2C buss, this index pointer is incremented.  If it is incremented past 31, it starts over at 0.
So, this mean that if your multibyte buffer is 10bytes long and you want it stored starting at address 25,
your first 7 bytes are stored at addresses[25..31], and the remaining 3 bytes are stored at addresses [0..2].
After the I2C write instruction has completed, the Chip Erases the current page, then writes the data from the page write buffer into the newly erased memory cells.

So, if you are going to write multiple bytes in one I2C transaction, you must consider the page size of the EEPROM you are using.
The write page size is device dependent.  Some of the little 2k EEPROMs have as small as a 4byte page size, some of the larger ones have 256 byte buffers.

Chuck.
*/
#include "EEPROM.h"
#include "errno.h"

union uFloat {
	uint8_t uBytes[4];
	float sFloat;
};

union uInt32 {
	uint8_t uBytes[4];
	int32_t sIn32t;
};

union uInt16 {
	uint8_t uBytes[2];
	int16_t sIn16t;
};

union uInt8 {
	uint8_t uBytes[1];
	int8_t sIn8t;
};

union uuInt8 {
	uint8_t uBytes[1];
	uint8_t uIn8t;
};

TEEPROM::TEEPROM() {

}

void TEEPROM::setup() {
}

void TEEPROM::run() {
	runned();

}

/**
* Write sequence of n bytes
* https://github.com/cyberp/AT24Cx/blob/master/AT24CX.cpp
*/

void TEEPROM::write(unsigned int address, uint8_t *data, int n) {
	// status quo
	int c = n;						// bytes left to write
	int offD = 0;					// current offset in data pointer
	int offP;						// current offset in page
	int nc = 0;						// next n bytes to write
	uint8_t i = 1;
	
	// write all bytes in multiple steps
	while (c > 0) {
		// calc offset in page
		offP = address % EEP_PAGESIZE;
		// maximal 30 bytes to write.  This size is equal to BUFFER_LENGTH - 2 bytes reserved for address.
		nc = min(min(c, 30), EEP_PAGESIZE - offP);
		uint8_t *dataPointer = data + offD;
		//errorHandler.setInfo("Address %ud  dp: %ul\r\n", address, dataPointer);
		i2cEEPROM.write16(address, nc, dataPointer);
		c -= nc;
		offD += nc;
		address += nc;
		

		//wait up to 50ms for the write to complete https://github.com/JChristensen/extEEPROM/blob/master/extEEPROM.cpp
		for (i = 100; i; --i) {
			delayMicroseconds(500);   //no point in waiting too fast
			if (i2cEEPROM.ping(2,false)) {
				break;
			}
		}

		if (i == 0) {
			errorHandler.setInfo(F("Could not write to EEPROM\r\n"));
		}
	}
}


/**
* Read sequence of n bytes
* https://github.com/cyberp/AT24Cx/blob/master/AT24CX.cpp
*/
void TEEPROM::read(unsigned int address, byte *data, int n) {
	int c = n;
	int offD = 0;
	// read until n bytes are read
	while (c > 0) {
		// read maximal 32 bytes
		int nc = c;
		if (nc > 32)
			nc = 32;

		uint8_t *dataPointer = data + offD;
		i2cEEPROM.read16(address, nc, dataPointer, 3);
		address += nc;
		offD += nc;
		c -= nc;
	}
}


void TEEPROM::writeu8t(uint16_t address, uint8_t data) {
	uuInt8 i;
	i.uIn8t = data;
	write(address, i.uBytes, 1);
	//i2cEEPROM.write16(address, 1, i.uBytes);
}

void TEEPROM::write16t(uint16_t address, int16_t data) {
	uInt16 i;
	i.sIn16t = data;
	write(address, i.uBytes, 2);
	//i2cEEPROM.write16(address, 2, i.uBytes);
}

void TEEPROM::write32t(uint16_t address, int32_t data) {
	uInt32 i;
	i.sIn32t = data;

	write(address, i.uBytes, 4);
	//i2cEEPROM.write16(address, 4, i.uBytes);
}

void TEEPROM::writeFloat(uint16_t address, float data) {
	uFloat f;
	f.sFloat = data;
	write(address, f.uBytes, 4);
	//i2cEEPROM.write16(address, 4, f.uBytes);
}

void TEEPROM::writeChars(unsigned int address, char *data, int length) {
	write(address, (byte*)data, length);
}


uint8_t TEEPROM::readu8t(uint16_t address) {
	uuInt8 i;
	read(address, i.uBytes, 1);
	//i2cEEPROM.read16(address, 1, i.uBytes, 3);
	return i.uIn8t;
}

int16_t TEEPROM::read16t(uint16_t address) {
	uInt16 i;
	read(address, i.uBytes, 2);
	//i2cEEPROM.read16(address, 2, i.uBytes, 3);
	return i.sIn16t;
}

int32_t TEEPROM::read32t(uint16_t address) {
	uInt32 i;
	read(address, i.uBytes, 4);
	//i2cEEPROM.read16(address, 4, i.uBytes, 3);
	return i.sIn32t;
}

float TEEPROM::readFloat(uint16_t address) {
	uFloat f;
	read(address, f.uBytes, 4);
	//i2cEEPROM.read16(address, 4, f.uBytes,3);
	return f.sFloat;
}



void TEEPROM::erase() {
	uint8_t _b[EEP_PAGESIZE];
	
	for (int j = 0; j < EEP_PAGESIZE; j++) {
		_b[j] = 0;
	}

	for (int i = 0; i < EEPADR_ERASE_PAGES; i++) {
		errorHandler.setInfoNoLog(F("!03,Erasing page %d\r\n"), i);
		write(i*EEP_PAGESIZE, _b, EEP_PAGESIZE);
		delay(50);
	}
}


void TEEPROM::readChars(unsigned int address, char *data, int n) {
	read(address, (byte*)data, n);
}


void TEEPROM::showConfig()
{
	errorHandler.setInfoNoLog(F("!03,enabled: %lu\r\n"), enabled);
	errorHandler.setInfoNoLog(F("!03,interval: %lu\r\n"), interval);
	errorHandler.setInfoNoLog(F("!03,I2C addr: %u\r\n"), i2cEEPROM.read_seven_bit_address());
}