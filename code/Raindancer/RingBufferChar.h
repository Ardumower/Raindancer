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

#ifndef _RINGBUFFERCHAR_h
#define _RINGBUFFERCHAR_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


//NODE_RINGBUFFER_MAX is limited to 32766
#define NODE_RINGBUFFER_MAX 10000

class RingBufferChar
{
public:

	RingBufferChar();

	void put(char dat);
	void putString(const char *dat);
	void putString(const __FlashStringHelper *ifsh);
	void get(char& dat);
	void resetRead();
	void print();

	void clear();

	bool unreadable(void);
	bool readable(void);

private:
	char buf[NODE_RINGBUFFER_MAX];
	int size;
	int addr_w, addr_r, addr_r_backup;
};

#endif


