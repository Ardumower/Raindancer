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


#include "RingBufferChar.h"
#include "hardware.h"
#include "errorhandler.h"
#include "config.h"

RingBufferChar::RingBufferChar()
{
	size = NODE_RINGBUFFER_MAX - 1;
	addr_w = 0;
	addr_r = 0;
}


void RingBufferChar::put(char  dat)
{
	addr_w = addr_w ? (addr_w - 1) : size;
	buf[addr_w] = dat;

	if (addr_w == addr_r) {
		// Bei ueberlauf addr_r weitersetzen
		addr_r = addr_r ? (addr_r - 1) : size;
	}
}

void RingBufferChar::putString(const char *dat) {
	for (int i = 0; dat[i] != '\0'; i++) {
		put(dat[i]);
		if (i > 200) { // if 0 at the end is missing
			errorHandler.setError(F("###### MORE THAN 200 ######\r\n"));
			break;
		}
	}
}


void RingBufferChar::putString(const __FlashStringHelper *ifsh)
{
	PGM_P p = reinterpret_cast<PGM_P>(ifsh);
	size_t i = 0;
	while (1) {
		char c = pgm_read_byte(p++);
		if (c == 0) break;
		put(c);
		i++;
		if (i > 200) { // if 0 at the end is missing
			errorHandler.setError(F("###### MORE THAN 200 F ######\r\n"));
			break;
		}

	}

}

void RingBufferChar::get(char& dat)
{
	addr_r = addr_r ? (addr_r - 1) : size;
	dat = buf[addr_r];
}


bool RingBufferChar::unreadable(void)
{
	return (addr_r == addr_w);
}

bool RingBufferChar::readable(void)
{
	return !(addr_r == addr_w);
}


void RingBufferChar::print()
{

	char character;

	addr_r_backup = addr_r;

	while (readable()) {
		get(character);
		debug->print(char(character));
		delay(1);
#if  CONF_ENABLEWATCHDOG ==  true
		watchdogReset();
#endif
	}

	// Reset addr_r in order not to delete the ringbuffer
	addr_r = addr_r_backup;

}

void RingBufferChar::clear()
{
	addr_w = 0;
	addr_r = 0;
}

