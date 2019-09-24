// rtc.h

/*
  Robotic Lawn Mower
  Copyright (c) 2017 by Kai W�rtz

  

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

#ifndef _RTC_h
#define _RTC_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "Thread.h"
#include "hardware.h"
#include "errorhandler.h"
#include "config.h"


class Trtc : public Thread
{
  private:


  public:

    bool flagShowRTCRead;

    byte hour = 0;
    byte minute = 0;
    byte dayOfWeek = 0;
    byte day = 0;
    byte month = 0;
    short year = 0;

    Trtc(): flagShowRTCRead(false) {}

    boolean findDS1307() {
      uint8_t buf[8];
      uint8_t curradd = i2cRTC.read_seven_bit_address();
      for (byte addr = 81; addr < 127; addr++) {
        errorHandler.setInfoNoLog(F("testing address= %d "), addr);
        i2cRTC.set_seven_bit_address(addr);
        if (i2cRTC.read8(0x00, 8, buf, 1) == 8) {
          errorHandler.setInfoNoLog(F(" success!\r\n"));
          i2cRTC.set_seven_bit_address(curradd);
          return true;
        }
        errorHandler.setInfoNoLog(F("\r\n"));
      }
      errorHandler.setInfoNoLog(F("error: no RTC module found\r\n"));
      i2cRTC.set_seven_bit_address(curradd);
      return false;
    }


    boolean readDS1307() {
      byte buf[8];
      if (i2cRTC.read8(0x00, 8, buf, 3) != 8) {
        errorHandler.setInfo(F("DS1307 comm error\r\n"));
        return false;
      }
      if (((buf[0] >> 7) != 0) || ((buf[1] >> 7) != 0) || ((buf[2] >> 7) != 0) || ((buf[3] >> 3) != 0)
          || ((buf[4] >> 6) != 0) || ((buf[5] >> 5) != 0) || ((buf[7] & B01101100) != 0)) {
        errorHandler.setInfo(F("DS1307 data1 error\r\n"));
        return false;
      }
      minute = 10 * ((buf[1] >> 4) & B00000111) + (buf[1] & B00001111);
      hour = 10 * ((buf[2] >> 4) & B00000111) + (buf[2] & B00001111);
      dayOfWeek = (buf[3] & B00000111) - 1;
      day = 10 * ((buf[4] >> 4) & B00000011) + (buf[4] & B00001111);
      month = 10 * ((buf[5] >> 4) & B00000001) + (buf[5] & B00001111);
      year = 10 * ((buf[6] >> 4) & B00001111) + (buf[6] & B00001111);
      if ((minute > 59) || (hour > 23) || (dayOfWeek > 6)
          || (month > 12) || (day > 31) || (day < 1)
          || (month < 1) || (year > 99)) {
        errorHandler.setInfo(F("DS1307 data2 error\r\n"));
        //addErrorCounter(ERR_RTC_DATA);
        return false;
      }
      year += 2000;
      return true;
    }


    boolean setDS1307() {
      byte buf[7];
      if (i2cRTC.read8( 0x00, 7, buf, 3) != 7) {
        errorHandler.setInfo(F("DS1307 comm error\r\n"));
        return false;
      }
      buf[0] = buf[0] & B01111111; // enable clock
      buf[1] = ((minute / 10) << 4) | (minute % 10);
      buf[2] = ((hour / 10) << 4) | (hour % 10);
      buf[3] = dayOfWeek + 1;
      buf[4] = ((day / 10) << 4) | (day % 10);
      buf[5] = ((month / 10) << 4) | (month % 10);
      buf[6] = ((year % 100 / 10) << 4) | (year % 10);
      i2cRTC.write8( 0x00, 7, buf);
      return true;
    }

    void setup() {
    }

    void showImmediately() {
      //errorHandler.setInfoNoLog(F("RTC time=%d:%d dayOfWeek=%d  date=%d.%d.%d\r\n"), hour, minute, dayOfWeek, day, month, year);
      //xdes1
      errorHandler.setInfoNoLog(F("$time,%d,%d,%d,%d,%d,%d\r\n"), hour, minute, dayOfWeek, day, month, year);
    }

    void show() {
      if (flagShowRTCRead) {
        showImmediately();
      }
    }

    void showConfig()
    {
      errorHandler.setInfoNoLog(F("!03,enabled: %lu\r\n"), enabled);
      errorHandler.setInfoNoLog(F("!03,interval: %lu\r\n"), interval);
      errorHandler.setInfoNoLog(F("!03,I2C addr: %u\r\n"), i2cRTC.read_seven_bit_address());
    }

    virtual void run() {
      // Wird alle 87ms aufgerufen
      runned();

      if (CONF_DISABLE_RTC_SERVICE) {
        return;
      }

      readDS1307();
      show();


    }


};

#endif


