/*
  Robotic Lawn Mower
  Copyright (c) 2017 by Kai Wuertz

  

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


/************************************************************************************************************************
  Code used from
************************************************************************************************************************/
//http://www.netzmafia.de/skripten/hardware/RasPi/Projekt-GPS/index.html
//http://www.netzmafia.de/skripten/hardware/RasPi/Projekt-GPS/nmea.h
//http://www.netzmafia.de/skripten/hardware/RasPi/Projekt-GPS/nmea.c

/************************************************************************************************************************
  Following belongs to programming the GPS Module in setup
************************************************************************************************************************/
/*
  https://www.youtube.com/watch?v=ylxwOg2pXrc

  This is the Arduino code Ublox NEO-6M GPS module
  this code extracts the GPS latitude and longitude so it can be used for other purposes

  Written by Ahmad Nejrabi for Robojax Video
  Date: Jan. 24, 2017, in Ajax, Ontario, Canada
  Permission granted to share this code given that this
  note is kept with the code.
  Disclaimer: this code is "AS IS" and for educational purpose only.
*/

/************************************************************************************************************************
  Following belongs to cint Tgps::pnpoly(int nvert, float *vertx, float *verty, float testx, float testy) which caluclates the point in a poloygon
************************************************************************************************************************/
/*
  https://wrf.ecse.rpi.edu//Research/Short_Notes/pnpoly.html

  Copyright(c) 1970 - 2003, Wm.Randolph Franklin

  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and / or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

  Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimers.
  Redistributions in binary form must reproduce the above copyright notice in the documentation and / or other materials provided with the distribution.
  The name of W.Randolph Franklin may not be used to endorse or promote products derived from this Software without specific prior written permission.
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

//https://stackoverflow.com/questions/217578/how-can-i-determine-whether-a-2d-point-is-within-a-polygon

//
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "gps.h"

/*
  Output no data received. Just switched on:
  !03,$GPGGA,,,,,,0,00,99.99,,,,,,*48
  !03,GPGGA quality: 0 satellites: 0  altitude: 0.00000000
  !03,$GPRMC,,V,,,,,,,,,,N*53
  !03,GPRMC speed: 0.000000 course:0.000000
  !03,GPRMC lat: 0.00000000,? lon: 0.00000000,?
  !03,GPRMC date: 0.0.0 time: 1:0:0

  Output time and date only received.
  !03,$GPGGA,145617.00,,,,,0,00,99.99,,,,,,*66
  !03,GPGGA quality: 0 satellites: 0  altitude: 0.00000000
  !03,$GPRMC,145618.00,V,,,,,,,230718,,,N*7D
  !03,GPRMC speed: 0.000000 course:0.000000
  !03,GPRMC lat: 0.00000000,? lon: 0.00000000,?
  !03,GPRMC date: 23.7.18 time: 15:56:18

  Output all values received:
  !03,GPGGA quality: 1 satellites: 5  altitude: 41.50000000
  !03,$GPRMC,145705.00,A,5305.24783,N,00926.92039,E,0.651,338.03,230718,,,A*6F
  !03,GPRMC speed: 0.651000 course:338.030000
  !03,GPRMC lat: 53.08746400,N lon: 9.44867300,E
  !03,GPRMC date: 23.7.18 time: 15:57:5
*/


/*
  GPGGA-Record examine and evaluate
*/
void Tgps::nmea_parse_gpgga(char *nmea, struct gpgga *loc)
    {
    char *p = nmea;

    p = strchr(p, ',') + 1;
    p = strchr(p, ',') + 1;
    //loc->latitude = atof(p);

    p = strchr(p, ',') + 1;
    //switch (p[0])
    //    {
    //    case 'N':
    //        loc->lat = 'N';
    //        break;
    //    case 'S':
    //        loc->lat = 'S';
    //        break;
    //    case ',':
    //        loc->lat = '\0';
    //        break;
    //    }
    p = strchr(p, ',') + 1;
    //loc->longitude = atof(p);
    p = strchr(p, ',') + 1;
    //switch (p[0])
    //    {
    //    case 'W':
    //        loc->lon = 'W';
    //        break;
    //    case 'E':
    //        loc->lon = 'E';
    //        break;
    //    case ',':
    //        loc->lon = '\0';
    //        break;
    //    }
    p = strchr(p, ',') + 1;
    loc->quality = (char)atoi(p);
    p = strchr(p, ',') + 1;
    loc->satellites = (char)atoi(p);
    p = strchr(p, ',') + 1;
    p = strchr(p, ',') + 1;
    loc->altitude = atof(p);
    }

/*
  GPRMC-Record examine and evaluate
*/
void Tgps::nmea_parse_gprmc(char *nmea, struct gprmc *loc)
    {
    char *p = nmea;

    p = strchr(p, ',') + 1; /* store Time as double */
    loc->time = atof(p);
    p = strchr(p, ',') + 1; /* skip status */
    p = strchr(p, ',') + 1;
    loc->latitude = atof(p);
    p = strchr(p, ',') + 1;
    switch (p[0])
        {
        case 'N':
            loc->lat = 'N';
            break;
        case 'S':
            loc->lat = 'S';
            break;
        case ',':
            loc->lat = '?';
            break;
        }
    p = strchr(p, ',') + 1;
    loc->longitude = atof(p);
    p = strchr(p, ',') + 1;
    switch (p[0])
        {
        case 'W':
            loc->lon = 'W';
            break;
        case 'E':
            loc->lon = 'E';
            break;
        case ',':
            loc->lon = '?';
            break;
        }
    p = strchr(p, ',') + 1;
    loc->speed = atof(p);
    p = strchr(p, ',') + 1;
    loc->course = atof(p);
    p = strchr(p, ',') + 1;
    loc->date = atoi(p); /* store Date as long int */
    }

// str1 = string pattern which is searched in str2
char nmea_strcmp(const char *str1, const char *str2)
    {
    while (*str1 && *str1 == *str2)
        {
        ++str1, ++str2;
        }
    return *str1;
    }

/*
  Determine message type (GPGGA, GPRMC, etc.)
  Additionally: Filtering of incorrect packages (wrong checksum)
  Pass the message as parameter message
  Return value: The message type if valid
*/
int8_t Tgps::nmea_get_message_type(char *message)
    {
    char checksum = 0;

    //char r = nmea_strcmp(N_GPGGA_STR, message);
    //debug->println(message);
    //debug->print("C: ");
    //debug->println(r);
    if (nmea_strcmp(CONF_N_GPGGA_STR, message) == '\0')
        {
        if ((checksum = nmea_valid_checksum(message)) != N_EMPTY)
            {
            //return checksum;
            return N_UNKNOWN;
            }
        return N_GPGGA;
        }

    if (nmea_strcmp(CONF_N_GPRMC_STR, message) == '\0')
        {
        if ((checksum = nmea_valid_checksum(message)) != N_EMPTY)
            {
            //return checksum;
            return N_UNKNOWN;
            }
        return N_GPRMC;
        }
    return N_UNKNOWN;
    }

// Check checksum of a record
int8_t Tgps::nmea_valid_checksum(char *message)
    {
    char checksum = (char)strtol(strchr(message, '*') + 1, NULL, 16);
    char p;
    char sum = 0;
    ++message;
    while ((p = *message++) != '*')
        {
        sum ^= p;
        }

    if (sum != checksum)
        {
        return N_CHECKSUM_ERR;
        }
    return N_EMPTY;
    }


// Convert length or width from degrees to decimal notation
double Tgps::gps_deg_dec(double deg_point)
    {
    double ddeg;
    double sec = modf(deg_point, &ddeg) * 60;
    int deg = (int)(ddeg / 100);
    int min = (int)(deg_point - (deg * 100));
    double absdlat = round(deg * 1000000.0);
    double absmlat = round(min * 1000000.0);
    double absslat = round(sec * 1000000.0);
    return round(absdlat + (absmlat / 60) + (absslat / 3600)) / 1000000.0;
    }

// Convert length and width from degrees to decimal notation
void Tgps::gps_degLat(double *latitude, char ns)
    {
    double lat = (ns == 'N') ? *latitude : -1 * (*latitude);
    *latitude = gps_deg_dec(lat);
    }

void Tgps::gps_degLong(double *longitude, char we)
    {
    double lon = (we == 'E') ? *longitude : -1 * (*longitude);
    *longitude = gps_deg_dec(lon);
    }


// Convert date (stored as long int, format: ddmmyy) to string
void Tgps::gps_date2str(long int date, char *datestr)
    {
    char buf[10];
    /* date: ttmmyy */

    buf[0] = '\0';
    sprintf(datestr, "%02ld", ((date / 10000) % 100)); /* Day */
    strcat(datestr, ".");
    sprintf(buf, "%02ld", ((date / 100) % 100));       /* Month */
    strcat(datestr, buf);
    strcat(datestr, ".");
    sprintf(buf, "%02ld", (date % 100));               /* Year */
    strcat(datestr, "20");
    strcat(datestr, buf);
    }


// Convert time(saved as double, format: hhmmss.sss) to string(without decimal places of seconds)
void Tgps::gps_time2str(double time, char *timestr)
    {
    char buf[10];
    long int ttt;

    ttt = (long)time;
    buf[0] = '\0';
    sprintf(timestr, "%02ld", (1 + (ttt / 10000) % 100));   /* Hour (UTC+1 -> MESZ) */
    strcat(timestr, ":");
    sprintf(buf, "%02ld", ((ttt / 100) % 100));             /* Minute */
    strcat(timestr, buf);
    strcat(timestr, ":");
    sprintf(buf, "%02ld", (ttt % 100));                     /* Second */
    strcat(timestr, buf);
    }

void Tgps::setup()
    {
    state = 0;
    idxInString = 0;
    flagShowGPS = false;
    gpsInString[0] = '\0';
    m_gpsData.altitude = 0;
    m_gpsData.course = 0;
    m_gpsData.day = 0;
    m_gpsData.dayOfWeek = 0;
    m_gpsData.hour = 0;
    m_gpsData.lat = '\?';
    m_gpsData.latitude = 0;
    m_gpsData.lon = '\?';
    m_gpsData.longitude = 0;
    m_gpsData.minute = 0;
    m_gpsData.month = 0;
    m_gpsData.quality = '\?';
    m_gpsData.satellites = '\?';
    m_gpsData.speed = 0;
    m_gpsData.year = 0;
    m_gpsData.second = 0;

    flagInsidePolygon = false;

    if (CONF_DISABLE_GPS)
        {
        return;
        }

    // send configuration data in UBX protocol
    if (CONF_INIT_GPS_WITH_UBLOX && !CONF_DISABLE_GPS)
        {
        errorHandler.setInfo(F("Programming GPS Module\r\n"));
        // https://www.youtube.com/watch?v=ylxwOg2pXrc
        for (unsigned int i = 0; i < sizeof(UBLOX_INIT); i++)
            {
            serialGPS.write(pgm_read_byte(UBLOX_INIT + i));
            delay(5); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
            }
        }
    errorHandler.setInfo(F("Programming GPS Module finished\r\n"));
    }


// https://wrf.ecse.rpi.edu//Research/Short_Notes/pnpoly.html
// nvert	Number of vertices in the polygon.Whether to repeat the first vertex at the end is discussed below.
// vertx, verty	Arrays containing the x - and y - coordinates of the polygon's vertices.
// testx, testy	X - and y - coordinate of the test point.
int Tgps::pnpoly(const int nvert, const float *vertx, const float *verty, float testx, float testy)
    {
    int i, j, c = 0;
    for (i = 0, j = nvert - 1; i < nvert; j = i++)
        {
        if (((verty[i] > testy) != (verty[j] > testy)) &&
            (testx < (vertx[j] - vertx[i]) * (testy - verty[i]) / (verty[j] - verty[i]) + vertx[i]))
            c = !c;
        }
    return c;
    }

/*
  I use here a state machine not to loose too much time in calculations and block other services
  Therefore the GPS data is calculated step by step and not in  the whole.
*/
bool Tgps::Run()
    {
    //unsigned long time;
    long int ttt;
    int result;
    int readNoOfChars;

    // Will be called every 0ms if service is activated

    if (CONF_DISABLE_GPS)
        {
        return true;
        }

    switch (state)
        {
        case 0:  // read gps string until '\n' received
          // This routine needs just 4us to read one byte from the serial line and process it.
          // GPS send with 9600 baud. This means, when one byte was read there is no
          // further byte arrived. Therefore the service is returning to the main loop
          // and will read the next byte/bytes when be called again.
          // This prevents from blocking the main loop while receiving too much bytes at
          // the same time.
            readNoOfChars = 0;
            while (serialGPS.available())
                {
                // read only 20 chars and then go back to the loop to do other tasks. This is needed, if the gps sends a burst of data
                // and the while loop would block the whole program. If you send date with more than 9600baud, data could be lost.
                readNoOfChars++;
                if (readNoOfChars > 20)
                    {
                    break; // exit while
                    }
                // get the new byte:
                char newChar = (char)serialGPS.getChar();

                if (newChar == '$')
                    {
                    idxInString = 0;
                    }

                // write char to gpsInString
                if (newChar == '\n')
                    {
                    gpsInString[idxInString] = '\0';
                    idxInString = 0;
                    state = 1;
                    break; // exit while
                    }
                else if (newChar != '\r')
                    {
                    gpsInString[idxInString] = newChar;
                    idxInString++;
                    if (idxInString > GPSINSTRINGLENGTH - 1) // gpsInString size = GPSINSTRINGLENGTH+3
                        {
                        errorHandler.setInfo(F("!03,Tgps buffer overflow\r\n"));
                        idxInString = 0;
                        gpsInString[idxInString] = '\0';
                        }
                    }
                }
            break;

        case 1: // pass ALL receivec data to control center if CONF_GPS_PASS_THROUGH == true
            state = 2;
            if (CONF_GPS_PASS_THROUGH)
                {
                if (flagSendToCC)
                    {
                    errorHandler.setInfo(F("%s\r\n"), gpsInString);
                    }
                }
            if (CONF_DEACTIVATE_GPS_CALCULATION) // no calcualtion of GPS Data should be done on the DUE. Therefore start state 0 again.
                {
                state = 0;
                }
            break;

        case 2: // determine record type and parse gps string to structure
          // then switch to the next state for the received message type
          //time = micros();
            state = 0;  // This must stand here, to go to state 0 if case N_GPGGA: is called and  flagShowGPS is false
            sentence_type = nmea_get_message_type(gpsInString);
            switch (sentence_type)
                {
                case N_GPGGA: // only show satelites, when user wants to see this
                    if (flagShowGPS)
                        {
                        nmea_parse_gpgga(gpsInString, &m_gpgga);
                        state = 20;
                        }
                    //time = micros() - time;
                    //debug->print("timeN_GPGGA 1: ");
                    //debug->println(time);
                    break;

                case N_GPRMC:
                    nmea_parse_gprmc(gpsInString, &m_gprmc);
                    state = 30;
                    //time = micros() - time;
                    //debug->print("timeN_GPRMC 1: ");
                    //debug->println(time);
                    break;
                default:
                    state = 0;
                    break;
                }
            break;


            /***********************************************
              Handle N_GPGGA record state: 2x
            ***********************************************/
        case 20:
            // send data to controlcenter if needed
            //if (flagSendToCC && !CONF_GPS_PASS_THROUGH)
            //    {
            //    errorHandler.setInfo(F("%s\r\n"), gpsInString);
            //    }
            state = 21;
            break;

        case 21: // Fill m_gpsData
            m_gpsData.quality = m_gpgga.quality;
            m_gpsData.satellites = m_gpgga.satellites;
            m_gpsData.altitude = m_gpgga.altitude;
            state = 0;

            if (flagShowGPS)
                {
                errorHandler.setInfo(F("!03,%s\r\n"), gpsInString);
                errorHandler.setInfo(F("!03,GPGGA quality: %u satellites: %u  altitude: %.8f\r\n"), m_gpgga.quality, m_gpgga.satellites, m_gpgga.altitude);
                }
            break;


            /***********************************************
              Handle N_GPRMC record state: 3x
            ***********************************************/

        case 30:
            // send data to controlcenter if needed but not if CONF_GPS_PASS_THROUGH == true
            //if (flagSendToCC && !CONF_GPS_PASS_THROUGH)
            //    {
            //    errorHandler.setInfo(F("%s\r\n"), gpsInString);
            //    }
            state = 31;
            break;
        case 31:
            // send encoder data to controlcenter if needed but not if CONF_GPS_PASS_THROUGH == true
            //if (flagSendToCC && !CONF_GPS_PASS_THROUGH)
            //    {
            //    errorHandler.setInfo(F("$GPENC,%ld,%ld\r\n"), encoderL.getTickCounter(), encoderR.getTickCounter());
            //    }
            state = 32;
            break;
        case 32: // calculate decimal degrees latitude
            gps_degLat(&(m_gprmc.latitude), m_gprmc.lat);
            state = 33;
            break;
        case 33:  // calculate decimal degrees longitude
            gps_degLong(&(m_gprmc.longitude), m_gprmc.lon);
            state = 34;
            break;
        case 34:
            // Fill m_gpsData
            // time = micros();
            m_gpsData.speed = m_gprmc.speed;
            m_gpsData.course = m_gprmc.course;

            m_gpsData.latitude = m_gprmc.latitude;
            m_gpsData.lat = m_gprmc.lat;
            m_gpsData.longitude = m_gprmc.longitude;
            m_gpsData.lon = m_gprmc.lon;

            // Zeitangabe (gespeichert als double, Format: hhmmss.sss) (ohne Nachkommastellen der Sekunden)
            ttt = (long)m_gprmc.time;
            m_gpsData.hour = (1 + (ttt / 10000) % 100); /* Hour (UTC+1 -> MESZ) */
            m_gpsData.minute = ((ttt / 100) % 100);     /* Minute */
            m_gpsData.second = (ttt % 100);             /* Second */

            // Datumsangabe (gespeichert als long int, Format: ddmmyy)
            m_gpsData.year = (m_gprmc.date % 100);          /* Year */
            m_gpsData.month = (m_gprmc.date / 100) % 100;   /* Month */
            m_gpsData.day = ((m_gprmc.date / 10000) % 100); /* Day */

            //time = micros() - time;
            //debug->print("timeN_GPGGA 31: ");
            //debug->println(time);

            if (flagShowGPS)
                {
                errorHandler.setInfo(F("!03,%s\r\n"), gpsInString);
                errorHandler.setInfo(F("!03,%s speed: %f course:%f\r\n"), CONF_N_GPRMC_STR, m_gpsData.speed, m_gpsData.course);
                errorHandler.setInfo(F("!03,%s lat: %.8f,%c lon: %.8f,%c\r\n"), CONF_N_GPRMC_STR, m_gpsData.latitude, m_gpsData.lat, m_gpsData.longitude, m_gpsData.lon);
                errorHandler.setInfo(F("!03,%s date: %d.%d.%d time: %d:%d:%d\r\n"), CONF_N_GPRMC_STR, m_gpsData.day, m_gpsData.month, m_gpsData.year, m_gpsData.hour, m_gpsData.minute, m_gpsData.second);
                }

            if (CONF_USE_GPS_POLYGON)
                {
                state = 35;
                }
            else
                {
                state = 0;
                }

            break;

        case 35: // chek if coordiante is on polygone
          //time = micros();
            result = pnpoly(CONF_NUMBER_OF_POLYGON_POINTS, CONF_LON_POLYGON_X, CONF_LAT_POLYGON_Y, m_gpsData.longitude, m_gpsData.latitude);
            //time = micros() - time;
            //debug->print("time polygon 35: ");
            //debug->println(time);
            if (result == 0)
                {
                flagInsidePolygon = false;
                if (flagShowGPS)
                    {
                    errorHandler.setInfo(F("!03,Outside gps polygon: %d\r\n"), result);
                    }
                }
            else
                {
                flagInsidePolygon = true;
                if (flagShowGPS)
                    {
                    errorHandler.setInfo(F("!03,Inside gps polygon: %d\r\n"), result);
                    }
                }


            state = 0;
            break;


        default:
            errorHandler.setError(F("Tgps state not found: %d\r\n"), state);
            state = 0;
            break;

        }//ENDSWITCH

        return true;
    }//ENDRUN

void Tgps::showConfig()
    {
    errorHandler.setInfo(F("!03,GPS Service\r\n"));
    errorHandler.setInfo(F("!03,enabled: %d\r\n"), IsRunning());
    errorHandler.setInfo(F("!03,interval: %lu\r\n"), interval);

    errorHandler.setInfo(F("!03,flagShowGPS %d\r\n"), flagShowGPS);
    errorHandler.setInfo(F("!03,flagSendToCC %d\r\n"), flagSendToCC);

    errorHandler.setInfo(F("!03,CONF_INIT_GPS_WITH_UBLOX %d\r\n"), CONF_INIT_GPS_WITH_UBLOX);
    errorHandler.setInfo(F("!03,CONF_GPS_PASS_THROUGH %d\r\n"), CONF_GPS_PASS_THROUGH);
    errorHandler.setInfo(F("!03,CONF_N_GPRMC_STR %s\r\n"), CONF_N_GPRMC_STR);
    errorHandler.setInfo(F("!03,CONF_N_GPGGA_STR %s\r\n"), CONF_N_GPGGA_STR);
    errorHandler.setInfo(F("!03,CONF_DEACTIVATE_GPS_CALCULATION %d\r\n"), CONF_DEACTIVATE_GPS_CALCULATION);
    }
