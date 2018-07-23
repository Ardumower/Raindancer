// 
// 
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
* GPGGA-Record examine and evaluate
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
* GPRMC-Record examine and evaluate
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
* Determine message type (GPGGA, GPRMC, etc.)
* Additionally: Filtering of incorrect packages (wrong checksum)
* Pass the message as parameter message
* Return value: The message type if valid
*/
int8_t Tgps::nmea_get_message_type(char *message)
    {
    char checksum = 0;

    //char r = nmea_strcmp(N_GPGGA_STR, message);
    //debug->println(message);
    //debug->print("C: ");
    //debug->println(r);
    if (nmea_strcmp(N_GPGGA_STR, message) == '\0')
        {
        if ((checksum = nmea_valid_checksum(message)) != N_EMPTY)
            {
            return checksum;
            }
        return N_GPGGA;
        }

    if (nmea_strcmp(N_GPRMC_STR, message) == '\0')
        {
        if ((checksum = nmea_valid_checksum(message)) != N_EMPTY)
            {
            return checksum;
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
    }


/*
* I use here a state machine not to loose too much time in calculations and block other services
* Therefore the GPS data is calculated step by step and not in in the whole.
*/
void Tgps::run()
    {
    //unsigned long time;
    long int ttt;

    // Will be called every 0ms if service is activated
    runned();

    if (CONF_DISABLE_GPS)
        {
        return;
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
            while (serialGPS.available())
                {
                // get the new byte:
                char newChar = (char)serialGPS.getChar();
                if (newChar == '$')
                    {
                    idxInString = 0;
                    }
                if (newChar == '\n')
                    {
                    gpsInString[idxInString] = '\0';
                    idxInString = 0;
                    state = 1;
                    }
                else if (newChar != '\r')
                    {
                    gpsInString[idxInString] = newChar;
                    idxInString++;
                    if (idxInString >= GPSINSTRINGLENGTH)
                        {
                        // I can put in here 0 because the gpsInString has the size GPSINSTRINGLENGTH+3
                        gpsInString[idxInString] = '\0';
                        idxInString = GPSINSTRINGLENGTH - 1;
                        errorHandler.setError(F("!03,Tgps buffer overflow%s\r\n"), gpsInString);
                        }
                    }
                }
            break;

        case 1: // determine record type and parse gps string to structure
            //time = micros();
            sentence_type = nmea_get_message_type(gpsInString);
            switch (sentence_type)
                {
                case N_GPGGA: // only show satelites, when user wants to see this
                    state = 0;
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
            * Handle N_GPGGA record state: 2x
            ***********************************************/
        case 20:
            // send data to controlcenter if needed
            //if (flagSendToCC)
            //    {
            //    errorHandler.setInfoNoLog(F("%s\r\n"), gpsInString);
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
                errorHandler.setInfoNoLog(F("!03,%s\r\n"), gpsInString);
                errorHandler.setInfoNoLog(F("!03,GPGGA quality: %u satellites: %u  altitude: %.8f\r\n"), m_gpgga.quality, m_gpgga.satellites, m_gpgga.altitude);
                }
            break;


            /***********************************************
            * Handle N_GPRMC record state: 3x
            ***********************************************/

        case 30:
            // send data to controlcenter if needed
            if (flagSendToCC)
                {
                errorHandler.setInfoNoLog(F("%s\r\n"), gpsInString);
                }
            state = 31;
            break;
        case 31:
            // send encoder data to controlcenter if needed
            if (flagSendToCC)
                {
                errorHandler.setInfoNoLog(F("$GPENC,%ld,%ld\r\n"), encoderL.getTickCounter(), encoderR.getTickCounter());
                }
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
                errorHandler.setInfoNoLog(F("!03,%s\r\n"), gpsInString);
                errorHandler.setInfoNoLog(F("!03,GPRMC speed: %f course:%f\r\n"), m_gpsData.speed, m_gpsData.course);
                errorHandler.setInfoNoLog(F("!03,GPRMC lat: %.8f,%c lon: %.8f,%c\r\n"), m_gpsData.latitude, m_gpsData.lat, m_gpsData.longitude, m_gpsData.lon);
                errorHandler.setInfoNoLog(F("!03,GPRMC date: %d.%d.%d time: %d:%d:%d\r\n"), m_gpsData.day, m_gpsData.month, m_gpsData.year, m_gpsData.hour, m_gpsData.minute, m_gpsData.second);
                }

            state = 0;
            break;


        default:
            errorHandler.setError(F("Tgps state not found: %d\r\n"), state);
            state = 0;
            break;

        }//ENDSWITCH
    }//ENDRUN

void Tgps::showConfig()
    {
    errorHandler.setInfoNoLog(F("!03,GPS Service\r\n"));
    errorHandler.setInfoNoLog(F("!03,enabled: %lu\r\n"), enabled);
    errorHandler.setInfoNoLog(F("!03,interval: %lu\r\n"), interval);
    }