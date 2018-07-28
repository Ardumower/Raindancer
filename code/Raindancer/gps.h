// gps.h
//http://www.netzmafia.de/skripten/hardware/RasPi/Projekt-GPS/index.html
//http://www.netzmafia.de/skripten/hardware/RasPi/Projekt-GPS/nmea.h
//http://www.netzmafia.de/skripten/hardware/RasPi/Projekt-GPS/nmea.c

#ifndef _GPS_h
#define _GPS_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "hardware.h"
#include "errorhandler.h"
#include "config.h"
#include "thread.h"


#define N_EMPTY 0x00
#define N_GPRMC 0x01
#define N_GPGGA 0x02
#define N_UNKNOWN 0x00
#define N_COMPLETE 0x03
#define N_CHECKSUM_ERR 0x80
#define N_MESSAGE_ERR 0xC0

#define GPSINSTRINGLENGTH 120

//#define GRABCHARLENGTH 20

/* Struktur fuer GPGGA-Record */
struct gpgga
    {
    //char Latitude_Temp[GRABCHARLENGTH + 1];           // Latitude field, grab chars
    //char Longitude_Temp[GRABCHARLENGTH + 1];          // Longitude field, grab chars

    double latitude;           /* degree of latitude */
    char lat;                  /* 'N' or 'S' */
    double longitude;          /* degree of longitude */
    char lon;                  /* 'E' or 'W' */
    unsigned char quality;     /* "Quality"-Field */
    unsigned char satellites;  /* Number of satellites */
    double altitude;           /* Above sea level */
    };

/* Struktur fuer GPRMC-Record */
struct gprmc
    {
    double latitude;           /* degree of latitude */
    char lat;                  /* 'N' or 'S' */
    double longitude;          /* degree of longitude */
    char lon;                  /* 'E' or 'W' */
    double speed;              /* speed */
    double course;             /* march direction */
    double time;               /* time of day */
    long int date;             /* date */
    };

struct gpsData
    {
    double latitude;           /* degree of latitude */
    char lat;                  /* 'N' or 'S' */
    double longitude;          /* degree of longitude */
    char lon;                  /* 'E' or 'W' */
    unsigned char quality;     /* "Quality"-Field */
    unsigned char satellites;  /* Number of satellites */
    double altitude;           /* Above sea level */
    double speed;              /* speed */
    double course;             /* march direction */

    byte hour = 0;
    byte minute = 0;
    byte second = 0;
    byte dayOfWeek = 0;
    byte day = 0;
    byte month = 0;
    short year = 0;
    };

class Tgps : public Thread
    {
    private:

        uint8_t state = 0;
        uint8_t sentence_type;

        int  idxInString;
        char gpsInString[GPSINSTRINGLENGTH + 3];

        struct gpgga m_gpgga;
        struct gprmc m_gprmc;

        
        // GPGGA-Record examine and evaluate
        void nmea_parse_gpgga(char *nmea, struct gpgga *loc);
 
        // GPRMC-Record examine and evaluate
        void nmea_parse_gprmc(char *nmea, struct gprmc *loc);

        /*
        * Determine message type (GPGGA, GPRMC, etc.)
        * Additionally: Filtering of incorrect packages (wrong checksum)
        * Pass the message as parameter message
        * Return value: The message type if valid
        */
        int8_t nmea_get_message_type(char *message);

        // Check checksum of a record
        int8_t nmea_valid_checksum(char *message);

        // Convert length or width from degrees to decimal notation
        double gps_deg_dec(double deg_point);

        // Convert length and width from degrees to decimal notation
        void gps_degLat(double *latitude, char ns);
        void gps_degLong(double *longitude, char we);

        // Convert date (stored as long int, format: ddmmyy) to string
        void gps_date2str(long int date, char *datestr);
        // Convert time(saved as double, format: hhmmss.sss) to string(without decimal places of seconds)
        void gps_time2str(double time, char *timestr);
        
        // is point inside polygon
        int pnpoly(int nvert, const float *vertx, const float *verty, float testx, float testy);

    public:
        bool flagInsidePolygon;
        bool flagShowGPS;  // show calculated gps data 
        bool flagSendToCC; // send data to control center
        gpsData m_gpsData; // pgs date determined

        void setup();
        virtual void run();
        void showConfig();

    };



#endif

