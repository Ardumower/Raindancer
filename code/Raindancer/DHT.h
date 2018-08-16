/* DHT library

  MIT license
  written by Adafruit Industries
*/
#ifndef DHT_H
#define DHT_H

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif


#include "Thread.h"

// Uncomment to enable printing out nice debug messages.
//#define DHT_DEBUG
//#define DEBUG_PRINTER Serial

// Setup debug printing macros.
#ifdef DHT_DEBUG
#define DEBUG_PRINT(...) { DEBUG_PRINTER.print(__VA_ARGS__); }
#define DEBUG_PRINTLN(...) { DEBUG_PRINTER.println(__VA_ARGS__); }
#else
#define DEBUG_PRINT(...) {}
#define DEBUG_PRINTLN(...) {}
#endif

// Define types of sensors.
#define DHT11 11
#define DHT22 22
#define DHT21 21
#define AM2301 21

#define OVERTEMPCOUNTLIMIT 2

class TDHT : public Thread
{
  public:
    uint16_t errorCounter;

    void show();
    void hide();


    TDHT(uint8_t type);
    void setup(void);
    virtual void run();

    float getLastReadTemperature();
    float readTemperature(bool S = false, bool force = false);
    float convertCtoF(float);
    float convertFtoC(float);
    float computeHeatIndex(float temperature, float percentHumidity, bool isFahrenheit = true);
    float readHumidity(bool force = false);
    boolean read(bool force = false);

  private:
    bool flagShowTemp;


    uint8_t data[5];
    uint8_t _type;
    uint8_t overTempCounter;

    uint32_t _lastreadtime, _maxcycles;
    bool _lastresult;

    uint32_t countPulse(bool level);
    uint32_t waitForPulse(bool level);

    float dhtTempActual;

    void showData();

};



#endif
