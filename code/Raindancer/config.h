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
* Following belongs to cint Tgps::pnpoly(int nvert, float *vertx, float *verty, float testx, float testy)  and  the code, wich sends data to the gps module in gps.cpp
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

#ifndef _CONFIG_h
#define _CONFIG_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

//=== IMPORTANT  IMPORTANT  IMPORTANT  IMPORTANT  IMPORTANT  IMPORTANT  IMPORTANT  IMPORTANT ===========================
// Select the configuration you want to use. Only one can be true. 
// If you develop the software, you are responsible, that the changes compiles with all configurations.

#define ARDUMOWER_CHASSIS  false
#define PARANELLO_CHASSIS  false
#define RAINDANCER_CHASSIS true
#define TEST_ON_DUE_ONLY   false

//======================================================================================================================
// CONFIG FOR ARDUMOWER CHASSIS
//======================================================================================================================
#if ARDUMOWER_CHASSIS == true
#define DHTTYPE DHT22   // DHT 22  Check out DHT.h for  define types of temperature sensors.    
#define CONF_OVERHEATING_TEMP  50.0f   // if this temperature is measured, robot shuts down the complete power for security

#define CONF_PC_SERIAL_SPEED			115200 // Speed serial consol
#define CONF_BT_SERIAL_SPEED			 19200 // Speed bluetooth - Original Ardumower has 19200
#define CONF_WAN_SERIAL_SPEED           115200 // Speed for serial wan network
#define CONF_GPS_SERIAL_SPEED		 	  9600 // Serial GPS Speed
#define CONF_NATIVE_USB_SPEED           115200 // Speed for native USB port

#define CONF_DISABLE_BT                 false
#define CONF_DISABLE_WAN                true
#define CONF_DISABLE_GPS                true   
#define CONF_DISABLE_NATIVE_USB         true

#define CONF_ENABLEWATCHDOG             true   // Set to false to disable Watchdog. true to enable.

#define CONF_ENCTICKSPERREVOLUTION		1060.0f // Count of positive AND negativ encoder flanks per one revolution at the DUE pin!!!
#define CONF_RADUMFANG_CM				78.54 // Wheel circumfence in cm original ardumower: 78.54f 

#define CONF_MAX_WHEEL_RPM				30.0f	// max revolution per  minute the wheel reaches when speed is 100%. This is at pwm 255. 
//#define CONF_MAX_ENCTICKS_PM            (CONF_MAX_WHEEL_RPM*CONF_ENCTICKSPERREVOLUTION) // max encoder ticks per minute when speed is 100%
// 1060ticks/rev*30rev/minute=31800ticks/min

#define CONF_MOW_MOT_UNDER_HEAVY_LOAD_ON   45.0f  //If mow motor Watt is over this value, it is assumed that the motor is under heavy load.
#define CONF_MOW_MOT_UNDER_HEAVY_LOAD_OFF  30.0f  //If the mow moter is under heavy load, the measured watt must come under this value to reset heavy load.
#define CONF_MOW_MOT_UNDER_LOAD            30.0f  //The mow motor is under load, if this Watt is measured.

#define CONF_DISTANCE_BETWEEN_WHEELS_CM	36.5f	// Distance where the wheels hits the ground do not measure on top of the wheels!!!
#define CONF_MAX_SPIRAL_RADIUS_CM		150.0f
#define CONF_START_SPIRAL_RADIUS_CM		27.0f
#define CONF_SPIRAL_SEGMENTS			16.0f

#define CONF_DISTANCE_BETWEEN_COILS_CM	12.0f  // used for calculatin the angle while crossing the perimeter. Used in getDistanceAngleCoilOut()

#define CONF_LEFT_ENCODER_INVERSE		false
#define CONF_RIGHT_ENCODER_INVERSE		false

#define CONF_LEFT_COIL_INVERSE          false
#define CONF_RIGHT_COIL_INVERSE         false


#define CONF_DISABLE_RANGE_SERVICE		true   // Disables my own range sensor running on bumper duino on pinUserSwitch3 => diNearObsacleSensor
#define CONF_DISABLE_BUMPER_SERVICE		true   // Disables original bumper sensor on pinBumperLeft => diBumperL and pinBumperRight => diBumperR
#define CONF_DISABLE_BUMPERDUINO_SERVICE	true   // Disables my own bumper duino sensor on pinUserSwitch2 => diBumperSensor


#define CONF_DISABLE_PERIMETER_SERVICE	false   // Disables perimeter sensor
#define CONF_DISABLE_RTC_SERVICE		true    // Disables rtc sensor
#define CONF_DISABLE_EEPROM_SERVICE		false   // Disables EEPROM requests
#define CONF_DISABLE_BATTERY_SERVICE	false   // Disables battery sensor
#define CONF_DISABLE_CHARGE_SERVICE		false   // Disables charge system service
#define CONF_DISABLE_RAIN_SERVICE       true   // Disables rain sensor
#define CONF_DISABLE_DHT_SERVICE        true // Disables temp sensor

#define CONF_DISABLE_MOTOR_STALL_CHECK  false   // Disables the motor stall/encoder check in closed loop control
#define CONF_DISABLE_MOW_MOTOR          false   // Disables the mow motor

#define CONF_ACTVATE_AUTO_SPIRAL        true

#define CONF_DISABLE_FAST_RETURN        true    // Disables fast retrun 

#define CONF_DISABLE_CHARGINGSTATION    true    // If set to true, robot don't drive to chargingstation if batttery is low. After bat low, robot drives to perimeter
// and set error: "bat low". Then you should power of the robot, connect the charging contacts and power on the robot to charge.
// If you have an open switch between battery and PCB1.3 then the DUE is powered through charging contacts only while battery is disconnnected.
// To prevent this, remove diode D37. If not EF1 must be 5A minimum. 1.6A will be destroyed while connecting charing contacts with open battery switch.

#define CONF_PASS_THROUGH_CHARGING_STATION false //the mower can go through the station
#define CONF_HEAD_CHARGING_STATION         true  //the mower can't go through the station
#define CONF_HEAD_CHARGING_DRIVE_BACK_CM   100   //when the mower leaving the head charging station, how far it should drive back
#define CONF_HEAD_CHARGING_DRIVE_FORW_CM    50   //when the mower drove back and then rotates 90 degree, how far it should run forward that both coils securely inside

#define CONF_USE_ZONE_RECOGNITION        false

#define CONF_CMD_ENABLE_CONSOLE_FEEDBACK false   // Send back received serial characters on debug interface


#define CONF_PER_CORRECTION_ANGLE	    30			// TRotateBothCoilsInside rotates both coils inside. This angle must be rotated further to stand parallel to the perimeter wire. 
// Depends on the chassis construction 
#define CONF_PER_USE_COUNTER_THRESHOLD	200			// If the perimeter magnitude is below this value, use signalCounterL/R to evaluate signal otherwise use magnetude of perimetersignal.
// This is to make the signal more robust when robot is in the middle of the lawn. 
#define CONF_PER_SIGNAL_LOST_TIME	    (2000ul)	// (ms)  When no perimetersignal is received on both coils in this timerange, TCheck2PerSignal stops motors until signal is reached again

#define CONF_PER_SIGNAL_LOST_TIME_OONECOIL	(10000ul) // same as CONF_PER_SIGNAL_LOST_TIME but the coils are checked separately.  
// For example: right coil receives signal and left coil not for this time then after this time motors will be stopped. 
// because it could be, that the left coil is broken	

#define CONF_DRIVE_MAX_CM				10000.0f // 10000cm = 100m  When starting from perimeter or obstaclw the robot drives not more than this distance.
#define CONF_DRIVE_OVER_PERIMETER_CM	20.0f    // Overrun the perimeter by 20cm 	
#define CONF_PERIMETER_DRIVE_BACK_CM    40.0f    // After Perimeter overrun drive back x cm
#define CONF_PERIMETER_DRIVE_BACK_ANGLE 90.0f    // Only drive back if overrun angle is smaller than this x degree. 0 degree is both coils faces forwared perimeter.
#define CONF_BUMPER_REVERSE_CM          30.0f    // After driving forward and bumnper activated, drive back this cm to get distance from Obstacle.
#define CONF_BUMPER_SEC_REVERSE_CM      40.0f    // Driving back this cm in second reverse\reverse2 nodes.

// configure bumper service
#define CONF_USE_LEFT_BUMPER            true     // left bumper is used
#define CONF_USE_RIGHT_BUMPER           true     // right bumper is used
#define CONF_LEFT_BUMPER_LOW_ACTIVE     true     // left bumper is activated when pin is low
#define CONF_RIGHT_BUMPER_LOW_ACTIVE    true     // right bumper is activated when pin is low
#define CONF_ESCAPEDIR_DEPENDING_ON_BUMPER false  // if set to true, the robot rotates in the oposit direction of the activated buper. If false, the escape direction is random.
                                                  // only set to true if you use a bumper for left and a bumper for right.

#define CONF_NEAR_PER_UPPER_THRESHOLD   80.0L    // Threshold of one coil where Perimetersignal is detected as near perimeter
#define CONF_NEAR_PER_LOWER_THRESHOLD   70.0L    // Threshold of the other coil where Perimetersignal is detected as near perimeter

#define CONF_VOLTAGE_LOW_BS				23.7f    // Batterysensor. Batteryvoltage to go home
#define CONF_VOLTAGE_SWITCHOFF_BS		21.7f    // Batterysensor. Batteryvoltage to switch power off for the PCB1.3

//xdes1
#define CONF_WAIT_FOR_PI_SHUTDOWN       true     // If true the software waits 50sec that the due shuts down

#define CONF_bb_flagBHTShowLastNode     true    // last node will shown on terminal, also if h (hide) is pressed

#define CONF_USE128BIT_PER_SIGNAL       1
//#define CONF_USE64BIT_PER_SIGNAL          1
//#define CONF_USE32BIT_PER_SIGNAL        1

//
// Rain Sensor
//
#define CONF_RAINSENSOR_USE_DEFAULT    true    // Use a rainsensor with digital voltage output connected to P41 Rain
#define CONF_RAINSENSOR_USE_ADC        false     // Use a rainsensor with analog voltage output connected at A6
#define CONF_RAINSENSOR_ADC_THRESHOLD  2700     // When analog sensor goes under this threshold for 10seconds, it is raining

//
// GPS CONFIGURATION
//
#define CONF_GPS_PASS_THROUGH           false    // When true, all received GPS data will be sent further to the control center, when control consol output is activated with set.cco,1.
// If false, only filtered GPS data will be sent further to the control center. The code will filter out the messagetype $GPRMC (from all received GPS messages)
// and send it to the control console, when control consol output is activated with set.cco,1.

#define CONF_N_GPRMC_STR            "$GPRMC"    // GPS messagetype begin for $GPRMC for NEO-6M
//#define CONF_N_GPRMC_STR          "$GNRMC"    // GPS messagetype begin for $GPRMC for NEO-M8N
#define CONF_N_GPGGA_STR            "$GPGGA"    // GPS messagetype begin for $GPGGA. The $GPGGA record is only shown with the command gps.show


#define CONF_DEACTIVATE_GPS_CALCULATION false   // if this is true, no GPS data will be calculated on the due. You need then to set CONF_GPS_PASS_THROUGH = true, that data is sent to the control console


#define CONF_USE_GPS_POLYGON            false    // When true, the received GPS signal is checked if the  position is in the defined polygon. If yes, then the robot  is accepted to be in the perimeterwire
// independent if the received signal is valid or not and the amplitude of the perimeter is smaller than CONF_PER_THRESHOLD_IGNORE_GPS.
#define CONF_PER_THRESHOLD_IGNORE_GPS   300    // If a perimeter signal is received higher this amplitude, the perimeter signal overwrites then the gps signal.

// You have to think about that the gps sgnal is not really  precisely. You can have a deviation from +-10m. This means, if the polygon is specified to near to the perimeter, and the perimeter signal
// is broken, the gps will overwrite this and the robot can driver over the perimeter.
// configure here a polygon of gps coordinates. You can use 3 to x points.
// if the robot measure a gps signal inside the polygon, it will override the perimetersignal and
// think it is inside the perimeter.
const float CONF_LAT_POLYGON_Y[] = { 54.08728f,54.08740f, 54.08760f,54.08750f,54.08720f }; // Latitude polygon points
const float CONF_LON_POLYGON_X[] = { 7.448400f,7.448500f,7.448400f,7.448700f,7.448800f }; // Longitudinal polygon points
const int   CONF_NUMBER_OF_POLYGON_POINTS = 5;  // The number of the ploygon points

#define CONF_INIT_GPS_WITH_UBLOX        false    // if this is set to true, the ublox gps nema 6/8 module will be initialised with the configuratinon in UBLOX_INIT[]
// when the firmware is starting. This means only the GxRMC sentence is going to be send from the GPS module.

// https://www.youtube.com/watch?v=ylxwOg2pXrc
const char UBLOX_INIT[] PROGMEM =
    {
    // Disable NMEA not use sentence
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x24, // GxGGA off
    //0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28,  // GxGGA on
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B, // GxGLL off
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32, // GxGSA off
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39, // GxGSV off
    //0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x04,0x40, // GxRMC off
    0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x01,0x01,0x01,0x01,0x01,0x01,0x09,0x54, // GxRMC on
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47, // GxVTG off

    // Disable UBX
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0xDC, //NAV-PVT off
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0xB9, //NAV-POSLLH off
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0xC0, //NAV-STATUS off

    // Enable UBX
    //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x18,0xE1, //NAV-PVT on
    //0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x13, 0xBE, //NAV-POSLLH on
    //0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x14, 0xC5, //NAV-STATUS on

    // Rate
    //0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12, //(10Hz)
    //0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A, //(5Hz)
    0xB5,0x62,0x06,0x08,0x06,0x00,0xB8,0x0B,0x01,0x00,0x01,0x00,0xD9,0x41 //1 each 3 second
    //0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39, //1 each 1 second
    };

#endif //#if ARDUMOWER_CHASSIS == true
//======================================================================================================================


//======================================================================================================================
// CONFIG FOR PARANELLO CHASSIS
//======================================================================================================================
#if PARANELLO_CHASSIS == true

#define DHTTYPE DHT22   // DHT 22  Check out DHT.h for  define types of temperature sensors.     
#define CONF_OVERHEATING_TEMP  50.0f   // if this temperature is measured, robot shuts down the complete power for security

#define CONF_PC_SERIAL_SPEED     115200 // Speed serial consol
#define CONF_BT_SERIAL_SPEED      19200 // Speed bluetooth - Original Ardumower has 19200
#define CONF_WAN_SERIAL_SPEED    115200 // Speed for serial wan network
#define CONF_GPS_SERIAL_SPEED	   38400 // Serial GPS Speed
#define CONF_NATIVE_USB_SPEED   250000 // Speed for native USB port

#define CONF_DISABLE_BT                 false
#define CONF_DISABLE_WAN                true
#define CONF_DISABLE_GPS                false   
#define CONF_DISABLE_NATIVE_USB         false

#define CONF_ENABLEWATCHDOG             true   // Set to false to disable Watchdog. true to enable.

#define CONF_ENCTICKSPERREVOLUTION    1010.0f // Count of positive AND negativ encoder flanks per one revolution at the DUE pin!!!
#define CONF_RADUMFANG_CM       114.66f // Wheel circumfence in cm original ardumower: 78.54f 

#define CONF_MAX_WHEEL_RPM        35.0f // max revolution per  minute the wheel reaches when speed is 100%. This is at pwm 255. 
//#define CONF_MAX_ENCTICKS_PM            (CONF_MAX_WHEEL_RPM*CONF_ENCTICKSPERREVOLUTION) // max encoder ticks per minute when speed is 100%
// 1060ticks/rev*30rev/minute=31800ticks/min

#define CONF_MOW_MOT_UNDER_HEAVY_LOAD_ON   45.0f  //If mow motor Watt is over this value, it is assumed that the motor is under heavy load.
#define CONF_MOW_MOT_UNDER_HEAVY_LOAD_OFF  30.0f  //If the mow moter is under heavy load, the measured watt must come under this value to reset heavy load.
#define CONF_MOW_MOT_UNDER_LOAD            30.0f  //The mow motor is under load, if this Watt is measured.

#define CONF_DISTANCE_BETWEEN_WHEELS_CM 48.0f // Distance where the wheels hits the ground do not measure on top of the wheels!!!
#define CONF_MAX_SPIRAL_RADIUS_CM   150.0f
#define CONF_START_SPIRAL_RADIUS_CM   27.0f
#define CONF_SPIRAL_SEGMENTS      16.0f

#define CONF_DISTANCE_BETWEEN_COILS_CM  13.0  // used for calculatin the angle while crossing the perimeter. Used in getDistanceAngleCoilOut()

#define CONF_LEFT_ENCODER_INVERSE   false
#define CONF_RIGHT_ENCODER_INVERSE    false

#define CONF_LEFT_COIL_INVERSE          true
#define CONF_RIGHT_COIL_INVERSE         false


#define CONF_DISABLE_RANGE_SERVICE    true   // Disables my own range sensor running on bumper duino on pinUserSwitch3 => diNearObsacleSensor
#define CONF_DISABLE_BUMPER_SERVICE   false   // Disables original bumper sensor on pinBumperLeft => diBumperL and pinBumperRight => diBumperR
#define CONF_DISABLE_BUMPERDUINO_SERVICE  true   // Disables my own bumper duino sensor on pinUserSwitch2 => diBumperSensor


#define CONF_DISABLE_PERIMETER_SERVICE  false   // Disables perimeter sensor
#define CONF_DISABLE_RTC_SERVICE    false    // Disables rtc sensor
#define CONF_DISABLE_EEPROM_SERVICE   false   // Disables EEPROM requests
#define CONF_DISABLE_BATTERY_SERVICE  false   // Disables battery sensor
#define CONF_DISABLE_CHARGE_SERVICE   false   // Disables charge system service
#define CONF_DISABLE_RAIN_SERVICE     true   // Disables rain sensor
#define CONF_DISABLE_DHT_SERVICE      false // Disables temp sensor

#define CONF_DISABLE_MOTOR_STALL_CHECK  false   // Disables the motor stall/encoder check in closed loop control
#define CONF_DISABLE_MOW_MOTOR          false   // Disables the mow motor

#define CONF_ACTVATE_AUTO_SPIRAL        true

#define CONF_DISABLE_FAST_RETURN        true    // Disables fast retrun 

#define CONF_DISABLE_CHARGINGSTATION    false    // If set to true, robot don't drive to chargingstation if batttery is low. After bat low, robot drives to perimeter
// and set error: "bat low". Then you should power of the robot, connect the charging contacts and power on the robot to charge.
// If you have an open switch between battery and PCB1.3 then the DUE is powered through charging contacts only while battery is disconnnected.
// To prevent this, remove diode D37. If not EF1 must be 5A minimum. 1.6A will be destroyed while connecting charing contacts with open battery switch.

#define CONF_PASS_THROUGH_CHARGING_STATION false //the mower can go through the station
#define CONF_HEAD_CHARGING_STATION         true  //the mower can't go through the station
#define CONF_HEAD_CHARGING_DRIVE_BACK_CM   120   //when the mower leaving the head charging station, how far it should drive back
#define CONF_HEAD_CHARGING_DRIVE_FORW_CM    50   //when the mower drove back and then rotates 90 degree, how far it should run forward that both coils securely inside

#define CONF_USE_ZONE_RECOGNITION        false

#define CONF_CMD_ENABLE_CONSOLE_FEEDBACK false   // Send back received serial characters on debug interface


#define CONF_PER_CORRECTION_ANGLE     5     // TRotateBothCoilsInside rotates both coils inside. This angle must be rotated further to stand parallel to the perimeter wire. 
// Depends on the chassis construction
#define CONF_PER_USE_COUNTER_THRESHOLD  200     // If the perimeter magnitude is below this value, use signalCounterL/R to evaluate signal otherwise use magnetude of perimetersignal.
// This is to make the signal more robust when robot is in the middle of the lawn.
#define CONF_PER_SIGNAL_LOST_TIME     (2000ul)  // (ms)  When no perimetersignal is received on both coils in this timerange, TCheck2PerSignal stops motors until signal is reached again

#define CONF_PER_SIGNAL_LOST_TIME_OONECOIL  (10000ul) // same as CONF_PER_SIGNAL_LOST_TIME but the coils are checked separately.  
// For example: right coil receives signal and left coil not for this time then after this time motors will be stopped.
// because it could be, that the left coil is broken

#define CONF_DRIVE_MAX_CM       10000.0f // 10000cm = 100m  When starting from perimeter or obstaclw the robot drives not more than this distance.
#define CONF_DRIVE_OVER_PERIMETER_CM  20.0f    // Overrun the perimeter by 20cm   
#define CONF_PERIMETER_DRIVE_BACK_CM    10.0f    // After Perimeter overrun drive back x cm
#define CONF_PERIMETER_DRIVE_BACK_ANGLE 30.0f    // Only drive back if overrun angle is smaller than this x degree. 0 degree is both coils faces forwared perimeter.
#define CONF_BUMPER_REVERSE_CM          30.0f    // After driving forward and bumnper activated, drive back this cm to get distance from Obstacle.
#define CONF_BUMPER_SEC_REVERSE_CM      40.0f    // Driving back this cm in second reverse\reverse2 nodes.

// configure bumper service
#define CONF_USE_LEFT_BUMPER            true     // left bumper is used
#define CONF_USE_RIGHT_BUMPER           true     // right bumper is used
#define CONF_LEFT_BUMPER_LOW_ACTIVE     true     // left bumper is activated when pin is low
#define CONF_RIGHT_BUMPER_LOW_ACTIVE    true     // right bumper is activated when pin is low
#define CONF_ESCAPEDIR_DEPENDING_ON_BUMPER true  // if set to true, the robot rotates in the oposit direction of the activated buper. If false, the escape direction is random.
                                                 // only set to true if you use a bumper for left and a bumper for right.

#define CONF_NEAR_PER_UPPER_THRESHOLD   80.0L    // Threshold of one coil where Perimetersignal is detected as near perimeter
#define CONF_NEAR_PER_LOWER_THRESHOLD   70.0L    // Threshold of the other coil where Perimetersignal is detected as near perimeter

#define CONF_VOLTAGE_LOW_BS       23.7f    // Batterysensor. Batteryvoltage to go home
#define CONF_VOLTAGE_SWITCHOFF_BS   21.7f    // Batterysensor. Batteryvoltage to switch power off for the PCB1.3

//xdes1
#define CONF_WAIT_FOR_PI_SHUTDOWN       true     // If true the software waits 50sec that the due shuts down

#define CONF_bb_flagBHTShowLastNode     true    // last node will shown on terminal, also if h (hide) is pressed

#define CONF_USE128BIT_PER_SIGNAL       1
//#define CONF_USE64BIT_PER_SIGNAL          1
//#define CONF_USE32BIT_PER_SIGNAL        1

//
// Rain Sensor
//
#define CONF_RAINSENSOR_USE_DEFAULT    true    // Use a rainsensor with digital voltage output connected to P41 Rain
#define CONF_RAINSENSOR_USE_ADC        false     // Use a rainsensor with analog voltage output connected at A6
#define CONF_RAINSENSOR_ADC_THRESHOLD  2700     // When analog sensor goes under this threshold for 10seconds, it is raining

//
// GPS CONFIGURATION
//
#define CONF_GPS_PASS_THROUGH           true    // When true, all received GPS data will be sent further to the control center, when control consol output is activated with set.cco,1.
// If false, only filtered GPS data will be sent further to the control center. The code will filter out the messagetype $GPRMC (from all received GPS messages)
// and send it to the control console, when control consol output is activated with set.cco,1.

//#define CONF_N_GPRMC_STR            "$GPRMC"    // GPS messagetype begin for $GPRMC for NEO-6M
#define CONF_N_GPRMC_STR          "$GNRMC"    // GPS messagetype begin for $GPRMC for NEO-M8N
#define CONF_N_GPGGA_STR            "$GNGGA"    // GPS messagetype begin for $GPGGA. The $GPGGA record is only shown with the command gps.show


#define CONF_DEACTIVATE_GPS_CALCULATION true   // if this is true, no GPS data will be calculated on the due. You need then to set CONF_GPS_PASS_THROUGH = true, that data is sent to the control console


#define CONF_USE_GPS_POLYGON            false    // When true, the received GPS signal is checked if the  position is in the defined polygon. If yes, then the robot  is accepted to be in the perimeterwire
                                                 // independent if the received signal is valid or not and the amplitude of the perimeter is smaller than CONF_PER_THRESHOLD_IGNORE_GPS.
#define CONF_PER_THRESHOLD_IGNORE_GPS   300      // If a perimeter signal is received higher this amplitude, the perimeter signal overwrites then the gps signal.

// You have to think about that the gps sgnal is not really  precisely. You can have a deviation from +-10m. This means, if the polygon is specified to near to the perimeter, and the perimeter signal
// is broken, the gps will overwrite this and the robot can driver over the perimeter.
// configure here a polygon of gps coordinates. You can use 3 to x points.
// if the robot measure a gps signal inside the polygon, it will override the perimetersignal and
// think it is inside the perimeter.
const float CONF_LAT_POLYGON_Y[] = { 54.08728f,54.08740f, 54.08760f,54.08750f,54.08720f }; // Latitude polygon points
const float CONF_LON_POLYGON_X[] = { 7.448400f,7.448500f,7.448400f,7.448700f,7.448800f }; // Longitudinal polygon points
const int   CONF_NUMBER_OF_POLYGON_POINTS = 5;  // The number of the ploygon points


#define CONF_INIT_GPS_WITH_UBLOX        true    // if this is set to true, the ublox gps nema 6/8 module will be initialised with the configuratinon in UBLOX_INIT[]
// when the firmware is starting. This means only the GxRMC sentence is going to be send from the GPS module.

// https://www.youtube.com/watch?v=ylxwOg2pXrc
const char UBLOX_INIT[] PROGMEM =
    {
    // Disable NMEA not use sentence
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x24, // GxGGA off
    //0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28,  // GxGGA on
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B, // GxGLL off
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32, // GxGSA off
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39, // GxGSV off
    //0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x04,0x40, // GxRMC off
    0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x01,0x01,0x01,0x01,0x01,0x01,0x09,0x54, // GxRMC on
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47, // GxVTG off

    // Disable UBX
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0xDC, //NAV-PVT off
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0xB9, //NAV-POSLLH off
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0xC0, //NAV-STATUS off

    // Enable UBX
    //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x18,0xE1, //NAV-PVT on
    //0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x13, 0xBE, //NAV-POSLLH on
    //0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x14, 0xC5, //NAV-STATUS on

    // Rate
    //0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12, //(10Hz)
    //0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A, //(5Hz)
    0xB5,0x62,0x06,0x08,0x06,0x00,0xB8,0x0B,0x01,0x00,0x01,0x00,0xD9,0x41 //1 each 3 second
    //0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39, //1 each 1 second
    };

#endif //#if PARANELLO_CHASSIS == true
//======================================================================================================================



//======================================================================================================================
// CONFIG FOR RAINDANCER CHASSIS
//======================================================================================================================
#if RAINDANCER_CHASSIS == true

#define DHTTYPE DHT22          // DHT 22  Check out DHT.h for  define types of temperature sensors.     
#define CONF_OVERHEATING_TEMP  50.0f   // if this temperature is measured, robot shuts down the complete power for security

#define CONF_PC_SERIAL_SPEED			115200 // Speed serial consol
#define CONF_BT_SERIAL_SPEED			115200 // Speed bluetooth - Original Ardumower has 19200
#define CONF_WAN_SERIAL_SPEED           115200 // Speed for serial wan network
#define CONF_GPS_SERIAL_SPEED		 	  9600 // Serial GPS Speed
#define CONF_NATIVE_USB_SPEED          2000000 // Speed for native USB port

#define CONF_DISABLE_BT                 false
#define CONF_DISABLE_WAN                true
#define CONF_DISABLE_GPS                false   // Disabled through service
#define CONF_DISABLE_NATIVE_USB         false

#define CONF_ENABLEWATCHDOG             true   // Set to false to disable Watchdog. true to enable.

#define CONF_ENCTICKSPERREVOLUTION		1060.0f // Count of positive AND negativ encoder flanks per one revolution at the DUE pin!!!
#define CONF_RADUMFANG_CM				80.738f // Wheel circumfence in cm original ardumower: 78.54f 

#define CONF_MAX_WHEEL_RPM				30.0f	// max revolution per  minute the wheel reaches when speed is 100%. This is at pwm 255. 
//#define CONF_MAX_ENCTICKS_PM            (CONF_MAX_WHEEL_RPM*CONF_ENCTICKSPERREVOLUTION) // max encoder ticks per minute when speed is 100%
                                                    // 1060ticks/rev*30rev/minute=31800ticks/min

#define CONF_MOW_MOT_UNDER_HEAVY_LOAD_ON   45.0f  //If mow motor Watt is over this value, it is assumed that the motor is under heavy load.
#define CONF_MOW_MOT_UNDER_HEAVY_LOAD_OFF  30.0f  //If the mow moter is under heavy load, the measured watt must come under this value to reset heavy load.
#define CONF_MOW_MOT_UNDER_LOAD            30.0f  //The mow motor is under load, if this Watt is measured.

#define CONF_DISTANCE_BETWEEN_WHEELS_CM	36.0f	// Distance where the wheels hits the ground do not measure on top of the wheels!!!
#define CONF_MAX_SPIRAL_RADIUS_CM		150.0f
#define CONF_START_SPIRAL_RADIUS_CM		27.0f
#define CONF_SPIRAL_SEGMENTS			16.0f

#define CONF_DISTANCE_BETWEEN_COILS_CM	9.8  // used for calculatin the angle while crossing the perimeter. Used in getDistanceAngleCoilOut()

#define CONF_LEFT_ENCODER_INVERSE		false
#define CONF_RIGHT_ENCODER_INVERSE		false

#define CONF_LEFT_COIL_INVERSE          true
#define CONF_RIGHT_COIL_INVERSE         false


#define CONF_DISABLE_RANGE_SERVICE		false   // Disables my own range sensor running on bumper duino on pinUserSwitch3 => diNearObsacleSensor
#define CONF_DISABLE_BUMPER_SERVICE		false   // Disables original bumper sensor on pinBumperLeft => diBumperL and pinBumperRight => diBumperR
#define CONF_DISABLE_BUMPERDUINO_SERVICE	false   // Disables my own bumper duino sensor on pinUserSwitch2 => diBumperSensor


#define CONF_DISABLE_PERIMETER_SERVICE	false   // Disables perimeter sensor
#define CONF_DISABLE_RTC_SERVICE		true    // Disables rtc sensor
#define CONF_DISABLE_EEPROM_SERVICE		false   // Disables EEPROM requests
#define CONF_DISABLE_BATTERY_SERVICE	false   // Disables battery sensor
#define CONF_DISABLE_CHARGE_SERVICE		false   // Disables charge system service
#define CONF_DISABLE_RAIN_SERVICE       false   // Disables rain sensor
#define CONF_DISABLE_DHT_SERVICE        false   // Disables temp sensor

#define CONF_DISABLE_MOTOR_STALL_CHECK  false   // Disables the motor stall/encoder check in closed loop control
#define CONF_DISABLE_MOW_MOTOR          false   // Disables the mow motor

#define CONF_ACTVATE_AUTO_SPIRAL        true

#define CONF_DISABLE_FAST_RETURN        false    // Disables fast retrun 

#define CONF_DISABLE_CHARGINGSTATION    false    // If set to true, robot don't drive to chargingstation if batttery is low. After bat low, robot drives to perimeter
                                                 // and set error: "bat low". Then you should power of the robot, connect the charging contacts and power on the robot to charge.
                                                 // If you have an open switch between battery and PCB1.3 then the DUE is powered through charging contacts only while battery is disconnnected.
                                                 // To prevent this, remove diode D37. If not EF1 must be 5A minimum. 1.6A will be destroyed while connecting charing contacts with open battery switch.
#define CONF_PASS_THROUGH_CHARGING_STATION true //the mower can go through the station
#define CONF_HEAD_CHARGING_STATION         false  //the mower can't go through the station
#define CONF_HEAD_CHARGING_DRIVE_BACK_CM   100   //when the mower leaving the head charging station, how far it should drive back
#define CONF_HEAD_CHARGING_DRIVE_FORW_CM    50   //when the mower drove back and then rotates 90 degree, how far it should run forward that both coils securely inside


#define CONF_USE_ZONE_RECOGNITION        false

#define CONF_CMD_ENABLE_CONSOLE_FEEDBACK false   // Send back received serial characters on debug interface


#define CONF_PER_CORRECTION_ANGLE	    5			// TRotateBothCoilsInside rotates both coils inside. This angle must be rotated further to stand parallel to the perimeter wire. 
                                                    // Depends on the chassis construction 
#define CONF_PER_USE_COUNTER_THRESHOLD	200			// If the perimeter magnitude is below this value, use signalCounterL/R to evaluate signal otherwise use magnetude of perimetersignal.
                                                    // This is to make the signal more robust when robot is in the middle of the lawn. 
#define CONF_PER_SIGNAL_LOST_TIME	    (2000ul)	// (ms)  When no perimetersignal is received on both coils in this timerange, TCheck2PerSignal stops motors until signal is reached again

#define CONF_PER_SIGNAL_LOST_TIME_OONECOIL	(10000ul) // same as CONF_PER_SIGNAL_LOST_TIME but the coils are checked separately.  
                                                      // For example: right coil receives signal and left coil not for this time then after this time motors will be stopped. 
                                                      // because it could be, that the left coil is broken	

#define CONF_DRIVE_MAX_CM				10000.0f // 10000cm = 100m  When starting from perimeter or obstacle the robot drives not more than this distance.
#define CONF_DRIVE_OVER_PERIMETER_CM	20.0f    // Overrun the perimeter by 20cm 	
#define CONF_PERIMETER_DRIVE_BACK_CM    10.0f    // After Perimeter overrun drive back x cm
#define CONF_PERIMETER_DRIVE_BACK_ANGLE 30.0f    // Only drive back if overrun angle is smaller than this x degree. 0 degree is both coils faces forwared perimeter.
#define CONF_BUMPER_REVERSE_CM          30.0f    // After driving forward and bumnper activated, drive back this cm to get distance from Obstacle.
#define CONF_BUMPER_SEC_REVERSE_CM      40.0f    // Driving back this cm in second reverse\reverse2 nodes.

// configure bumper service
#define CONF_USE_LEFT_BUMPER            false     // left bumper is used
#define CONF_USE_RIGHT_BUMPER           true      // right bumper is used
#define CONF_LEFT_BUMPER_LOW_ACTIVE     true      // left bumper is activated when pin is low
#define CONF_RIGHT_BUMPER_LOW_ACTIVE    false     // right bumper is activated when pin is low
#define CONF_ESCAPEDIR_DEPENDING_ON_BUMPER false  // if set to true, the robot rotates in the opposite direction of the activated buper. If false, the escape direction is random.
                                                 // only set to true if you use a bumper for left and a bumper for right.

#define CONF_NEAR_PER_UPPER_THRESHOLD   80.0L    // Threshold of one coil where Perimetersignal is detected as near perimeter
#define CONF_NEAR_PER_LOWER_THRESHOLD   70.0L    // Threshold of the other coil where Perimetersignal is detected as near perimeter

#define CONF_VOLTAGE_LOW_BS				23.7f    // Batterysensor. Batteryvoltage to go home
#define CONF_VOLTAGE_SWITCHOFF_BS		21.7f    // Batterysensor. Batteryvoltage to switch power off for the PCB1.3

//xdes1
#define CONF_WAIT_FOR_PI_SHUTDOWN       true     // If true the software waits 50sec that the due shuts down

#define CONF_bb_flagBHTShowLastNode     true    // last node will shown on terminal, also if h (hide) is pressed

#define CONF_USE128BIT_PER_SIGNAL       1
//#define CONF_USE64BIT_PER_SIGNAL          1
//#define CONF_USE32BIT_PER_SIGNAL        1


//
// Rain Sensor
//
#define CONF_RAINSENSOR_USE_DEFAULT    false    // Use a rainsensor with digital voltage output connected to P41 Rain
#define CONF_RAINSENSOR_USE_ADC        true     // Use a rainsensor with analog voltage output connected at A6
#define CONF_RAINSENSOR_ADC_THRESHOLD  2700     // When analog sensor goes under this threshold for 10seconds, it is raining


//
// GPS CONFIGURATION
//
#define CONF_GPS_PASS_THROUGH           true    // When true, all received GPS data will be sent further to the control center, when control consol output is activated with set.cco,1.
// If false, only filtered GPS data will be sent further to the control center. The code will filter out the messagetype $GPRMC (from all received GPS messages)
// and send it to the control console, when control consol output is activated with set.cco,1.

#define CONF_N_GPRMC_STR            "$GPRMC"    // GPS messagetype begin for $GPRMC for NEO-6M
//#define CONF_N_GPRMC_STR          "$GNRMC"    // GPS messagetype begin for $GPRMC for NEO-M8N
#define CONF_N_GPGGA_STR            "$GPGGA"    // GPS messagetype begin for $GPGGA. The $GPGGA record is only shown with the command gps.show


#define CONF_DEACTIVATE_GPS_CALCULATION false   // if this is true, no GPS data will be calculated on the due. You need then to set CONF_GPS_PASS_THROUGH = true, that data is sent to the control console


#define CONF_USE_GPS_POLYGON            true    // When true, the received GPS signal is checked if the  position is in the defined polygon. If yes, then the robot  is accepted to be in the perimeterwire
                                                 // independent if the received signal is valid or not and the amplitude of the perimeter is smaller than CONF_PER_THRESHOLD_IGNORE_GPS.
#define CONF_PER_THRESHOLD_IGNORE_GPS   300     // If a perimeter signal is received higher this amplitude, the perimeter signal overwrites then the gps signal.

// You have to think about that the gps sgnal is not really  precisely. You can have a deviation from +-10m. This means, if the polygon is specified to near to the perimeter, and the perimeter signal
// is broken, the gps will overwrite this and the robot can driver over the perimeter.
// configure here a polygon of gps coordinates. You can use 3 to x points.
// if the robot measure a gps signal inside the polygon, it will override the perimetersignal and
// think it is inside the perimeter.
const float CONF_LAT_POLYGON_Y[] = { 54.08728f,54.08740f, 54.08760f,54.08750f,54.08720f }; // Latitude polygon points
const float CONF_LON_POLYGON_X[] = { 7.448400f,7.448500f,7.448400f,7.448700f,7.448800f }; // Longitudinal polygon points
const int   CONF_NUMBER_OF_POLYGON_POINTS = 5; // The number of the ploygon points


#define CONF_INIT_GPS_WITH_UBLOX        true    // if this is set to true, the ublox gps nema 6/8 module will be initialised with the configuratinon in UBLOX_INIT[]
// when the firmware is starting. This means only the GxRMC sentence is going to be send from the GPS module.

// https://www.youtube.com/watch?v=ylxwOg2pXrc
const char UBLOX_INIT[] PROGMEM =
    {
    // Disable NMEA not use sentence
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x24, // GxGGA off
    //0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28,  // GxGGA on
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B, // GxGLL off
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32, // GxGSA off
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39, // GxGSV off
    //0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x04,0x40, // GxRMC off
    0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x01,0x01,0x01,0x01,0x01,0x01,0x09,0x54, // GxRMC on
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47, // GxVTG off

    // Disable UBX
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0xDC, //NAV-PVT off
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0xB9, //NAV-POSLLH off
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0xC0, //NAV-STATUS off

    // Enable UBX
    //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x18,0xE1, //NAV-PVT on
    //0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x13, 0xBE, //NAV-POSLLH on
    //0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x14, 0xC5, //NAV-STATUS on

    // Rate
    //0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12, //(10Hz)
    //0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A, //(5Hz)
    0xB5,0x62,0x06,0x08,0x06,0x00,0xB8,0x0B,0x01,0x00,0x01,0x00,0xD9,0x41 //1 each 3 second
    //0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39, //1 each 1 second
    };
#endif // #if RAINDANCER_CHASSIS == true
//======================================================================================================================

//======================================================================================================================
// CONFIG FOR TESTING ON DUE only without PCB1.3
//======================================================================================================================
#if TEST_ON_DUE_ONLY == true

#define DHTTYPE DHT22          // DHT 22  Check out DHT.h for  define types of temperature sensors.     
#define CONF_OVERHEATING_TEMP  50.0f   // if this temperature is measured, robot shuts down the complete power for security

#define CONF_PC_SERIAL_SPEED			115200 // Speed serial consol
#define CONF_BT_SERIAL_SPEED			115200 // Speed bluetooth - Original Ardumower has 19200
#define CONF_WAN_SERIAL_SPEED           115200 // Speed for serial wan network
#define CONF_GPS_SERIAL_SPEED		 	  9600 // Serial GPS Speed
#define CONF_NATIVE_USB_SPEED          2000000 // Speed for native USB port

#define CONF_DISABLE_BT                 true
#define CONF_DISABLE_WAN                true
#define CONF_DISABLE_GPS                false   // Disabled through service
#define CONF_DISABLE_NATIVE_USB         false

#define CONF_ENABLEWATCHDOG             true   // Set to false to disable Watchdog. true to enable.

#define CONF_ENCTICKSPERREVOLUTION		1060.0f // Count of positive AND negativ encoder flanks per one revolution at the DUE pin!!!
#define CONF_RADUMFANG_CM				80.738f // Wheel circumfence in cm original ardumower: 78.54f 

#define CONF_MAX_WHEEL_RPM				30.0f	// max revolution per  minute the wheel reaches when speed is 100%. This is at pwm 255. 
//#define CONF_MAX_ENCTICKS_PM            (CONF_MAX_WHEEL_RPM*CONF_ENCTICKSPERREVOLUTION) // max encoder ticks per minute when speed is 100%
// 1060ticks/rev*30rev/minute=31800ticks/min

#define CONF_MOW_MOT_UNDER_HEAVY_LOAD_ON   45.0f  //If mow motor Watt is over this value, it is assumed that the motor is under heavy load.
#define CONF_MOW_MOT_UNDER_HEAVY_LOAD_OFF  30.0f  //If the mow moter is under heavy load, the measured watt must come under this value to reset heavy load.
#define CONF_MOW_MOT_UNDER_LOAD            30.0f  //The mow motor is under load, if this Watt is measured.

#define CONF_DISTANCE_BETWEEN_WHEELS_CM	36.0f	// Distance where the wheels hits the ground do not measure on top of the wheels!!!
#define CONF_MAX_SPIRAL_RADIUS_CM		150.0f
#define CONF_START_SPIRAL_RADIUS_CM		27.0f
#define CONF_SPIRAL_SEGMENTS			16.0f

#define CONF_DISTANCE_BETWEEN_COILS_CM	9.8  // used for calculatin the angle while crossing the perimeter. Used in getDistanceAngleCoilOut()

#define CONF_LEFT_ENCODER_INVERSE		false
#define CONF_RIGHT_ENCODER_INVERSE		false

#define CONF_LEFT_COIL_INVERSE          true
#define CONF_RIGHT_COIL_INVERSE         false


#define CONF_DISABLE_RANGE_SERVICE		true   // Disables my own range sensor running on bumper duino on pinUserSwitch3 => diNearObsacleSensor
#define CONF_DISABLE_BUMPER_SERVICE		true   // Disables original bumper sensor on pinBumperLeft => diBumperL and pinBumperRight => diBumperR
#define CONF_DISABLE_BUMPERDUINO_SERVICE	true   // Disables my own bumper duino sensor on pinUserSwitch2 => diBumperSensor


#define CONF_DISABLE_PERIMETER_SERVICE	false   // Disables perimeter sensor
#define CONF_DISABLE_RTC_SERVICE		true    // Disables rtc sensor
#define CONF_DISABLE_EEPROM_SERVICE		true   // Disables EEPROM requests
#define CONF_DISABLE_BATTERY_SERVICE	true   // Disables battery sensor
#define CONF_DISABLE_CHARGE_SERVICE		true   // Disables charge system service
#define CONF_DISABLE_RAIN_SERVICE       true   // Disables rain sensor
#define CONF_DISABLE_DHT_SERVICE        false   // Disables temp sensor

#define CONF_DISABLE_MOTOR_STALL_CHECK  true   // Disables the motor stall/encoder check in closed loop control
#define CONF_DISABLE_MOW_MOTOR          true   // Disables the mow motor

#define CONF_ACTVATE_AUTO_SPIRAL        true

#define CONF_DISABLE_FAST_RETURN        false    // Disables fast retrun 

#define CONF_DISABLE_CHARGINGSTATION    false    // If set to true, robot don't drive to chargingstation if batttery is low. After bat low, robot drives to perimeter
// and set error: "bat low". Then you should power of the robot, connect the charging contacts and power on the robot to charge.
// If you have an open switch between battery and PCB1.3 then the DUE is powered through charging contacts only while battery is disconnnected.
// To prevent this, remove diode D37. If not EF1 must be 5A minimum. 1.6A will be destroyed while connecting charing contacts with open battery switch.
#define CONF_PASS_THROUGH_CHARGING_STATION false //the mower can go through the station
#define CONF_HEAD_CHARGING_STATION         true  //the mower can't go through the station
#define CONF_HEAD_CHARGING_DRIVE_BACK_CM   100   //when the mower leaving the head charging station, how far it should drive back
#define CONF_HEAD_CHARGING_DRIVE_FORW_CM    50   //when the mower drove back and then rotates 90 degree, how far it should run forward that both coils securely inside


#define CONF_USE_ZONE_RECOGNITION        false

#define CONF_CMD_ENABLE_CONSOLE_FEEDBACK false   // Send back received serial characters on debug interface


#define CONF_PER_CORRECTION_ANGLE	    5			// TRotateBothCoilsInside rotates both coils inside. This angle must be rotated further to stand parallel to the perimeter wire. 
// Depends on the chassis construction 
#define CONF_PER_USE_COUNTER_THRESHOLD	200			// If the perimeter magnitude is below this value, use signalCounterL/R to evaluate signal otherwise use magnetude of perimetersignal.
// This is to make the signal more robust when robot is in the middle of the lawn. 
#define CONF_PER_SIGNAL_LOST_TIME	    (2000ul)	// (ms)  When no perimetersignal is received on both coils in this timerange, TCheck2PerSignal stops motors until signal is reached again

#define CONF_PER_SIGNAL_LOST_TIME_OONECOIL	(10000ul) // same as CONF_PER_SIGNAL_LOST_TIME but the coils are checked separately.  
// For example: right coil receives signal and left coil not for this time then after this time motors will be stopped. 
// because it could be, that the left coil is broken	

#define CONF_DRIVE_MAX_CM				10000.0f // 10000cm = 100m  When starting from perimeter or obstaclw the robot drives not more than this distance.
#define CONF_DRIVE_OVER_PERIMETER_CM	20.0f    // Overrun the perimeter by 20cm 	
#define CONF_PERIMETER_DRIVE_BACK_CM    10.0f    // After Perimeter overrun drive back x cm
#define CONF_PERIMETER_DRIVE_BACK_ANGLE 30.0f    // Only drive back if overrun angle is smaller than this x degree. 0 degree is both coils faces forwared perimeter.
#define CONF_BUMPER_REVERSE_CM          30.0f    // After driving forward and bumnper activated, drive back this cm to get distance from Obstacle.
#define CONF_BUMPER_SEC_REVERSE_CM      40.0f    // Driving back this cm in second reverse\reverse2 nodes.

// configure bumper service
#define CONF_USE_LEFT_BUMPER            false     // left bumper is used
#define CONF_USE_RIGHT_BUMPER           true      // right bumper is used
#define CONF_LEFT_BUMPER_LOW_ACTIVE     true      // left bumper is activated when pin is low
#define CONF_RIGHT_BUMPER_LOW_ACTIVE    false     // right bumper is activated when pin is low
#define CONF_ESCAPEDIR_DEPENDING_ON_BUMPER false  // if set to true, the robot rotates in the opposite direction of the activated buper. If false, the escape direction is random.
// only set to true if you use a bumper for left and a bumper for right.

#define CONF_NEAR_PER_UPPER_THRESHOLD   80.0L    // Threshold of one coil where Perimetersignal is detected as near perimeter
#define CONF_NEAR_PER_LOWER_THRESHOLD   70.0L    // Threshold of the other coil where Perimetersignal is detected as near perimeter

#define CONF_VOLTAGE_LOW_BS				23.7f    // Batterysensor. Batteryvoltage to go home
#define CONF_VOLTAGE_SWITCHOFF_BS		21.7f    // Batterysensor. Batteryvoltage to switch power off for the PCB1.3

//xdes1
#define CONF_WAIT_FOR_PI_SHUTDOWN       true    // If true the software waits 50sec that the due shuts down

#define CONF_bb_flagBHTShowLastNode     true    // last node will shown on terminal, also if h (hide) is pressed

#define CONF_USE128BIT_PER_SIGNAL       1
//#define CONF_USE64BIT_PER_SIGNAL          1
//#define CONF_USE32BIT_PER_SIGNAL        1

//
// Rain Sensor
//
#define CONF_RAINSENSOR_USE_DEFAULT    false    // Use a rainsensor with digital voltage output connected to P41 Rain
#define CONF_RAINSENSOR_USE_ADC        true     // Use a rainsensor with analog voltage output connected at A6
#define CONF_RAINSENSOR_ADC_THRESHOLD  2700     // When analog sensor goes under this threshold for 10seconds, it is raining

//
// GPS CONFIGURATION
//
#define CONF_GPS_PASS_THROUGH           true    // When true, all received GPS data will be sent further to the control center, when control consol output is activated with set.cco,1.
                                                // If false, only filtered GPS data will be sent further to the control center. The code will filter out the messagetype $GPRMC (from all received GPS messages)
                                                // and send it to the control console, when control consol output is activated with set.cco,1.

#define CONF_N_GPRMC_STR            "$GPRMC"    // GPS messagetype begin for $GPRMC for NEO-6M
//#define CONF_N_GPRMC_STR          "$GNRMC"    // GPS messagetype begin for $GPRMC for NEO-M8N
#define CONF_N_GPGGA_STR            "$GPGGA"    // GPS messagetype begin for $GPGGA. The $GPGGA record is only shown with the command gps.show


#define CONF_DEACTIVATE_GPS_CALCULATION false   // if this is true, no GPS data will be calculated on the due. You need then to set CONF_GPS_PASS_THROUGH = true, that data is sent to the control console


#define CONF_USE_GPS_POLYGON            true    // When true, the received GPS signal is checked if the  position is in the defined polygon. If yes, then the robot  is accepted to be in the perimeterwire
                                                // independent if the received signal is valid or not and the amplitude of the perimeter is smaller than CONF_PER_THRESHOLD_IGNORE_GPS.
#define CONF_PER_THRESHOLD_IGNORE_GPS   300     // If a perimeter signal is received higher this amplitude, the perimeter signal overwrites then the gps signal.

// You have to think about that the gps sgnal is not really  precisely. You can have a deviation from +-10m. This means, if the polygon is specified to near to the perimeter, and the perimeter signal
// is broken, the gps will overwrite this and the robot can driver over the perimeter.
// configure here a polygon of gps coordinates. You can use 3 to x points.
// if the robot measure a gps signal inside the polygon, it will override the perimetersignal and
// think it is inside the perimeter.
const float CONF_LAT_POLYGON_Y[] = { 54.08728f,54.08740f, 54.08760f,54.08750f,54.08720f }; // Latitude polygon points
const float CONF_LON_POLYGON_X[] = { 10.448400f,10.448500f,10.448400f,10.448700f,10.448800f }; // Longitudinal polygon points
const int   CONF_NUMBER_OF_POLYGON_POINTS = 5;  // The number of the ploygon points


#define CONF_INIT_GPS_WITH_UBLOX        true    // if this is set to true, the ublox gps nema 6/8 module will be initialised with the configuratinon in UBLOX_INIT[]
// when the firmware is starting. This means only the GxRMC sentence is going to be send from the GPS module.
// You can configure what you want to get in UBLOX_INIT




// https://www.youtube.com/watch?v=ylxwOg2pXrc
const char UBLOX_INIT[] PROGMEM =               // initial configuration for nema 6/8 modules when CONF_INIT_GPS_WITH_UBLOX is set to true
    {
    // Disable NMEA not use sentence
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x24, // GxGGA off
    //0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28,  // GxGGA on used for quality and number of satellites and altitude
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B, // GxGLL off
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32, // GxGSA off
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39, // GxGSV off
    //0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x04,0x40, // GxRMC off
    0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x01,0x01,0x01,0x01,0x01,0x01,0x09,0x54, // GxRMC on
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47, // GxVTG off

    // Disable UBX
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0xDC, //NAV-PVT off
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0xB9, //NAV-POSLLH off
    0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0xC0, //NAV-STATUS off

    // Enable UBX
    //0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x18,0xE1, //NAV-PVT on
    //0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x13, 0xBE, //NAV-POSLLH on
    //0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x14, 0xC5, //NAV-STATUS on

    // Rate
    //0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12, //(10Hz)
    //0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A, //(5Hz)
    0xB5,0x62,0x06,0x08,0x06,0x00,0xB8,0x0B,0x01,0x00,0x01,0x00,0xD9,0x41 //1 each 3 second
    //0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39, //1 each 1 second
    };

#endif // #if TEST_ON_DUE_ONLY == true
//======================================================================================================================

#endif //#ifndef _CONFIG_h



