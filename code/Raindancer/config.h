// config.h

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

//======================================================================================================================
// CONFIG FOR ARDUMOWER CHASSIS
//======================================================================================================================
#if ARDUMOWER_CHASSIS == true
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321      

#define CONF_PASS_THROUGH_CHARGING_STATION true //the mower can cross over the station
#define CONF_HEAD_CHARGING_STATION false //the mower can't cross over the station

#define CONF_PC_SERIAL_SPEED			115200 // Speed serial consol
#define CONF_BT_SERIAL_SPEED			19200 // Speed bluetooth - Original Ardumower has 19200

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

#define CONF_DISABLE_CHARGINGSTATION    true    // If set to true, robot don't drive to chargingstation if batttery is low. After bat low, robot drives to perimeter
// and set error: "bat low". Then you should power of the robot, connect the charging contacts and power on the robot to charge.
// If you have an open switch between battery and PCB1.3 then the DUE is powered through charging contacts only while battery is disconnnected.
// To prevent this, remove diode D37. If not EF1 must be 5A minimum. 1.6A will be destroyed while connecting charing contacts with open battery switch.

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

#define CONF_bb_flagBHTShowLastNode     true    // last node will shown on terminal, also if h (hide) is pressed

#define CONF_USE128BIT_PER_SIGNAL       1
//#define CONF_USE64BIT_PER_SIGNAL          1
//#define CONF_USE32BIT_PER_SIGNAL        1


#endif //#if ARDUMOWER_CHASSIS == true
//======================================================================================================================


//======================================================================================================================
// CONFIG FOR PARANELLO CHASSIS
//======================================================================================================================
#if PARANELLO_CHASSIS == true

#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321      

#define CONF_PASS_THROUGH_CHARGING_STATION false //the mower can cross over the station
#define CONF_HEAD_CHARGING_STATION true //the mower can't cross over the station

#define CONF_PC_SERIAL_SPEED     115200 // Speed serial consol
#define CONF_BT_SERIAL_SPEED      19200 // Speed bluetooth - Original Ardumower has 19200

#define CONF_ENABLEWATCHDOG             true   // Set to false to disable Watchdog. true to enable.

#define CONF_ENCTICKSPERREVOLUTION    1010.0f // Count of positive AND negativ encoder flanks per one revolution at the DUE pin!!!
#define CONF_RADUMFANG_CM       114.66f // Wheel circumfence in cm original ardumower: 78.54f 

#define CONF_MAX_WHEEL_RPM        30.0f // max revolution per  minute the wheel reaches when speed is 100%. This is at pwm 255. 
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
#define CONF_DISABLE_RTC_SERVICE    true    // Disables rtc sensor
#define CONF_DISABLE_EEPROM_SERVICE   false   // Disables EEPROM requests
#define CONF_DISABLE_BATTERY_SERVICE  false   // Disables battery sensor
#define CONF_DISABLE_CHARGE_SERVICE   false   // Disables charge system service
#define CONF_DISABLE_RAIN_SERVICE     true   // Disables rain sensor
#define CONF_DISABLE_DHT_SERVICE      false // Disables temp sensor

#define CONF_DISABLE_MOTOR_STALL_CHECK  false   // Disables the motor stall/encoder check in closed loop control
#define CONF_DISABLE_MOW_MOTOR          false   // Disables the mow motor

#define CONF_ACTVATE_AUTO_SPIRAL        true

#define CONF_DISABLE_CHARGINGSTATION    true    // If set to true, robot don't drive to chargingstation if batttery is low. After bat low, robot drives to perimeter
// and set error: "bat low". Then you should power of the robot, connect the charging contacts and power on the robot to charge.
// If you have an open switch between battery and PCB1.3 then the DUE is powered through charging contacts only while battery is disconnnected.
// To prevent this, remove diode D37. If not EF1 must be 5A minimum. 1.6A will be destroyed while connecting charing contacts with open battery switch.

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

#define CONF_bb_flagBHTShowLastNode     true    // last node will shown on terminal, also if h (hide) is pressed

#define CONF_USE128BIT_PER_SIGNAL       1
//#define CONF_USE64BIT_PER_SIGNAL          1
//#define CONF_USE32BIT_PER_SIGNAL        1
#endif //#if PARANELLO_CHASSIS == true
//======================================================================================================================



//======================================================================================================================
// CONFIG FOR RAINDANCER CHASSIS
//======================================================================================================================
#if RAINDANCER_CHASSIS == true

#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321      

#define CONF_PASS_THROUGH_CHARGING_STATION true //the mower can cross over the station
#define CONF_HEAD_CHARGING_STATION false //the mower can't cross over the station

#define CONF_PC_SERIAL_SPEED			115200 // Speed serial consol
#define CONF_BT_SERIAL_SPEED			115200 // Speed bluetooth - Original Ardumower has 19200

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

#define CONF_DISABLE_CHARGINGSTATION    false    // If set to true, robot don't drive to chargingstation if batttery is low. After bat low, robot drives to perimeter
                                                 // and set error: "bat low". Then you should power of the robot, connect the charging contacts and power on the robot to charge.
												 // If you have an open switch between battery and PCB1.3 then the DUE is powered through charging contacts only while battery is disconnnected.
												 // To prevent this, remove diode D37. If not EF1 must be 5A minimum. 1.6A will be destroyed while connecting charing contacts with open battery switch.

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

#define CONF_bb_flagBHTShowLastNode     true    // last node will shown on terminal, also if h (hide) is pressed

#define CONF_USE128BIT_PER_SIGNAL       1
//#define CONF_USE64BIT_PER_SIGNAL          1
//#define CONF_USE32BIT_PER_SIGNAL        1


#endif // #if RAINDANCER_CHASSIS == true
//======================================================================================================================



#endif //#ifndef _CONFIG_h

