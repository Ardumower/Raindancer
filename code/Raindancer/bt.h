
#ifndef BT_H
#define BT_H

#include <Arduino.h>


/*
BT automatic programmer
*linvor
*HC03/04/05/06
*ModiaTek FBT06/MBTV4

NOTE for HC05:           Connect KEY pin to 3.3V!
NOTE for HC06/linvor:    Do NOT pair/connect (LED must be blinking)
NOTE for FBT06/MBTV4:    First you have to solder the PIO11 pin to VCC (PIN 12) which is 3.3 Volts using a thin wire.
*/

class BluetoothConfig
{
public:
	BluetoothConfig();
	void setParams(String name, int pin, long baudrate, boolean quickBaudScan);
	void checkModule(boolean quickBaudScan);

	void checkHC5();
private:
	void setConfigs(byte *config);
	void writeBT(String s);
	boolean detectBaudrate(boolean quickBaudScan);
	void readBT();
	void printInfo();
	void writeReadBT(String s);
	void setName(String name);
	void setPin(int pin);
	void setBaudrate(long rate);
	void detectModuleType();
	byte btTestConfig[24];
	byte btConfig;
	char btType;
	char btData;
	String btResult;
	long btRate;
};




#endif



