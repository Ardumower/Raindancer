/*
Ardumower (www.ardumower.de)
Copyright (c) 2013-2014 by Alexander Grau
Copyright (c) 2013-2014 by Sven Gennat

Private-use only! (you need to ask for a commercial-use)

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

Private-use only! (you need to ask for a commercial-use)
*/

#include "bt.h"
#include "hardware.h"

enum {
	BT_UNKNOWN = 0,
	BT_LINVOR_HC06 = 1, // Linvor / HC06
	BT_HC05 = 2,        // HC05                 
	BT_FBT06_MBTV4 = 3, // ModiaTek FBT06/MBTV4 
};


void BluetoothConfig::setConfigs(byte *config) {
	for (int i = 0; i < 24; i++) {
		btTestConfig[i] = config[i];
	}
}

BluetoothConfig::BluetoothConfig() {
	btType = BT_UNKNOWN;
	btRate = 9600;
#ifdef __AVR__
	byte configs[24] = { SERIAL_8N1, SERIAL_5N1, SERIAL_6N1, SERIAL_7N1,
		SERIAL_5N2, SERIAL_6N2, SERIAL_7N2, SERIAL_8N2,
		SERIAL_5E1, SERIAL_6E1, SERIAL_7E1, SERIAL_8E1,
		SERIAL_5E2, SERIAL_6E2, SERIAL_7E2, SERIAL_8E2,
		SERIAL_5O1, SERIAL_6O1, SERIAL_7O1, SERIAL_8O1,
		SERIAL_5O2, SERIAL_6O2, SERIAL_7O2, SERIAL_8O2 };
	setConfigs(configs);
	btConfig = SERIAL_8N1;
#else
	btTestConfig[1] = { 0 };
	btConfig = 0;
#endif                            
}


void BluetoothConfig::readBT() {

	btResult = "";
	if (bt.serial.available()) {
		pc.serial.print(F("  received: "));
		while (bt.readable()) {
			btData = bt.getChar();
			btResult += char(btData);
			pc.serial.print(btData);
		}
	}


	//DEBUG("btResult=");
	//DEBUGLN(btResult);
}

void BluetoothConfig::writeBT(String s) {
	pc.serial.print("send: " + s);
	bt.serial.print(s);
}


void BluetoothConfig::writeReadBT(String s) {
	writeBT(s);
	delay(2000);
	readBT();
	int counter = 0;
	while ((btResult.startsWith(F("ERROR")) && counter < 4)) {
		// retry 
		writeBT(s);
		delay(2000);
		readBT();

		counter++;
	}
	pc.serial.println();
}


void BluetoothConfig::setName(String name) {
	boolean res = false;
	pc.serial.println();
	pc.serial.print(F("setting name "));
	pc.serial.print(name);
	pc.serial.println("...");
	switch (btType) {
	case BT_LINVOR_HC06:
		writeReadBT("AT+NAME" + name);
		res = btResult.startsWith(F("OKsetname"));
		break;
	case BT_HC05:
		writeReadBT("AT+NAME=" + name + "\r\n");
		res = (btResult.indexOf(F("OK")) != -1);
		break;
	case BT_FBT06_MBTV4:
		writeReadBT("AT+NAME" + name + "\r\n");
		res = (btResult.indexOf(F("OK")) != -1);
		break;
	}
	if (res) pc.serial.println(F("=>success"));
	else pc.serial.println(F("=>error setting name"));
}


void BluetoothConfig::setPin(int pin) {
	boolean res = false;
	pc.serial.println();
	pc.serial.print(F("setting pin "));
	pc.serial.print(pin);
	pc.serial.println(F("..."));
	switch (btType) {
	case BT_LINVOR_HC06:
		writeReadBT("AT+PIN" + String(pin));
		res = (btResult.startsWith("OKsetPIN"));
		break;
	case BT_HC05:
		writeReadBT("AT+PSWD=" + String(pin) + "\r\n");
		res = (btResult.indexOf(F("OK")) != -1);
		break;
	case BT_FBT06_MBTV4:
		writeReadBT("AT+PIN" + String(pin) + "\r\n");
		res = (btResult.indexOf(F("OK")) != -1);
		break;
	}
	if (res) pc.serial.println(F("=>success"));
	else pc.serial.println(F("=>error setting pin"));
}


void BluetoothConfig::setBaudrate(long rate) {
	pc.serial.println();
	pc.serial.print(F("setting baudrate "));
	pc.serial.print(rate);
	pc.serial.println(F("..."));
	String n = "4";
	boolean res = false;
	switch (btType) {
	case BT_LINVOR_HC06:
		switch (rate) {
		case 1200: n = "1"; break;
		case 2400: n = "2"; break;
		case 4800: n = "3"; break;
		case 9600: n = "4"; break;
		case 19200: n = "5"; break;
		case 38400: n = "6"; break;
		case 57600: n = "7"; break;
		case 115200: n = "8"; break;
		case 230400: n = "9"; break;
		case 460800: n = "A"; break;
		case 921600: n = "B"; break;
		case 1382400: n = "C"; break;
		}
		writeReadBT(F("AT+PN")); // no parity
		writeReadBT("AT+BAUD" + String(n));
		res = (btResult.startsWith("OK" + String(rate)));
		break;
	case BT_HC05:
		writeReadBT("AT+UART=" + String(rate) + ",0,0" + "\r\n");
		res = (btResult.indexOf(F("OK")) != -1);
		break;
	case BT_FBT06_MBTV4:
		switch (rate) {
		case 1200: n = "1"; break;
		case 2400: n = "2"; break;
		case 4800: n = "3"; break;
		case 9600: n = "4"; break;
		case 19200: n = "5"; break;
		case 38400: n = "6"; break;
		case 57600: n = "7"; break;
		case 115200: n = "8"; break;
		case 230400: n = "9"; break;
		case 460800: n = "10"; break;
		case 921600: n = "11"; break;
		case 1382400: n = "12"; break;
		}
		writeReadBT("AT+BAUD" + String(n) + "\r\n");
		res = (btResult.indexOf(F("OK")) != -1);
		break;
	}
	if (res) {
		btRate = rate;
		pc.serial.println(F("=>success"));
	}
	else {
		pc.serial.println(F("=>error setting baudrate"));
	}
}

void BluetoothConfig::checkHC5() {
	byte btData;
	Serial2.begin(38400);
	delay(2000);
	pc.serial.println("Send \r\n");
	Serial2.write("\r\n");
	Serial2.flush();
	delay(500);
	while (Serial2.available()) {
		btData = Serial2.read();
		Serial.write(btData);
		//state = 1;
	}

	pc.serial.println("Send AT ...\r\n");
	int state = 0;
	Serial.println("Send AT ...");
	Serial2.write("AT\r\n");
	delay(2000);// prepare for next data ...
	while (Serial2.available()) {
		btData = Serial2.read();
		Serial.write(btData);
		state = 1;
	}
	if (state == 0) {
		Serial.println("No Data received");
	}
	delay(1000);// prepare for next data ...

	state = 0;
	Serial.println("Send AT ...");
	Serial2.write("AT\r\n");
	delay(2000);// prepare for next data ...
	while (Serial2.available()) {
		btData = Serial2.read();
		Serial.write(btData);
		state = 1;
	}
	if (state == 0) {
		Serial.println("No Data received");
	}


}

boolean BluetoothConfig::detectBaudrate(boolean quickBaudScan) {
	pc.serial.println();
	pc.serial.println(F("detecting baudrate..."));
	for (int i = 0; i < 8; i++) {
		switch (i) {
		case 0: btRate = 9600; break;
		case 1: btRate = 38400; break;
		case 2: btRate = 19200; break;
		case 3: btRate = 57600; break;
		case 4: btRate = 115200; break;
		case 5: btRate = 4800; break;
		case 6: btRate = 2400; break;
		case 7: btRate = 1200; break;
		}
		for (unsigned int j = 0; j < sizeof btTestConfig; j++) {
			btConfig = btTestConfig[j];
			pc.serial.print(F("trying baudrate "));
			pc.serial.print(btRate);
			pc.serial.print(F(" config "));
			pc.serial.println(j);
			pc.serial.println(F("..."));
#ifdef __AVR__
			bt.serial.begin(btRate, btConfig);
#else
			bt.serial.begin(btRate);
#endif


			writeReadBT(F("AT"));  // linvor/HC06 does not want a terminator!    
			if (btResult.startsWith(F("OK"))) {
				pc.serial.println(F("=>success"));
				return true;
			}


			writeReadBT(F("AT\r\n"));  // HC05 wants a terminator!          
			if (btResult.startsWith(F("OK"))) {
				pc.serial.println(F("=>success"));
				return true;
			}
			if (quickBaudScan) break;
		}
		//writeReadBT("ATI1\n"); // BTM info   
		//writeReadBT("ATZ0\n"); // BTM factory    
		//writeReadBT("ATC0\r\nATQ1\r\nATI1\r\n"); // BTM    
	}
	pc.serial.println(F("=>error detecting baudrate"));
	return false;
}

void BluetoothConfig::detectModuleType() {
	pc.serial.println();
	pc.serial.println(F("detecting BT type..."));
	writeReadBT(F("AT+VERSION"));
	if ((btResult.startsWith("OKlinvor")) || (btResult.startsWith("hc01"))) {
		pc.serial.println(F("=>it's a linvor/HC06"));
		btType = BT_LINVOR_HC06;
		return;
	}
	writeReadBT(F("AT+VERSION?\r\n"));
	if (btResult.indexOf(F("OK")) != -1) {
		pc.serial.println(F("=>must be a HC03/04/05 ?"));
		btType = BT_HC05;
	}
	writeReadBT(F("AT+VERSION\r\n"));
	if (btResult.indexOf(F("ModiaTek")) != -1) {
		pc.serial.println(F("=>it's a FBT06/MBTV4"));
		btType = BT_FBT06_MBTV4;
	}
}

void BluetoothConfig::printInfo() {
	pc.serial.println(F("\r\n\r\nHC-03/04/05/06/linvor/ModiaTek Bluetooth config programmer"));
	pc.serial.println(F("NOTE for HC05: Connect KEY pin to 3.3V! Or power on module while pressing button for 2sec!"));
	pc.serial.println(F("NOTE for HC06/linvor: Do NOT pair/connect (LED must be blinking)"));
	pc.serial.println(F("NOTE for FBT06/MBTV4: First you have to solder the PIO11 pin to VCC (PIN 12) which is 3.3 Volts using a thin wire."));
}

void BluetoothConfig::setParams(String name, int pin, long baudrate, boolean quickBaudScan) {
	//delay(2000);

	printInfo();
	if (detectBaudrate(quickBaudScan)) {
		detectModuleType();
		if (btType != BT_UNKNOWN) {
			setName(name);
			setPin(pin);
			setBaudrate(baudrate);
			pc.serial.println(F("You may restart BT module now!"));
		}
		else pc.serial.println(F("ERROR: Bluetooth module version not recognized"));
	}
	else pc.serial.println(F("ERROR: Bluetooth module not found"));
}



void BluetoothConfig::checkModule(boolean quickBaudScan) {
	//delay(2000);

	printInfo();
	if (detectBaudrate(quickBaudScan)) {
		detectModuleType();
		if (btType == BT_UNKNOWN)
			pc.serial.println(F("ERROR: Bluetooth module version not recognized"));
	}
	else pc.serial.println(F("ERROR: Bluetooth module not found"));
}






