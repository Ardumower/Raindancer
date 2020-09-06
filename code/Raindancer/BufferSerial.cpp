
#include "BufferSerial.h"



BufferSerial::BufferSerial(UARTClass& s, const int& bufferSize) : serial(s), usbserial(SerialUSB) // set SerialUSB as default for usbserial that the reference has a value, which is not used because this is a serial buffer
{
	//_setup(bufferSize);
	isUSB = false;

}

BufferSerial::BufferSerial(Serial_& s, const int& bufferSize) : serial(Serial), usbserial(s) // set Serial as default for serial that teh reference has a value, which is not used because this is a usbserial buffer
{
	//_setup(bufferSize);
	isUSB = true;
}



BufferSerial :: ~BufferSerial()
{
	//delete[] _buf;
}

/*void BufferSerial::_setup(const int& bufferSize)
{
if (bufferSize > 1) {
_buf = new char[bufferSize];
_size = bufferSize - 1;
}
else {
_buf = new char[2];
_size = 1;
}
_present = 0;
_last = 0;

}
*/


/*
int BufferSerial::_getShift(volatile const int& value)
{
return value ? (value - 1) : _size;
}
*/

/*
void BufferSerial::run(void)
{

while (serial.available()) {
int n = _getShift(_last);
_buf[n] = (char)serial.read();
_last = n;
}

}
*/


int BufferSerial::available() {
	return (isUSB ? usbserial.available() : serial.available());
}

int BufferSerial::availableForWrite() {
	return (isUSB ? usbserial.availableForWrite() : serial.availableForWrite());
}

void BufferSerial::begin(unsigned long x) {
	return (isUSB ? usbserial.begin(x) : serial.begin(x));
}

void BufferSerial::flush() {
	 isUSB ? usbserial.flush() : serial.flush();
}

char BufferSerial::getChar(void)
{
	return static_cast<char>(isUSB ? usbserial.read() : serial.read());
}

size_t BufferSerial::print(const __FlashStringHelper *ifsh)
{
	return (isUSB ? usbserial.print(ifsh) : serial.print(ifsh));
}

size_t BufferSerial::print(const String &s)
{
	return (isUSB ? usbserial.print(s) : serial.print(s));
}

size_t BufferSerial::print(const char str[])
{
	return (isUSB ? usbserial.print(str) : serial.print(str));
}

size_t BufferSerial::print(char c)
{
	return (isUSB ? usbserial.print(c) : serial.print(c));
}

size_t BufferSerial::print(unsigned char b, int base)
{
	return (isUSB ? usbserial.print(b, base) : serial.print(b, base));
}

size_t BufferSerial::print(int n, int base)
{
	return (isUSB ? usbserial.print(n, base) : serial.print(n, base));
}

size_t BufferSerial::print(unsigned int n, int base)
{
	return (isUSB ? usbserial.print(n, base) : serial.print(n, base));
}

size_t BufferSerial::print(long n, int base)
{
	return (isUSB ? usbserial.print(n, base) : serial.print(n, base));
}

size_t BufferSerial::print(unsigned long n, int base)
{
	return (isUSB ? usbserial.print(n, base) : serial.print(n, base));
}

size_t BufferSerial::print(double n, int digits)
{
	return (isUSB ? usbserial.print(n,digits) : serial.print(n, digits));
}

size_t BufferSerial::println(const __FlashStringHelper *ifsh)
{
	return (isUSB ? usbserial.println(ifsh) : serial.println(ifsh));
}

size_t BufferSerial::print(const Printable& x)
{
	return (isUSB ? usbserial.print(x) : serial.print(x));
}

size_t BufferSerial::println(void)
{
	return (isUSB ? usbserial.println() : serial.println());
}

size_t BufferSerial::println(const String &s)
{
	return (isUSB ? usbserial.println(s) : serial.println(s));
}

size_t BufferSerial::println(const char c[])
{
	return (isUSB ? usbserial.println(c) : serial.println(c));
}

size_t BufferSerial::println(char c)
{
	return (isUSB ? usbserial.println(c) : serial.println(c));
}

size_t BufferSerial::println(unsigned char b, int base)
{
	return (isUSB ? usbserial.println(b,base) : serial.println(b,base));
}

size_t BufferSerial::println(int num, int base)
{
	return (isUSB ? usbserial.println(num, base) : serial.println(num, base));
}

size_t BufferSerial::println(unsigned int num, int base)
{
	return (isUSB ? usbserial.println(num, base) : serial.println(num, base));
}

size_t BufferSerial::println(long num, int base)
{
	return (isUSB ? usbserial.println(num, base) : serial.println(num, base));
}

size_t BufferSerial::println(unsigned long num, int base)
{
	return (isUSB ? usbserial.println(num, base) : serial.println(num, base));
}

size_t BufferSerial::println(double num, int digits)
{
	return (isUSB ? usbserial.println(num, digits) : serial.println(num, digits));
}

size_t BufferSerial::println(const Printable& x)
{
	return (isUSB ? usbserial.println(x) : serial.println(x));
}

size_t BufferSerial::write(const uint8_t x)
    {
    return (isUSB ? usbserial.write(x) : serial.write(x));
    }

