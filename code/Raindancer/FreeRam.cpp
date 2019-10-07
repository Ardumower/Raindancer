// 
// https://forum.arduino.cc/index.php?topic=381203.0
// https://forum.arduino.cc/index.php?topic=404908.0
// https://forum.pjrc.com/threads/33443-How-to-display-free-ram
// 

#include "FreeRam.h"
#include "hardware.h"

#include <malloc.h>
#include <stdlib.h>
#include <stdio.h>

#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  

extern char _end;

void FreeMem(void) {
	char *ramstart = (char *)0x20070000;
	char *ramend = (char *)0x20088000;

	char Txt[256];


	char *heapend = sbrk(0);
	register char * stack_ptr asm("sp");
	struct mallinfo mi = mallinfo();

	unsigned int* ptr_i;
	unsigned int i = 0xaa;
	ptr_i = &i;

	sprintf(Txt, "Size of bool: %d\r\n", sizeof(bool)); debug->print(Txt);
	sprintf(Txt, "Size of int: %d, Size of uint: %d\r\n", sizeof(int), sizeof(unsigned int)); debug->print(Txt);
	sprintf(Txt, "Size of char: %d, Size of uchar: %d\r\n", sizeof(char), sizeof(unsigned char)); debug->print(Txt);
	sprintf(Txt, "Size of short: %d, Size of ushort: %d\r\n", sizeof(short), sizeof(unsigned short)); debug->print(Txt);
	sprintf(Txt, "Size of float: %d, Size of double: %d\r\n", sizeof(float), sizeof(double)); debug->print(Txt);
	sprintf(Txt, "Size of long: %d, Size of longlong: %d\r\n", sizeof(long), sizeof(long long)); debug->print(Txt);
	sprintf(Txt, "Size of pointer (ptr_i): %d\r\n", sizeof(ptr_i)); debug->print(Txt);
	sprintf(Txt, "Addr of pointer (ptr_i): 0x%x\r\n", (unsigned int)ptr_i); debug->print(Txt);
	sprintf(Txt, "Value of i: 0x%x\n", *ptr_i); debug->print(Txt);


	sprintf(Txt, "    arena = %d\r\n", mi.arena);     debug->print(Txt);
	sprintf(Txt, "  ordblks = %d\r\n", mi.ordblks);   debug->print(Txt);
	sprintf(Txt, " uordblks = %d\r\n", mi.uordblks);  debug->print(Txt);
	sprintf(Txt, " fordblks = %d\r\n", mi.fordblks);  debug->print(Txt);
	sprintf(Txt, " keepcost = %d\r\n", mi.keepcost);  debug->println(Txt);

	sprintf(Txt, "RAM Start:    %lx\r\n", (unsigned long)ramstart);  debug->print(Txt);
	sprintf(Txt, "Data/Bss end: %lxv\n", (unsigned long)&_end);     debug->print(Txt);
	sprintf(Txt, "Heap End:     %lx\r\n", (unsigned long)heapend);   debug->print(Txt);
	sprintf(Txt, "Stack Ptr:    %lx\r\n", (unsigned long)stack_ptr); debug->print(Txt);
	sprintf(Txt, "RAM End:      %lx\r\n", (unsigned long)ramend);    debug->println(Txt);

	sprintf(Txt, "Heap RAM Used:      %d\r\n", mi.uordblks);                       debug->print(Txt);
	sprintf(Txt, "Program RAM Used:   %d\r\n", &_end - ramstart);                  debug->print(Txt);
	sprintf(Txt, "Stack RAM Used:     %d\r\n", ramend - stack_ptr);                debug->print(Txt);
	sprintf(Txt, "Estimated Free RAM: %d\r\n", stack_ptr - heapend + mi.fordblks); debug->print(Txt);

	/* add main program code here */
	uint32_t stackTop;
	uint32_t heapTop;

	// current position of the stack.
	stackTop = (uint32_t)&stackTop;

	// current position of heap.
	void* hTop = malloc(1);
	heapTop = (uint32_t)hTop;
	free(hTop);
	// The difference is the free, available ram.
	sprintf(Txt, "Estimated Free RAM2:%lu\r\n", stackTop - heapTop); debug->print(Txt);
	char top;
	sprintf(Txt, "Estimated Free RAM3:%d\r\n", &top - reinterpret_cast<char*>(sbrk(0))); debug->print(Txt);

}

