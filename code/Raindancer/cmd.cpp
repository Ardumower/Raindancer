
/*******************************************************************
Copyright (C) 2009 FreakLabs
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

1. Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
3. Neither the name of the the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software
without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS'' AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
SUCH DAMAGE.

Originally written by Christopher Wang aka Akiba.
Please post support questions to the FreakLabs forum.

*******************************************************************/
/*!
\file Cmd.c

This implements a simple command line interface for the Arduino so that
its possible to execute individual functions within the sketch.
*/
/**************************************************************************/

#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "cmd.h"
#include "hardware.h"
#include "errorhandler.h"
#include "config.h"


// command line message buffer and pointer
static uint8_t msg[MAX_MSG_SIZE];
static uint8_t *msg_ptr;

// linked list for command table
static cmd_t *cmd_tbl_list, *cmd_tbl;

// text strings for command prompt (stored in flash)
//const char cmd_banner[] PROGMEM = "*************** CMD *******************";
//const char cmd_prompt[] PROGMEM = "CMD >> ";
const char cmd_unrecog[] PROGMEM = "CMD: Command not recognized.";

/**************************************************************************/
/*!
Generate the main command prompt
*/
/**************************************************************************/
void cmd_display()
    {
    /*
    char buf[50];

    debug.println();

    strcpy_P(buf, cmd_banner);
    debug.println(buf);

    strcpy_P(buf, cmd_prompt);
    debug.print(buf);
    */
    }

/**************************************************************************/
/*!
Parse the command line. This function tokenizes the command input, then
searches for the command table entry associated with the commmand. Once found,
it will jump to the corresponding function.
*/
/**************************************************************************/
void cmd_parse(char *cmd)
    {
    uint8_t argc, i = 0;
    char *argv[30];
    char buf[50];
    cmd_t *cmd_entry;

    debug->flush();
    //fflush(stdout);

    // parse the command line statement and break it up into space-delimited
    // strings. the array of strings will be saved in the argv array.
    argv[i] = strtok(cmd, ",");
    do
        {
        argv[++i] = strtok(NULL, ",");
        } while ((i < 30) && (argv[i] != NULL));

        // save off the number of arguments for the particular command.
        argc = i;

        // parse the command table for valid command. used argv[0] which is the
        // actual command name typed in at the prompt
        for (cmd_entry = cmd_tbl; cmd_entry != NULL; cmd_entry = cmd_entry->next)
            {
            //debug.print(argv[0]); debug.print(" "); debug->println(cmd_entry->cmd);
            if (!strcmp(argv[0], cmd_entry->cmd))
                {
                cmd_entry->func(argc, argv);
                cmd_display();
                return;
                }
            }

        // command not recognized. print message and re-generate prompt.
        strcpy_P(buf, cmd_unrecog);
        debug->println(buf);

        cmd_display();
    }

/**************************************************************************/
/*!
This function processes the individual characters typed into the command
prompt. It saves them off into the message buffer unless its a "backspace"
or "enter" key.
*/
/**************************************************************************/
void cmd_handler()
    {
    char c = debug->getChar();


    switch (c)
        {
        case '\r':
            // terminate the msg and reset the msg ptr. then send
            // it to the handler for processing.
            if (CONF_CMD_ENABLE_CONSOLE_FEEDBACK)
                {
                debug->print("\r\n");
                }
            *msg_ptr = '\0';
            cmd_parse((char *)msg);
            msg_ptr = msg;
            break;

        case '\b':
            // backspace
            if (CONF_CMD_ENABLE_CONSOLE_FEEDBACK)
                {
                debug->print(c);
                }

            if (msg_ptr > msg)
                {
                msg_ptr--;
                }
            break;
        case 0x3: // End of text ueberlesen
                  //debug.print("0x3\r\n");
            break;
        case 0x20: // Space ueberlesen
                   //debug.print("0x20\r\n");
            break;
        case '\n':  // Linefeed ueberlesen da nur auf \r reagiert wird
            break;


        case '\0':  // Terminal.exe sends by connect two times \0. This I delete here
            break;

        default:
            // normal character entered. add it to the buffer
            if (CONF_CMD_ENABLE_CONSOLE_FEEDBACK)
                {
                debug->print(c);
                }
            *msg_ptr++ = c;
            break;
        }
    }

/**************************************************************************/
/*!
This function should be set inside the main loop. It needs to be called
constantly to check if there is any available input at the command prompt.
*/
/**************************************************************************/
void cmdPoll()
    {
    int readNoOfChars = 0;
   
    if (pc.available())
        {
        debug = &pc;
        while (pc.available())
            {
            cmd_handler();

            readNoOfChars++;
            if (readNoOfChars > 30)
                {
                break; // exit while
                }
            }
        }
    else if (wan.available() && !CONF_DISABLE_WAN)
        {
        debug = &wan;
        while (wan.available())
            {
            cmd_handler();
            readNoOfChars++;
            if (readNoOfChars > 30)
                {
                break; // exit while
                }
            }
        }

    else if (bt.available() && !CONF_DISABLE_BT)
        {
        debug = &bt;
        while (bt.available())
            {
            cmd_handler();
            readNoOfChars++;
            if (readNoOfChars > 30)
                {
                break; // exit while
                }
            }
        }
    else if (nativeUSB.available() && !CONF_DISABLE_NATIVE_USB)
        {
        debug = &nativeUSB;
        while (nativeUSB.available())
            {
            cmd_handler();
            readNoOfChars++;
            if (readNoOfChars > 30)
                {
                break; // exit while
                }
            }
        }
    }

/**************************************************************************/
/*!
Initialize the command line interface. This sets the terminal speed and
and initializes things.
*/
/**************************************************************************/
void cmdInit()
    {
    // init the msg ptr
    msg_ptr = msg;

    // init the command table
    cmd_tbl_list = NULL;
    }

/**************************************************************************/
/*!
Add a command to the command table. The commands should be added in
at the setup() portion of the sketch.
*/
/**************************************************************************/
void cmdAdd(char *name, void(*func)(int argc, char **argv))
    {

    // alloc memory for command struct
    cmd_tbl = (cmd_t *)malloc(sizeof(cmd_t));

    if (cmd_tbl == NULL)
        {
        errorHandler.setError(F("cmdAdd 0 malloc could not allocate memory\r\n"));
        }
    // alloc memory for command name
    char *cmd_name = (char *)malloc(strlen(name) + 1);

    if (cmd_name == NULL)
        {
        errorHandler.setError(F("cmdAdd 1 malloc could not allocate memory\r\n"));
        }
    // copy command name
    strcpy(cmd_name, name);

    // terminate the command name
    cmd_name[strlen(name)] = '\0';

    //debug.serial.println(cmd_name);

    // fill out structure
    cmd_tbl->cmd = cmd_name;
    cmd_tbl->func = func;
    cmd_tbl->next = cmd_tbl_list;
    cmd_tbl_list = cmd_tbl;

    }

/**************************************************************************/
/*!
Convert a string to a number. The base must be specified, ie: "32" is a
different value in base 10 (decimal) and base 16 (hexadecimal).
*/
/**************************************************************************/
long cmdStr2Num(char *str, uint8_t base)
    {
    return strtol(str, NULL, base);
    }

float cmdStr2Float(char *str)
    {
    return  atof(str);
    }



bool TCmd::Run() {
      PT_BEGIN();
      while (1) {
            PT_YIELD_INTERVAL();
            cmdPoll();
      }
      PT_END();
}



