/*
Robotic Lawn Mower
Copyright (c) 2017 by Kai Würtz



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

#ifndef NODESTACK_H
#define NODESTACK_H

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "hardware.h"
#include "errorhandler.h"

class Node;

#define NODE_STACK_MAX 20

class NodeStack
{

public:
    Node*   data[NODE_STACK_MAX];
    int     size;

    NodeStack() : size(0) {}       // Constructor      
    

    ~NodeStack() { }    // Destructor

    Node* Top() { 
        if (size == 0) {
            errorHandler.setError(F("!03,Error: stack empty\r\n"));
            return NULL;
        }
        return data[size-1];
    }


    Node*& operator[](int idx) {
        return data[idx];
    }

    Node* Get(int i) {
        if (i >= size) {
            errorHandler.setError(F("!03,Error: stack index greater than size\r\n"));
            return NULL;
        }
        return data[i];
    }

    void Push(Node* d) {
        if (size < NODE_STACK_MAX)
            data[size++] = d;
        else
            errorHandler.setError(F("!03,Error: stack full\r\n"));
    }

    void Pop() {
        if (size == 0)
            errorHandler.setError(F("!03,Error: stack empty\r\n"));
        else
            size--;
    }

    void Clear() {
        size = 0;
    }
};



#endif

