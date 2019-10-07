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

#include "errorhandler.h"


// Forwarddeclarations
class Node;
enum NodeStatus : uint8_t;
//enum NodeAdditionalInfo : uint8_t;

#define NODE_STACK_MAX 30


typedef struct {
	Node*   node;
	NodeStatus previousStatus;
	NodeStatus newStatus;
} NodeStackItem;

class NodeStack {

public:

	NodeStack();// Constructor      
	~NodeStack();   // Destructor

	Node* Top();

	Node*& operator[](int idx);

	NodeStackItem Get(int i);
	Node* GetNode(int i);
	NodeStatus GetPrevStatus(int i);

	void Push(NodeStatus _previousStatus, NodeStatus _newStatus, Node* n);

	NodeStackItem  Pop();

	void Clear();

	bool isNotEmpty();

	uint16_t GetSize();

private:
	NodeStackItem stack[NODE_STACK_MAX];

	uint16_t  size;
};



#endif

