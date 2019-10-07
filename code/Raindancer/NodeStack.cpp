// 
// 
// 

#include "NodeStack.h"
#include "BehaviourTree.h"


NodeStack::NodeStack() : size(0) {}       // Constructor      


NodeStack::~NodeStack() {}    // Destructor

Node* NodeStack::Top() {
	if (size == 0) {
		errorHandler.setError(F("!03,Error: stack empty\r\n"));
		return NULL;
	}
	return stack[size - 1].node;
}


Node*& NodeStack::operator[](int idx) {
	return stack[idx].node;
}


NodeStackItem NodeStack::Get(int i) {
	NodeStackItem ret;
	if (i >= size) {
		errorHandler.setError(F("!03,Error: stack index greater than size\r\n"));
		ret.node = NULL;
		ret.previousStatus = BH_INVALID;
		return ret;
	}

	ret.node = stack[i].node;
	ret.previousStatus = stack[i].previousStatus;
	ret.newStatus = stack[i].newStatus;
	return ret;
}


Node* NodeStack::GetNode(int i) {
	if (i >= size) {
		errorHandler.setError(F("!03,Error: stack index greater than size\r\n"));
		return NULL;
	}
	return stack[i].node;
}

NodeStatus NodeStack::GetPrevStatus(int i) {
	if (i >= size) {
		errorHandler.setError(F("!03,Error: stack index greater than size\r\n"));
		return BH_INVALID;
	}
	return stack[i].previousStatus;
}



void NodeStack::Push(NodeStatus _previousStatus, NodeStatus _newStatus, Node* n) {
	if (size < NODE_STACK_MAX) {
		// don't push node again if existing
		/*
		for (int i = 0; i < size; i++) {
		if (d->m_nodeId == data[i]->m_nodeId) {
		return;
		}
		}
		*/

		stack[size].previousStatus = _previousStatus;
		stack[size].newStatus = _newStatus;
		stack[size].node = n;
		size++;
		//errorHandler.setInfoNoLog(F("PUSH: %s\r\n"),d->getNodeName());
	}
	else {
		errorHandler.setInfo(F("!03,********************\r\n"));
		errorHandler.setInfo(F("!03,* Info: STACK FULL *\r\n"));
		errorHandler.setInfo(F("!03,* Info: STACK FULL *\r\n"));
		errorHandler.setInfo(F("!03,* Info: STACK FULL *\r\n"));
		errorHandler.setInfo(F("!03,********************\r\n"));
	}
}

NodeStackItem NodeStack::Pop() {
	if (size == 0) {
		errorHandler.setError(F("!03,Error: stack empty\r\n"));
		size = 1;
	}

	return stack[--size];

}

void NodeStack::Clear() {
	size = 0;
}


bool NodeStack::isNotEmpty() {
	return size > 0;
}

uint16_t NodeStack::GetSize() {
	return size;
}
