/*
Behaviour Tree
Copyright (c) 2019 by Kai Würtz

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


The base of this code comes from https://github.com/aigamedev/btsk
I have extended it to own needs.
*/

#include "BehaviourTree.h"

//#define DEBUG_BHT_TREE 1
//#define DEBUG_BHT_SETUP 1

// Action, Condition, WaitDecorator and ParallelUntilFail will always be logged. One can enable LOG_CONTROL_NODES
// to see how the tree run. The values will be shown, after the tick comes back to the behavior tree.

#define LOG_CONTROL_NODES 1



#ifdef DEBUG_BHT_TREE
#  define DTREE(x) x
#else
#  define DTREE(x) 
#endif

#ifdef DEBUG_BHT_SETUP
#  define DSETUP(x) x
#else
#  define DSETUP(x) 
#endif

#ifdef LOG_CONTROL_NODES
#	define SAVE_OLD_STATUS NodeStatus oldState = m_eNodeStatus;
#	define PUSH_STACK  if (m_eNodeStatus != oldState) { \
					bb.changedStatusNodes.Push(oldState, m_eNodeStatus, this); \
				 }
#	define PUSH_STACK_ABORT  bb.changedStatusNodes.Push(oldState, m_eNodeStatus, this); 
#     define PUSH_STACK_TRY_ABORT_CHILD  bb.changedStatusNodes.Push(BH_EMPTY, BH_TRY_TO_ABORT_CHILD, this); 
#else
#	define SAVE_OLD_STATUS
#	define PUSH_STACK
#	define PUSH_STACK_ABORT
#     define PUSH_STACK_TRY_ABORT_CHILD
#endif



#ifdef BHT_USE_SERVICES
#  define RUNSERVICE(b) runService(b);
#else
#  define RUNSERVICE(b) 
#endif


static int nodeIdCounter = 0;

const char* enumNodeStatusStrings[] = { "BH_FAILURE", "BH_SUCCESS", "BH_RUNNING", "BH_ABORTED", "BH_INVALID","TRUE","FALSE", "SET_FLAG: FALSE", "WAITING", "WAITTIME_EXPIRED", "TRY_TO_ABORT_CHILD(REN)", "", "WRONG1","WRONG2","WRONG3" };

const char* enumNodeAddInfoStrings[] = { "FALSE", "TRUE", "SET_FLAG: FALSE", "WAITING", "WAITTIME_EXPIRED", "TRY_TO_ABORT_CHILD", " ", "WRONG1","WRONG2","WRONG3" };

// ============================================================================
// Node
Node::Node()
	: m_eNodeStatus(BH_INVALID) {
	m_nodeId = nodeIdCounter++;
}

Node::~Node() {
}

bool Node::onSetup(Blackboard& bb) {
	DSETUP(errorHandler.setInfoNoLog(F("%s %d\t\t Node::onSetup -> status: %s\r\n"), m_nodeName, m_nodeId, enumNodeStatusStrings[m_eNodeStatus]););
	return true;
}

NodeStatus Node::getNodeStatus() const {
	return m_eNodeStatus;
}

int16_t Node::getNodeID() const {
	return m_nodeId;
}

char* Node::getNodeName() const {
	return m_nodeName;
}

void Node::setNodeName(char* s) {
	m_nodeName = s;
}

void Node::reset() {
	m_eNodeStatus = BH_INVALID;
}

bool Node::isTerminated() const {
	return m_eNodeStatus == BH_SUCCESS || m_eNodeStatus == BH_FAILURE;
}

/*
inline bool Node::isRunning() const {
return m_eNodeStatus == BH_RUNNING;
}*/


bool Node::isInvalid() const {
	return m_eNodeStatus == BH_INVALID;
}


// ============================================================================
// Action

void Action::onInitialize(Blackboard& bb) {
}

void Action::onTerminate(NodeStatus status, Blackboard& bb) {
}

Action::Action() {
}

Action::~Action() {
}

// This function will be called to run a node.
NodeStatus Action::tick(Blackboard& bb) {

	NodeStatus oldState = m_eNodeStatus;

	if (m_eNodeStatus != BH_RUNNING) {
		_start_time_of_node = millis();
		DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t act::tick -> calling onInitialize status: %s\r\n"), m_nodeName, m_nodeId, enumNodeStatusStrings[m_eNodeStatus]););
		onInitialize(bb);
	}

	m_eNodeStatus = onUpdate(bb);
	DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t act::tick -> called onUpdate returned status: %s\r\n"), m_nodeName, m_nodeId, enumNodeStatusStrings[m_eNodeStatus]););


	if (m_eNodeStatus != oldState) {
		bb.changedStatusNodes.Push(oldState, m_eNodeStatus, this);
	}


	if (m_eNodeStatus != BH_RUNNING) {
		DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t act::tick -> calling onTerminate status: %s\r\n"), m_nodeName, m_nodeId, enumNodeStatusStrings[m_eNodeStatus]););
		onTerminate(m_eNodeStatus, bb);
	}

	DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t act::tick -> returning status to parent: %s\r\n"), m_nodeName, m_nodeId, enumNodeStatusStrings[m_eNodeStatus]););
	return m_eNodeStatus;
}

void Action::abort(Blackboard& bb) {
	NodeStatus oldState = m_eNodeStatus;
	DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t act::abort -> executing onTerminate\r\n"), m_nodeName, m_nodeId););
	onTerminate(BH_ABORTED, bb);
	m_eNodeStatus = BH_ABORTED;
	bb.changedStatusNodes.Push(oldState, m_eNodeStatus, this);
}

unsigned long Action::getTimeInNode() {
	return millis() - _start_time_of_node;
}

void Action::setTimeInNode(unsigned long t) {
	_start_time_of_node = t;
}

// ============================================================================
//
//  Action class with condition
//  Works likes a conditon and action added to a Sequence
//  The condition will always be called, before the Action will be done
//  If condition is BH_SUCCESS the Action node will be called.
//  If condition returns with BH_FAILURE, the node returns with BH_FAILURE. If the node is also in running state, abort will be called before returning BH_FAILURE.

NodeStatus ActionCond::tick(Blackboard& bb) {
	NodeStatus result;
	NodeStatus oldState = m_eNodeStatus;

	result = onCheckCondition(bb);

	if (result == BH_SUCCESS) {
		DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t ActionCond::tick -> cond returned BH_SUCCESS current status: %s\r\n"), m_nodeName, m_nodeId, enumNodeStatusStrings[m_eNodeStatus]););
		return Action::tick(bb);
	}

	if (isRunning()) { // Is current node running, then abort ActionCond
		DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t ActionCond::tick -> cond BH_FAILURE and node running -> call abort %s\r\n"), m_nodeName, m_nodeId, enumNodeStatusStrings[m_eNodeStatus]););
		abort(bb);
		oldState = m_eNodeStatus;
	}
	// Node is not running when code comes here and condition was false

	// if condition returns runnig, error should be thrown
	if (result != BH_FAILURE) {
		errorHandler.setError(F("%s %d\t\t ActionCond::tick -> condition returned != BH_FAILURE\r\n"), m_nodeName, m_nodeId);
	}


	DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t ActionCond::tick -> returning failure to parent: %s\r\n"), m_nodeName, m_nodeId, enumNodeStatusStrings[m_eNodeStatus]););
	m_eNodeStatus = BH_FAILURE;
	if (m_eNodeStatus != oldState) {
		bb.changedStatusNodes.Push(oldState, m_eNodeStatus, this);
	}
	return BH_FAILURE;
}

// ============================================================================
//
//  Action class with memory condition
//  Works likes a conditon and action added to a MemSequence
//  The condition will not be called if the node is in status RUNNING
//  If condition returns BH_FAILURE, the node returns with FAIURE

NodeStatus ActionMemCond::tick(Blackboard& bb) {
	NodeStatus result;
	NodeStatus oldState = m_eNodeStatus;

	if (isRunning()) { // Is current node running, then don't query condition.
		DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t ActionMemCond::tick -> cond not called node because node is in status %s\r\n"), m_nodeName, m_nodeId, enumNodeStatusStrings[m_eNodeStatus]););
		return Action::tick(bb);
	}

	result = onCheckCondition(bb);

	if (result == BH_SUCCESS) {
		DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t ActionMemCond::tick -> cond returned BH_SUCCESS current status %s\r\n"), m_nodeName, m_nodeId, enumNodeStatusStrings[m_eNodeStatus]););
		return Action::tick(bb);
	}

	// Node is not running, therfore I don't need to abort here.

	// if condition returns runnig, error should be thrown
	// if condition returns runnig, error should be thrown
	if (result != BH_FAILURE) {
		errorHandler.setError(F("%s %d\t\t ActionMemCond::tick -> condition returned != BH_FAILURE\r\n"), m_nodeName, m_nodeId);
	}

	DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t ActionMemCond::tick ->cond returned BH_FAILURE returning failure to parent: %s\r\n"), m_nodeName, m_nodeId, enumNodeStatusStrings[m_eNodeStatus]););
	m_eNodeStatus = BH_FAILURE;
	if (m_eNodeStatus != oldState) {
		bb.changedStatusNodes.Push(oldState, m_eNodeStatus, this);
	}
	return BH_FAILURE;
}


// ============================================================================
// Condition
// The condition node has only onCheckCondition function. It should only query things and never change things!
Condition::Condition() {
}

Condition::~Condition() {
}

// This function will be called to run a node.
NodeStatus Condition::tick(Blackboard& bb) {

	NodeStatus oldState;

	oldState = m_eNodeStatus;

	m_eNodeStatus = onCheckCondition(bb);

	if (m_eNodeStatus != oldState) {
		bb.changedStatusNodes.Push(oldState, m_eNodeStatus, this);
	}

	DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t cond::tick -> returning status to parent: %s\r\n"), m_nodeName, m_nodeId, enumNodeStatusStrings[m_eNodeStatus]););
	return m_eNodeStatus;
}

void Condition::abort(Blackboard& bb) {
	NodeStatus oldState = m_eNodeStatus;
	DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t act::abort -> executing onTerminate\r\n"), m_nodeName, m_nodeId););
	m_eNodeStatus = BH_ABORTED;
	bb.changedStatusNodes.Push(oldState, m_eNodeStatus, this);

}
// ============================================================================
//
//  Service Class
//  Services must be derivated from this class. 
//  Services can be assigned to composit nodes. Every time the composit node is executed, it will check if
//  the service shold be run. (interval expired). Therefore the intervall must be set in milli seconds.
//  A service will not be runned if the associated composit node is not executed.
//  The service will be runned, before the childs of the compositnodes are executed.
//  Services are often used to make checks and to update variables on the Blackboard.These may take the place of Parallel nodes.

#ifdef BHT_USE_SERVICES

CompNodeService::~CompNodeService() {
}

CompNodeService::CompNodeService(unsigned long _interval) {
	last_run = millis();
	setInterval(_interval);
};

void CompNodeService::runned() {
	// Saves last_run
	last_run = millis();
}

void CompNodeService::setInterval(unsigned long _interval) {
	// Save interval
	interval = _interval;
}

bool CompNodeService::shouldRun() {

	unsigned long time = millis();
	if (time - last_run >= interval) {
		return (true);
	}

	return false;
}


#endif //#ifdef BHT_USE_SERVICES

/*
CompositeNode
==========
A CompositeNode can have one or more children. The node is responsible to propagate the Blackboard signal to its children, respecting some order.
A CompositeNode node also must decide which and when to return the state values of its children, when the value is SUCCESS or FAILURE.
Notice that, when a child returns RUNNING, the composite node must return the state immediately.
*/


bool CompositeNode::onSetup(Blackboard& bb) {

	DSETUP(errorHandler.setInfoNoLog(F("%s %d\t\t CompositeNode::onSetup -> status: %s\r\n"), m_nodeName, m_nodeId, enumNodeStatusStrings[m_eNodeStatus]););
	// Check if all children were initialized
	// If we check this here, we don't need to use if(children[i] != NULL){...} in the classes, which based on CompositeNode.
	for (int i = 0; i < m_children_size; i++) {
		if (!m_children[i]) {
			errorHandler.setError(F("%s %d\t\t CompositeNode found child which is not initialized\r\n"), m_nodeName, m_nodeId);
			return false;
		}
	}

#ifdef BHT_USE_SERVICES
	// Check if all services were initialized
	for (int i = 0; i < m_service_size; i++) {
		if (!m_services[i]) {
			errorHandler.setError(F("%s %d\t\t CompositeNode found service which is not initialized\r\n"), m_nodeName, m_nodeId);
			return false;
		}
	}
#endif
	// call the onSetup of the children.
	for (int i = 0; i < m_children_size; i++) {
		if (!m_children[i]->onSetup(bb)) {
			return false;
		}
	}
	return true;
}



#ifdef BHT_USE_SERVICES
CompositeNode::CompositeNode() : m_children_size(0), m_service_size(0) {}
#else
CompositeNode::CompositeNode() : m_children_size(0) {}
#endif

CompositeNode::~CompositeNode() {
	for (int i = 0; i < m_children_size; i++) {
		if (!m_children[i]) {
			delete m_children[i];
		}
	}
	delete[] m_children;

#ifdef BHT_USE_SERVICES
	for (int i = 0; i < m_service_size; i++) {
		if (!m_services[i]) {
			delete m_services[i];
		}
	}
	delete[] m_services;
#endif
}

void CompositeNode::setupNumberOfChildren(const int& n) {

	// only 255 sub nodes allowed 
	if (n > 255) {
		errorHandler.setError(F("!03,ERROR CompositeNode size > 255 %s\r\n"), m_nodeName);
	}

	if (m_children_size == 0) {
		m_children = new Node*[n];
		m_children_size = n;
		_clearChildren();
	}
	else {
		errorHandler.setError(F("!03,ERROR CompositeNode children array size already set %s\r\n"), m_nodeName);
	}
}

void CompositeNode::_clearChildren() {
	for (int i = 0; i < m_children_size; i++) {
		m_children[i] = NULL;
	}
}


void CompositeNode::addChild(Node* child) {
	// Find an empty slot
	for (int i = 0; i < m_children_size; i++) {
		if (!m_children[i]) {
			// Found an empty slot, now add child
			m_children[i] = child;
			return;
		}
	}
	// Array is full
	errorHandler.setError(F("!03,ERROR CompositeNode array is full %s\r\n"), m_nodeName);

	return;
}


void CompositeNode::abortChild(Blackboard& bb, int i) {
	PUSH_STACK_TRY_ABORT_CHILD;
	DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t CompositeNode::abortChild -> try abort child: %s\r\n"), m_nodeName, m_nodeId, m_children[i]->m_nodeName););
	if (m_children[i]->isRunning()) {
		DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t CompositeNode::abortChild -> abort child: %s\r\n"), m_nodeName, m_nodeId, m_children[i]->m_nodeName););
		m_children[i]->abort(bb);
	}
}

void CompositeNode::abortChildren(Blackboard& bb, int i) {
	PUSH_STACK_TRY_ABORT_CHILD;
	DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t CompositeNode::abortChildren -> try abort children\r\n"), m_nodeName, m_nodeId););
	for (int j = i; j < m_children_size; j++) {
		if (m_children[j]->isRunning()) {
			DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t CompositeNode::abortChildren -> abort child: %s\r\n"), m_nodeName, m_nodeId, m_children[j]->m_nodeName););
			m_children[j]->abort(bb);
		}
	}
}

void CompositeNode::addChildren(Node* child1) {
	setupNumberOfChildren(1);
	addChild(child1);
}
void CompositeNode::addChildren(Node* child1, Node* child2) {
	setupNumberOfChildren(2);
	addChild(child1);
	addChild(child2);
}
void CompositeNode::addChildren(Node* child1, Node* child2, Node* child3) {
	setupNumberOfChildren(3);
	addChild(child1);
	addChild(child2);
	addChild(child3);
}
void CompositeNode::addChildren(Node* child1, Node* child2, Node* child3, Node* child4) {
	setupNumberOfChildren(4);
	addChild(child1);
	addChild(child2);
	addChild(child3);
	addChild(child4);
}
void CompositeNode::addChildren(Node* child1, Node* child2, Node* child3, Node* child4, Node* child5) {
	setupNumberOfChildren(5);
	addChild(child1);
	addChild(child2);
	addChild(child3);
	addChild(child4);
	addChild(child5);
}

void CompositeNode::addChildren(Node* child1, Node* child2, Node* child3, Node* child4, Node* child5, Node* child6) {
	setupNumberOfChildren(6);
	addChild(child1);
	addChild(child2);
	addChild(child3);
	addChild(child4);
	addChild(child5);
	addChild(child6);
}


void CompositeNode::addChildren(Node* child1, Node* child2, Node* child3, Node* child4, Node* child5, Node* child6, Node* child7) {
	setupNumberOfChildren(7);
	addChild(child1);
	addChild(child2);
	addChild(child3);
	addChild(child4);
	addChild(child5);
	addChild(child6);
	addChild(child7);
}

void CompositeNode::addChildren(Node* child1, Node* child2, Node* child3, Node* child4, Node* child5, Node* child6, Node* child7, Node* child8) {
	setupNumberOfChildren(8);
	addChild(child1);
	addChild(child2);
	addChild(child3);
	addChild(child4);
	addChild(child5);
	addChild(child6);
	addChild(child7);
	addChild(child8);
}

void CompositeNode::addChildren(Node* child1, Node* child2, Node* child3, Node* child4, Node* child5, Node* child6, Node* child7, Node* child8, Node* child9) {
	setupNumberOfChildren(9);
	addChild(child1);
	addChild(child2);
	addChild(child3);
	addChild(child4);
	addChild(child5);
	addChild(child6);
	addChild(child7);
	addChild(child8);
	addChild(child9);
}

void CompositeNode::addChildren(Node* child1, Node* child2, Node* child3, Node* child4, Node* child5, Node* child6, Node* child7, Node* child8, Node* child9, Node* child10) {
	setupNumberOfChildren(10);
	addChild(child1);
	addChild(child2);
	addChild(child3);
	addChild(child4);
	addChild(child5);
	addChild(child6);
	addChild(child7);
	addChild(child8);
	addChild(child9);
	addChild(child10);

}

void CompositeNode::addChildren(Node* child1, Node* child2, Node* child3, Node* child4, Node* child5, Node* child6, Node* child7, Node* child8, Node* child9, Node* child10, Node* child11) {
	setupNumberOfChildren(11);
	addChild(child1);
	addChild(child2);
	addChild(child3);
	addChild(child4);
	addChild(child5);
	addChild(child6);
	addChild(child7);
	addChild(child8);
	addChild(child9);
	addChild(child10);
	addChild(child11);

}

void CompositeNode::addChildren(Node* child1, Node* child2, Node* child3, Node* child4, Node* child5, Node* child6, Node* child7, Node* child8, Node* child9, Node* child10, Node* child11, Node* child12) {
	setupNumberOfChildren(12);
	addChild(child1);
	addChild(child2);
	addChild(child3);
	addChild(child4);
	addChild(child5);
	addChild(child6);
	addChild(child7);
	addChild(child8);
	addChild(child9);
	addChild(child10);
	addChild(child11);
	addChild(child12);

}

#ifdef BHT_USE_SERVICES

void CompositeNode::_clearServices() {
	for (int i = 0; i < m_service_size; i++) {
		m_services[i] = NULL;
	}
}

void CompositeNode::setupNumberOfServices(const int& n) {

	// only 255 sub nodes allowed 
	if (n > 255) {
		errorHandler.setError(F("!03,ERROR ServiceNode size > 255 %s\r\n"), m_nodeName);
	}

	if (m_service_size == 0) {
		m_services = new CompNodeService*[n];
		m_service_size = n;
		_clearServices();
	}
	else {
		errorHandler.setError(F("!03,ERROR CompositeNode service array size already set %s\r\n"), m_nodeName);
	}
}


void CompositeNode::addService(CompNodeService* _service) {
	// Find an empty slot
	for (int i = 0; i < m_service_size; i++) {
		if (!m_services[i]) {
			// Found an empty slot, now add child
			m_services[i] = _service;
			return;
		}
	}
	// Array is full
	errorHandler.setError(F("!03,ERROR CompositeNode service array is full %s\r\n"), m_nodeName);

	return;
}

void CompositeNode::runService(Blackboard& bb) {
	for (int i = 0; i < m_service_size; i++) {
		if (m_services[i]->shouldRun()) {
			m_services[i]->onRun(bb);
			m_services[i]->runned();
		}
	}
}

void CompositeNode::setServiceInterval(int idx, unsigned long _interval) {
	if (!m_services[idx]) {
		m_services[idx]->setInterval(_interval);
	}
	else {
		errorHandler.setError(F("!03,ERROR CompositeNode has no service configured %s idx: %d\r\n"), m_nodeName,idx);
	}

}

void CompositeNode::addServices(CompNodeService* service1) {
	setupNumberOfServices(1);
	addService(service1);
}
void CompositeNode::addServices(CompNodeService* service1, CompNodeService* service2) {
	setupNumberOfServices(2);
	addService(service1);
	addService(service2);
}
void CompositeNode::addServices(CompNodeService* service1, CompNodeService* service2, CompNodeService* service3) {
	setupNumberOfServices(3);
	addService(service1);
	addService(service2);
	addService(service3);
}
#endif //#ifdef BHT_USE_SERVICES


// ============================================================================
//The sequence node calls its children sequentially until one of them returns FAILURE or RUNNING. If all children return the success state, the sequence also returns SUCCESS.
Sequence::Sequence() : m_idxLastRunnedNode(0) {
}

NodeStatus Sequence::tick(Blackboard& bb) {
	SAVE_OLD_STATUS;

	RUNSERVICE(bb);

	if (!isRunning()) {
		DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t Sequence::tick initialize status: %s\r\n"), m_nodeName, m_nodeId, enumNodeStatusStrings[m_eNodeStatus]););
		m_idxLastRunnedNode = 0;
	}

	// run all children until one returns running or failure
	for (int i = 0; i < m_children_size; i++) {
		m_eNodeStatus = m_children[i]->tick(bb); // The status of the child is also my status

		if (m_eNodeStatus != BH_SUCCESS) {

			PUSH_STACK;

			// Switching from a low -> high priority branch causes an abort signal to be sent to the previously executing low priority branch.
			// if the current child is running or failure the sequence will break.
			// therefore check if the current node was also in state running at the previous call.
			// If yes, the last runned child is also in BH_RUNNING mode and must be aborted if it is not the same child.

			if (m_idxLastRunnedNode > i) {
				abortChild(bb, m_idxLastRunnedNode);
			}

			/*
			if ((m_children[m_idxLastRunnedNode]->isRunning()) && (m_idxLastRunnedNode != i)) {
				DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t Sequence::tick abort child: %s\r\n"), m_nodeName, m_nodeId, m_children[m_idxLastRunnedNode]->getNodeName()););
				PUSH_STACK_TRY_ABORT_CHILD;
				m_children[m_idxLastRunnedNode]->abort(bb);
			}
			*/
			m_idxLastRunnedNode = i;
			return m_eNodeStatus;
		}
	}

	PUSH_STACK;

	return BH_SUCCESS;  // All children succeeded so the entire sequence succeeds
}

void Sequence::abort(Blackboard& bb) {
	SAVE_OLD_STATUS;
	if (m_children[m_idxLastRunnedNode]->isRunning()) {
		DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t Sequence::abort -> abort child: %s\r\n"), m_nodeName, m_nodeId, m_children[m_idxLastRunnedNode]->m_nodeName););
		m_children[m_idxLastRunnedNode]->abort(bb);
	}
	m_eNodeStatus = BH_ABORTED;
	PUSH_STACK_ABORT;
}

// ============================================================================
//  A Filter is a branch in the tree that will not execute its child behavior under specific conditions.
//  For instance, if an attack has a cooldown timer to prevent it from executing
//  too often, or a behavior that is only valid at a specific distance away from a target, etc.

void Filter::setNumberOfChildren(int n) {
	CompositeNode::setupNumberOfChildren(n);
}
void Filter::addCondition(Condition* cond) {
	addChild(cond);
}
void Filter::addAction(Node* action) {
	addChild(action);
}

// ============================================================================
// MemSequence is similar to Sequence node, but when a child returns a RUNNING state, its index is recorded and in the next tick the
// MemSequence call the child recorded directly, without calling previous children again.
// Therefore it is not possible to switching from a low -> high priority 

MemSequence::MemSequence() : m_idxLastRunnedNode(0) {}


NodeStatus MemSequence::tick(Blackboard& bb) {

	SAVE_OLD_STATUS;

	RUNSERVICE(bb);

	if (!isRunning()) {
		DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t MemSequence::tick initialize status: %s\r\n"), m_nodeName, m_nodeId, enumNodeStatusStrings[m_eNodeStatus]););
		m_idxLastRunnedNode = 0;
	}

	for (int i = m_idxLastRunnedNode; i < m_children_size; i++) {
		m_eNodeStatus = m_children[i]->tick(bb);

		if (m_eNodeStatus != BH_SUCCESS) {

			PUSH_STACK;

			m_idxLastRunnedNode = i;
			return m_eNodeStatus;
		}
	}

	PUSH_STACK;

	return BH_SUCCESS;  // All children failed so the entire run() operation fails.
}

void MemSequence::abort(Blackboard& bb) {
	SAVE_OLD_STATUS;
	if (m_children[m_idxLastRunnedNode]->isRunning()) {
		DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t MemSequence::abort -> abort child: %s\r\n"), m_nodeName, m_nodeId, m_children[m_idxLastRunnedNode]->m_nodeName););
		m_children[m_idxLastRunnedNode]->abort(bb);
	}
	m_eNodeStatus = BH_ABORTED;
	PUSH_STACK_ABORT;
}

// ============================================================================
// StarSequence Use this ControlNode when you don't want to tick children again that already returned SUCCESS.
// StarSequence tries to executes all children one after another
// StarSequence will return FAILURE immediately if any child returns FAILURE. But if StarSequence is called again, the child which has returned FAILURE is called directly again.
// StarSequence will return RUNNING immediately if any child returns RUNNING. But if StarSequence is called again, the child which has returned RUNNING is called directly again.
// StarSequence will return SUCCESS if all childs returned SUCCESS
// If a child return SUCCESS, the next child will be executed (StarSequence will not return)

// Currently the StarSequence has the problem, that m_idxLastRunnedNode = 0 if the node is running and then will be aborted.
// But if it returns FAILURE and a higher priority branch is executed, then m_idxLastRunnedNode will not be set to 0.

/*
StarSequence::StarSequence() : m_idxLastRunnedNode(0), originalStatus(BH_INVALID) {}


NodeStatus StarSequence::tick(Blackboard& bb) {

	m_eNodeStatus = originalStatus;

	RUNSERVICE(bb);

	// Object exists?
	while (m_idxLastRunnedNode < m_children_size) {
		SAVE_OLD_STATUS;

		m_eNodeStatus = m_children[m_idxLastRunnedNode]->tick(bb);
		switch (m_eNodeStatus) {
		case BH_RUNNING:
		{
			PUSH_STACK;
			return BH_RUNNING;
			break;
		}
		case BH_FAILURE:
		{
			PUSH_STACK;
			originalStatus = m_eNodeStatus;
			m_eNodeStatus = BH_RUNNING;
			return BH_FAILURE;
			break;
		}
		case BH_SUCCESS:
		{
			PUSH_STACK;
			m_idxLastRunnedNode++;
			break;
		}
		default:
			errorHandler.setInfoNoLog(F("%s %d\t\t StarSequence::tick wrong status from child: %s\r\n"), m_nodeName, m_nodeId, m_children[m_idxLastRunnedNode]->getNodeName());
			break;

		}
	}
	// All the children have returned SUCCESS
	if (m_idxLastRunnedNode >= m_children_size) {
		m_idxLastRunnedNode = 0;
	}

	originalStatus = m_eNodeStatus;
	//	errorHandler.setInfoNoLog(F("%s %d\t\t Selector::tick initialize status: %s\r\n"), m_nodeName, m_nodeId, enumNodeStatusStrings[m_eNodeStatus]);)
	return BH_SUCCESS;  // Should never been reached.

}

void StarSequence::abort(Blackboard& bb) {
	SAVE_OLD_STATUS;
	if (m_children[m_idxLastRunnedNode]->isRunning()) {
		DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t MemRing::abort -> abort child: %s\r\n"), m_nodeName, m_nodeId, m_children[m_idxLastRunnedNode]->m_nodeName););
		m_children[m_idxLastRunnedNode]->abort(bb);
	}
	m_idxLastRunnedNode = 0;
	originalStatus = BH_ABORTED;
	m_eNodeStatus = BH_ABORTED;
	PUSH_STACK_ABORT;
}
*/


// ============================================================================
// The Selector node (sometimes called priority) calls its children sequentially until one of them returns SUCCESS, RUNNING. 
// If all children return the failure state, the priority also returns FAILURE.

Selector::Selector() : m_idxLastRunnedNode(0) {}

NodeStatus Selector::tick(Blackboard& bb) {
	SAVE_OLD_STATUS;

	RUNSERVICE(bb);

	if (!isRunning()) {
		DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t Selector::tick initialize status: %s\r\n"), m_nodeName, m_nodeId, enumNodeStatusStrings[m_eNodeStatus]););
		m_idxLastRunnedNode = 0;
	}

	for (int i = 0; i < m_children_size; i++) {
		m_eNodeStatus = m_children[i]->tick(bb);

		if (m_eNodeStatus != BH_FAILURE) {
			PUSH_STACK;
			// Switching from a low -> high priority branch causes an abort signal to be sent to the previously executing low priority branch.
			// if the current child is running or success the sequence will break.
			// therefore check if the current node was also in state running at the previous call.
			// If yes, the last runned child is also in BH_RUNNING mode and must be aborted if it is not the same child.

			if (m_idxLastRunnedNode > i) {
				abortChild(bb, m_idxLastRunnedNode);
			}

			/*
			if ((m_children[m_idxLastRunnedNode]->isRunning()) && (m_idxLastRunnedNode != i)) {
				DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t Selector::tick abort child: %s\r\n"), m_nodeName, m_nodeId, m_children[m_idxLastRunnedNode]->getNodeName()););
				PUSH_STACK_TRY_ABORT_CHILD;
				m_children[m_idxLastRunnedNode]->abort(bb);
			}
			*/

			m_idxLastRunnedNode = i;

			return m_eNodeStatus;
		}
	}

	PUSH_STACK;
	return BH_FAILURE;  // All children failed so the entire run() operation fails
}

void Selector::abort(Blackboard& bb) {
	SAVE_OLD_STATUS;
	if (m_children[m_idxLastRunnedNode]->isRunning()) {
		DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t Selector::abort -> abort child: %s\r\n"), m_nodeName, m_nodeId, m_children[m_idxLastRunnedNode]->m_nodeName););
		m_children[m_idxLastRunnedNode]->abort(bb);
	}
	m_eNodeStatus = BH_ABORTED;
	PUSH_STACK_ABORT;
}

// ============================================================================
// MemSelector is similar to Selector node, but when a child returns a  RUNNING state, its index is recorded and in the next tick the
// MemSelector  calls the running child directly, without calling previous children again.
// Therefore it is not possible to switching from a low -> high priority 

MemSelector::MemSelector() : m_idxLastRunnedNode(0) {}


NodeStatus MemSelector::tick(Blackboard& bb) {
	SAVE_OLD_STATUS;

	RUNSERVICE(bb);

	if (!isRunning()) {
		DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t MemSelector::tick initialize status: %s\r\n"), m_nodeName, m_nodeId, enumNodeStatusStrings[m_eNodeStatus]););
		m_idxLastRunnedNode = 0;
	}

	for (int i = m_idxLastRunnedNode; i < m_children_size; i++) {
		// Object exists?
		m_eNodeStatus = m_children[i]->tick(bb);

		if (m_eNodeStatus != BH_FAILURE) {
			PUSH_STACK;
			m_idxLastRunnedNode = i; //don't have to ask if node is running, because if not running, m_idxLastRunnedNode will be reseted to 0.
			return m_eNodeStatus;
		}
	}
	PUSH_STACK;
	return BH_FAILURE;  // All children failed so the entire run() operation fails.
}

void MemSelector::abort(Blackboard& bb) {
	SAVE_OLD_STATUS;
	if (m_children[m_idxLastRunnedNode]->isRunning()) {
		DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t MemSelector::abort -> abort child: %s\r\n"), m_nodeName, m_nodeId, m_children[m_idxLastRunnedNode]->m_nodeName););
		m_children[m_idxLastRunnedNode]->abort(bb);
	}
	m_eNodeStatus = BH_ABORTED;
	PUSH_STACK_ABORT;
}

// ============================================================================
// Parallel executes all children "parallel"
// Parallel will return FAILURE immediately if any child returns FAILURE
// Parallel will return SUCCESS immediately if any child returns SUCCESS
// Parallel will return RUNNING if all children return RUNNING

Parallel::Parallel() {}


NodeStatus Parallel::tick(Blackboard& bb) {
	SAVE_OLD_STATUS;

	RUNSERVICE(bb);

	/*if (!isRunning()) {
	DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t Parallel::tick initialize status: %s\r\n"), m_nodeName, m_nodeId, enumNodeStatusStrings[m_eNodeStatus]););

	}*/

	for (int i = 0; i < m_children_size; i++) {
		m_eNodeStatus = m_children[i]->tick(bb);
		if (m_eNodeStatus != BH_RUNNING) {
			PUSH_STACK;
			// Abort all running children
			abortChildren(bb, 0);
			/*
			PUSH_STACK_TRY_ABORT_CHILD;
			for (int i = 0; i < m_children_size; i++) {
				if (m_children[i]->isRunning()) {
					DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t Parallel::tick -> abort child: %s\r\n"), m_nodeName, m_nodeId, m_children[i]->m_nodeName););
					m_children[i]->abort(bb);
				}
			}
			*/
			return m_eNodeStatus;
		}
	}
	PUSH_STACK;
	return BH_RUNNING;  // All children failed so the entire run() operation fails.
}

void Parallel::abort(Blackboard& bb) {
	SAVE_OLD_STATUS;
	for (int i = 0; i < m_children_size; i++) {
		if (m_children[i]->isRunning()) {
			DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t Parallel::abort -> abort child: %s\r\n"), m_nodeName, m_nodeId, m_children[i]->m_nodeName););
			m_children[i]->abort(bb);
		}
	}
	m_eNodeStatus = BH_ABORTED;
	PUSH_STACK_ABORT;
}

// ============================================================================
// ParallelUntilFail executes all children "parallel"
// ParallelUntilFail will return FAILURE immediately if any child returns FAILURE
// ParallelUntilFail will return SUCCESS if all children return SUCCESS
// ParallelUntilFail will return RUNNING if at least one child returns RUNNING and the other childs return SUCCESS

ParallelUntilFail::ParallelUntilFail() {}


NodeStatus ParallelUntilFail::tick(Blackboard& bb) {
	SAVE_OLD_STATUS;

	RUNSERVICE(bb);

	/*if (!isRunning()) {
	DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t ParallelUntilFail::tick initialize status: %s\r\n"), m_nodeName, m_nodeId, enumNodeStatusStrings[m_eNodeStatus]););
	}*/

	int countRunning = 0;
	for (int i = 0; i < m_children_size; i++) {
		m_eNodeStatus = m_children[i]->tick(bb);
		if (m_eNodeStatus == BH_RUNNING) {
			countRunning++;
		}
		if (m_eNodeStatus == BH_FAILURE) {
			PUSH_STACK;
			// Abort all running children
			abortChildren(bb, 0);
			/*
			PUSH_STACK_TRY_ABORT_CHILD;
			for (int i = 0; i < m_children_size; i++) {
				if (m_children[i]->isRunning()) {
					DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t ParallelUntilFail::tick -> abort child: %s\r\n"), m_nodeName, m_nodeId, m_children[i]->m_nodeName););
					m_children[i]->abort(bb);
				}
			}
			*/
			return m_eNodeStatus;
		}
	}

	if (countRunning > 0) {
		m_eNodeStatus = BH_RUNNING;
		return BH_RUNNING;  // All children failed so the entire run() operation fails.
	}

	m_eNodeStatus = BH_SUCCESS;
	PUSH_STACK;
	return BH_SUCCESS;  // All children failed so the entire run() operation fails.
}

void ParallelUntilFail::abort(Blackboard& bb) {
	SAVE_OLD_STATUS;
	for (int i = 0; i < m_children_size; i++) {
		if (m_children[i]->isRunning()) {
			DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t ParallelUntilFail::abort -> abort child: %s\r\n"), m_nodeName, m_nodeId, m_children[i]->m_nodeName););
			m_children[i]->abort(bb);
		}
	}
	m_eNodeStatus = BH_ABORTED;
	PUSH_STACK_ABORT;
}

// ============================================================================
// Monitor
// Arguably, continuously checking if assumptions are valid (i.e., monitoring conditions)
// is the most useful pattern that involves running behaviors in parallel. Many behaviors
// tend to have assumptions that should be maintained while a behavior is active, and if
// those assumptions are found invalid the whole sub-tree should exit. Some examples of this
// include using an object (assumes the object exists) or melee attacks (assumes the enemy is
// in range), and many others.


void Monitor::setNumberOfChildren(int n) {
	CompositeNode::setupNumberOfChildren(n);
}
void Monitor::addCondition(Condition* cond) {
	addChild(cond);
}
void Monitor::addAction(Node* action) {
	addChild(action);
}


// ============================================================================
/*
Decorators
Decorators are special nodes that can have only a single child. The goal of the decorator is to change the
behavior of the child by manipulating the returning value or changing its calling frequency.
*/
DecoratorNode::DecoratorNode() : m_pChild(NULL) {}
DecoratorNode::DecoratorNode(Node* child) : m_pChild(child) {}

bool DecoratorNode::onSetup(Blackboard& bb) {

	DSETUP(errorHandler.setInfoNoLog(F("%s %d\t\t DecoratorNode::onSetup -> status: %s\r\n"), m_nodeName, m_nodeId, enumNodeStatusStrings[m_eNodeStatus]););

	if (m_pChild == NULL) {
		errorHandler.setError(F("!03,%s %d\t\t DecoratorNode m_pChild == NULL\r\n"), m_nodeName, m_nodeId);
		return false;
	}
	if (!m_pChild->onSetup(bb)) {
		return false;
	}
	return true;
}
void DecoratorNode::setChild(Node* newChild) {
	m_pChild = newChild;
}

void DecoratorNode::abort(Blackboard& bb) {
	SAVE_OLD_STATUS;
	if (m_pChild->isRunning()) {
		DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t DecoratorNode::abort -> abort child: %s\r\n"), m_nodeName, m_nodeId, m_pChild->m_nodeName););
		m_pChild->abort(bb);
	}
	m_eNodeStatus = BH_ABORTED;
	PUSH_STACK_ABORT;
}

// ============================================================================
// Check a flag, wich is given by a pointer. If it is true, the child will be called.
// If it is false, the child will be aborted if running
ExecuteOnTrue::ExecuteOnTrue() : DecoratorNode(), m_pFlag(NULL), m_setFalseOnTerminate(false) {}
ExecuteOnTrue::ExecuteOnTrue(Node* child, bool* _flag) : DecoratorNode(child), m_pFlag(_flag), m_setFalseOnTerminate(false) {}

bool ExecuteOnTrue::onSetup(Blackboard& bb) {

	DSETUP(errorHandler.setInfoNoLog(F("%s %d\t\t ExecuteOnTrue::onSetup -> status: %s\r\n"), m_nodeName, m_nodeId, enumNodeStatusStrings[m_eNodeStatus]););

	if (m_pFlag == NULL) {
		errorHandler.setError(F("!03,%s %d\t\t ExecuteOnTrue m_flag == NULL\r\n"), m_nodeName, m_nodeId);
		return false;
	}

	if (!DecoratorNode::onSetup(bb)) {
		return false;
	}
	return true;
}

void ExecuteOnTrue::setFlag(bool * _pFlag) {
	m_pFlag = _pFlag;
	m_flag_last_value = *m_pFlag;
}

void ExecuteOnTrue::setChangedStatusNodes(Blackboard& bb) {
	if (*m_pFlag != m_flag_last_value) {
		if (*m_pFlag == true) {
			bb.changedStatusNodes.Push(BH_FALSE, BH_TRUE, this);
		}
		else {
			bb.changedStatusNodes.Push(BH_TRUE, BH_FALSE, this);
		}
		m_flag_last_value = *m_pFlag;
	}
}
void ExecuteOnTrue::setChangedStatusNodesSetFlagFalse(Blackboard& bb) {
	if (*m_pFlag == true) {
		bb.changedStatusNodes.Push(BH_TRUE, BH_SET_FLAG_FALSE, this);
	}
	*m_pFlag = false;
	m_flag_last_value = *m_pFlag;
}
void ExecuteOnTrue::setFlagToFalseOnTerminate() {
	m_setFalseOnTerminate = true;
}

NodeStatus ExecuteOnTrue::tick(Blackboard& bb) {
	SAVE_OLD_STATUS;
	setChangedStatusNodes(bb);

	if (*m_pFlag == true) {
		m_eNodeStatus = m_pChild->tick(bb);

		if (!isRunning() && m_setFalseOnTerminate) {
			setChangedStatusNodesSetFlagFalse(bb);

		}

		PUSH_STACK;
		return m_eNodeStatus;
	}
	else {
		DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t ExecuteOnTrue::tick -> flag=false. Child NOT executed: %s\r\n"), m_nodeName, m_nodeId, m_pChild->m_nodeName););
	}

	m_eNodeStatus = BH_FAILURE;

	if (m_setFalseOnTerminate) {
		setChangedStatusNodesSetFlagFalse(bb);
	}

	PUSH_STACK;

	if (m_pChild->isRunning()) {
		PUSH_STACK_TRY_ABORT_CHILD;
		m_pChild->abort(bb);
	}

	return m_eNodeStatus;
}

void ExecuteOnTrue::abort(Blackboard& bb) {
	if (m_setFalseOnTerminate) {
		setChangedStatusNodesSetFlagFalse(bb);
	}
	else {
		setChangedStatusNodes(bb);
	}
	DecoratorNode::abort(bb);
}


// ============================================================================
//
//  Condition decorator
//  Works likes a conditon and action added to a Sequence
//  The condition will always be called, before the child->tick will be called
//  If condition is BH_SUCCESS the child->tick will be called.
//  If condition returns with BH_FAILURE, the node returns with BH_FAILURE. If 
//  the child is also in running satate, bort will be called before returning BH_FAILURE which aborts the child.

ConditionDeco::ConditionDeco() : DecoratorNode() {}
ConditionDeco::ConditionDeco(Node* child) : DecoratorNode(child) {}

NodeStatus ConditionDeco::tick(Blackboard& bb) {
	NodeStatus result;
	SAVE_OLD_STATUS;

	result = onCheckCondition(bb);

	if (result == BH_SUCCESS) {
		DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t ConditionDeco::tick -> cond returned BH_SUCCESS -> call child->tick %s\r\n"), m_nodeName, m_nodeId, enumNodeStatusStrings[m_eNodeStatus]););
		m_eNodeStatus = m_pChild->tick(bb);
		PUSH_STACK;
		return m_eNodeStatus;
	}

	if (m_pChild->isRunning()) { // Is child node running, then abort child
		DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t ConditionDeco::tick -> cond BH_FAILURE and child node running -> call child->abort %s\r\n"), m_nodeName, m_nodeId, enumNodeStatusStrings[m_eNodeStatus]););
		abort(bb);
#ifdef LOG_CONTROL_NODES
		oldState = m_eNodeStatus;
#endif
	}

	// Node is not running when code comes here and condition was failure

	m_eNodeStatus = BH_FAILURE;
	PUSH_STACK;
	DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t ConditionDeco::tick -> returning failure to parent: %s\r\n"), m_nodeName, m_nodeId, enumNodeStatusStrings[m_eNodeStatus]););
	return BH_FAILURE;
}


// ============================================================================
//
//  Condition decorator with memory
//  Works likes a conditon and action added to a MemSequence
//  The condition will not be called if the child is in status RUNNING. But the child->tick is called.
//  If condition returns BH_SUCCESS, the child->tick is called.
//  If condition returns BH_FAILURE, the node returns with BH_FAILURE


ConditionMemDeco::ConditionMemDeco() : DecoratorNode() {}
ConditionMemDeco::ConditionMemDeco(Node* child) : DecoratorNode(child) {}


NodeStatus ConditionMemDeco::tick(Blackboard& bb) {
	NodeStatus result;
	SAVE_OLD_STATUS;

	if (m_pChild->isRunning()) { // Is child is running, then don't query condition. Run child directly.
		DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t ConditionMemDeco::tick -> cond not called because child is running -> calling child %s\r\n"), m_nodeName, m_nodeId, enumNodeStatusStrings[m_eNodeStatus]););
		m_eNodeStatus = m_pChild->tick(bb);
		PUSH_STACK;
		return m_eNodeStatus;
	}

	DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t ConditionMemDeco::tick -> calling condition %s\r\n"), m_nodeName, m_nodeId, enumNodeStatusStrings[m_eNodeStatus]););
	result = onCheckCondition(bb);

	if (result == BH_SUCCESS) {
		DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t ConditionMemDeco::tick -> cond returned BH_SUCCESS  -> calling child %s\r\n"), m_nodeName, m_nodeId, enumNodeStatusStrings[m_eNodeStatus]););
		m_eNodeStatus = m_pChild->tick(bb);
		PUSH_STACK;
		return m_eNodeStatus;
	}

	// Node is not running, therfore I don't need to abort here.
	m_eNodeStatus = BH_FAILURE;
	PUSH_STACK;
	DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t ConditionMemDeco::tick -> cond returned BH_FAILURE. Returning failure to parent %s\r\n"), m_nodeName, m_nodeId, enumNodeStatusStrings[m_eNodeStatus]););
	return BH_FAILURE;
}


// ============================================================================
// Like the NOT operator, the inverter decorator negates the result of its child node, i.e., SUCCESS state becomes FAILURE, and FAILURE becomes SUCCESS.
// Notice that, inverter does not change RUNNING or ERROR states, as described in algorithm below.
Inverter::Inverter() : DecoratorNode() {}
Inverter::Inverter(Node* child) : DecoratorNode(child) {}

NodeStatus Inverter::tick(Blackboard& bb) {
	SAVE_OLD_STATUS;

	m_eNodeStatus = m_pChild->tick(bb);
	if (m_eNodeStatus == BH_FAILURE) {
		m_eNodeStatus = BH_SUCCESS;
	}
	else if (m_eNodeStatus == BH_SUCCESS) {
		m_eNodeStatus = BH_FAILURE;
	}
	PUSH_STACK;
	return m_eNodeStatus;
}


// ============================================================================
// A succeeder will always return success, irrespective of what the child node actually returned.
// These are useful in cases where you want to process a branch of a tree where a failure is expected or anticipated, but you donâ€™t want to abandon processing of a sequence that branch sits on.
Succeeder::Succeeder() : DecoratorNode() {}
Succeeder::Succeeder(Node* child) : DecoratorNode(child) {}

NodeStatus Succeeder::tick(Blackboard& bb) {
	SAVE_OLD_STATUS;
	m_eNodeStatus = m_pChild->tick(bb);
	if (m_eNodeStatus == BH_FAILURE) {
		m_eNodeStatus = BH_SUCCESS;
	}
	else if (m_eNodeStatus == BH_SUCCESS) {
		m_eNodeStatus = BH_SUCCESS;
	}
	PUSH_STACK;
	return m_eNodeStatus;
}


// ============================================================================
// The opposite of a Succeeder, always returning fail.  Note that this can be achieved also by using an Inverter and setting its child to a Succeeder.
Failer::Failer() : DecoratorNode() {}
Failer::Failer(Node* child) : DecoratorNode(child) {}

NodeStatus Failer::tick(Blackboard& bb) {
	SAVE_OLD_STATUS;
	m_eNodeStatus = m_pChild->tick(bb);
	if (m_eNodeStatus == BH_FAILURE) {
		m_eNodeStatus = BH_FAILURE;
	}
	else if (m_eNodeStatus == BH_SUCCESS) {
		m_eNodeStatus = BH_FAILURE;
	}
	PUSH_STACK;
	return m_eNodeStatus;
}


// ============================================================================
// A repeater will reprocess its child node each time its child returns a result. These are often used at the very base of the tree,
// to make the tree to run continuously. Repeaters may optionally run their children a set number of times before returning to their parent.
//

Repeat::Repeat() : DecoratorNode() {}
Repeat::Repeat(Node* child) : DecoratorNode(child) {}

void Repeat::setCount(int count) {
	m_iLimit = count;
}


NodeStatus Repeat::tick(Blackboard& bb) {
	SAVE_OLD_STATUS;

	if (!isRunning()) {
		DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t Repeat::tick initialize status: %s\r\n"), m_nodeName, m_nodeId, enumNodeStatusStrings[m_eNodeStatus]););
		m_iCounter = 0;
	}

	for (;;) {
		m_eNodeStatus = m_pChild->tick(bb);
		if (m_eNodeStatus == BH_RUNNING) {
			PUSH_STACK;
			break;
		}
		if (m_eNodeStatus == BH_FAILURE) {
			PUSH_STACK;
			return BH_FAILURE;
		}
		if (++m_iCounter == m_iLimit) {
			m_eNodeStatus = BH_SUCCESS;
			PUSH_STACK;
			return BH_SUCCESS;
		}
		m_pChild->reset();
	}
	return BH_RUNNING;
}


// ============================================================================
// Like a repeater, these decorators will continue to reprocess their child. That is until the child finally returns a failure, at which point the repeater will return success to its parent.

NodeStatus RepeatUntilFail::tick(Blackboard& bb) {
	do {
		m_eNodeStatus = m_pChild->tick(bb);
	} while (!(BH_FAILURE == m_eNodeStatus));

	return BH_SUCCESS;
}


// ============================================================================
// WaitDecorator node. Waits the configured amount of time until the child will be executed.
// As long as the child returns BH_RUNNING, the status waittimeExpired will be latched

WaitDecorator::WaitDecorator() : m_ulWaitMillis(0), m_ulStartTime(0), m_waittimeExpired(false) {}


NodeStatus WaitDecorator::tick(Blackboard& bb) {

	if (!isRunning()) {
		DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t WaitDecorator::tick initialize status: %s\r\n"), m_nodeName, m_nodeId, enumNodeStatusStrings[m_eNodeStatus]););
		m_waittimeExpired = false;
		m_ulStartTime = millis();
		m_eNodeStatus = BH_RUNNING;
		bb.changedStatusNodes.Push(BH_EMPTY, BH_WAITING, this);
	}


	if ((millis() - m_ulStartTime > m_ulWaitMillis) && (m_waittimeExpired == false)) {
		m_waittimeExpired = true;
		bb.changedStatusNodes.Push(BH_WAITING, BH_WAITTIME_EXPIRED, this);
		//debug->printf("millis(): %lu  m_ulStartTime: %lu\r\n",millis(), m_ulStartTime);
	}

	if (m_waittimeExpired) {
		m_eNodeStatus = m_pChild->tick(bb);
		return m_eNodeStatus;
	}

	return m_eNodeStatus;
}

void WaitDecorator::setWaitMillis(uint32_t millis) {
	m_ulWaitMillis = millis;
}


void WaitDecorator::abort(Blackboard& bb) {
	SAVE_OLD_STATUS;
	if (m_pChild->isRunning()) {
		DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t WaitDecorator::abort -> abort child: %s\r\n"), m_nodeName, m_nodeId, m_pChild->m_nodeName););
		m_pChild->abort(bb);

		if (m_waittimeExpired) {
			bb.changedStatusNodes.Push(BH_WAITTIME_EXPIRED, BH_ABORTED, this);
		}
		else {
			bb.changedStatusNodes.Push(BH_WAITING, BH_ABORTED, this);
		}

	}
	m_eNodeStatus = BH_ABORTED;
	PUSH_STACK_ABORT;
}



// ============================================================================
// WaitDecorator node. Waits the configured amount of time until the child will be executed.
// The time is set in a variable at the black board
// As long as the child returns BH_RUNNING, the status waittimeExpired will be latched

WaitBBTimeDecorator::WaitBBTimeDecorator() : m_ulpWaitMillis(NULL), m_ulStartTime(0), m_waittimeExpired(false) {}

bool WaitBBTimeDecorator::onSetup(Blackboard& bb) {

	DSETUP(errorHandler.setInfoNoLog(F("%s %d\t\t WaitBBTimeDecorator::onSetup -> status: %s\r\n"), m_nodeName, m_nodeId, enumNodeStatusStrings[m_eNodeStatus]););

	if (m_ulpWaitMillis == NULL) {
		errorHandler.setError(F("!03,%s %d\t\t WaitBBTimeDecorator m_ulpWaitMillis == NULL\r\n"), m_nodeName, m_nodeId);
		return false;
	}

	if (!DecoratorNode::onSetup(bb)) {
		return false;
	}
	return true;
}

NodeStatus WaitBBTimeDecorator::tick(Blackboard& bb) {

	if (!isRunning()) {
		DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t WaitBBTimeDecorator::tick initialize status: %s\r\n"), m_nodeName, m_nodeId, enumNodeStatusStrings[m_eNodeStatus]););
		m_waittimeExpired = false;
		m_ulStartTime = millis();
		m_eNodeStatus = BH_RUNNING;
		bb.changedStatusNodes.Push(BH_WAITING, BH_WAITING, this);
	}


	if ((millis() - m_ulStartTime > (*m_ulpWaitMillis)) && (m_waittimeExpired == false)) {
		m_waittimeExpired = true;
		bb.changedStatusNodes.Push(BH_WAITING, BH_WAITTIME_EXPIRED, this);
		//debug->printf("millis(): %lu  m_ulStartTime: %lu\r\n",millis(), m_ulStartTime);
	}

	if (m_waittimeExpired) {
		m_eNodeStatus = m_pChild->tick(bb);
	}

	return m_eNodeStatus;
}

void WaitBBTimeDecorator::setWaitBBPointer(uint32_t* pWaitTimePointer) {
	m_ulpWaitMillis = pWaitTimePointer;
}


void WaitBBTimeDecorator::abort(Blackboard& bb) {
	SAVE_OLD_STATUS;
	if (m_pChild->isRunning()) {
		DTREE(errorHandler.setInfoNoLog(F("%s %d\t\t WaitBBTimeDecorator::abort -> abort child: %s\r\n"), m_nodeName, m_nodeId, m_pChild->m_nodeName););
		m_pChild->abort(bb);

		if (m_waittimeExpired) {
			bb.changedStatusNodes.Push(BH_WAITTIME_EXPIRED, BH_ABORTED, this);
		}
		else {
			bb.changedStatusNodes.Push(BH_WAITING, BH_ABORTED, this);
		}

	}
	m_eNodeStatus = BH_ABORTED;
	PUSH_STACK_ABORT;
}
// ============================================================================
//  BehaviourTree
//  In this class the root node must be set

BehaviourTree::BehaviourTree() :root(NULL), flagLogChangedNode(true) {}

void BehaviourTree::setRootNode(Node* newChild) {
	root = newChild;
}
bool BehaviourTree::onSetup(Blackboard& bb) {
	m_nodeName = (char*)"tree";

	if (root == NULL) {
		errorHandler.setInfoNoLog(F("**TREE root is not initialized**\r\n"));
		return false;
	}
	DSETUP(errorHandler.setInfoNoLog(F("%s %d\t\t BehaviourTree::onSetup -> status: %s\r\n"), m_nodeName, m_nodeId, enumNodeStatusStrings[m_eNodeStatus]););
	return root->onSetup(bb);
}

NodeStatus BehaviourTree::tick(Blackboard& bb) {

	DTREE(errorHandler.setInfoNoLog(F("** TREE BEGIN **\r\n")););

	bb.changedStatusNodes.Clear();

	// run the tree
	m_eNodeStatus = root->tick(bb);

	if (flagLogChangedNode) {
		groupIdx++;
		/*
		if (groupIdx > 9) {
			groupIdx = 0;
		}
		*/
		int size = bb.changedStatusNodes.GetSize();
		//errorHandler.setInfoNoLog(F("size: %d\r\n"), size);

		for (int i = 0; i < size; i++) {
			NodeStackItem n = bb.changedStatusNodes.Get(i);
			if (n.node != NULL) {
				sprintf(errorHandler.msg, "%3d %-20s\t%s -> %s\r\n", groupIdx, n.node->m_nodeName, enumNodeStatusStrings[n.previousStatus], enumNodeStatusStrings[n.newStatus]);
				if (bb.flagBHTShowChanges) {
					errorHandler.setInfo();
				}
				else {
					errorHandler.writeToLogOnly();
				}
			}
			else {
				errorHandler.setInfoNoLog(F("******************\r\n"));
				errorHandler.setInfoNoLog(F("** NODE IS NULL **\r\n"));
				errorHandler.setInfoNoLog(F("******************\r\n"));
			}

		}

	}
	return m_eNodeStatus;
}

void BehaviourTree::abort(Blackboard& myBlackboard) {
	if (root->isRunning()) {
		root->abort(myBlackboard);
	}
	m_eNodeStatus = BH_ABORTED;
}

void BehaviourTree::reset(Blackboard& bb) {

	flagLogChangedNode = true;
	groupIdx = 0;

}



