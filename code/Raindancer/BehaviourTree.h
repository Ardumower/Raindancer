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




#ifndef BEHAVIOURTREE_H
#define BEHAVIOURTREE_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "Blackboard.h"

// Comment out,  if you don't want to use services
// Services are often used to make checks and to update the Blackboard.
// These take the place of traditional Parallel nodes in other Behavior Tree systems.
// Because this  Behavior Tree systems supports also parallel nodes (also Monitor), you can decide.
#define BHT_USE_SERVICES 1

enum NodeStatus : uint8_t {
	BH_FAILURE = 0,	// Return value from Action/Condition Node
	BH_SUCCESS = 1,	// Return value from Action/Condition Node
	BH_RUNNING = 2,	// Return value from Action/Condition Node
	BH_ABORTED = 3,	// Abortfunction was called
	BH_INVALID = 4,	// Node just created or reseted 
	// Following states are not used by the behavior tree. They are only used to send more information to changedStatusNodes,
	// wich will show the changed nodes at the end of the tick of the tree
	BH_TRUE = 5,	// Will ony be used by ExecuteOnTrue, ConditionDeco, ConditionMemDeco to log if flag is true or false
	BH_FALSE = 6,	// Will ony be used by ExecuteOnTrue, ConditionDeco, ConditionMemDeco to log if flag is true or false
	BH_SET_FLAG_FALSE = 7,	// Will ony be used by ExecuteOnTrue to show in changedStatusNodes if flag is true or false
	BH_WAITING = 8,	// Will only be used by WaitDecorator to show in changedStatusNodes  that the decorator is waiting
	BH_WAITTIME_EXPIRED = 9, // Will only be used by WaitDecorator to show in changedStatusNodes  that the waittime has expired
	BH_TRY_TO_ABORT_CHILD = 10, // Will be used to give info in changedStatusNodes when the contolnodes tries to abort the childs
	BH_EMPTY = 11  // Write empty string
};

/*
enum NodeAdditionalInfo : uint8_t {
	// Following states are not used by the behavior tree. They are only used to send more information to changedStatusNodes,
	// wich will show the changed nodes at the end of the tick of the tree
	aBH_FALSE = 0,	// Will ony be used by ExecuteOnTrue to show in changedStatusNodes if flag is true or false
	aBH_TRUE = 1,	// Will ony be used by ExecuteOnTrue to show in changedStatusNodes if flag is true or false
	aBH_SET_FLAG_FALSE = 2,	// Will ony be used by ExecuteOnTrue to show in changedStatusNodes if flag is true or false
	aBH_WAITING = 3,	// Will only be used by WaitDecorator to show in changedStatusNodes  that the decorator is waiting
	aBH_WAITTIME_EXPIRED = 4, // Will only be used by WaitDecorator to show in changedStatusNodes  that the waittime has expired
	aBH_TRY_TO_ABORT_CHILD = 5, // Will be used to give info in changedStatusNodes when the contolnodes tries to abort the childs
	aBH_EMPTY = 6
};
*/

extern const char* enumNodeStatusStrings[];
//extern const char* enumNodeAddInfoStrings[];
// ============================================================================
/**
* Base class for actions, conditions, CompositeNode and behavior tree.
*/

class Node {
public:
	int16_t m_nodeId;
	char* m_nodeName;
	NodeStatus m_eNodeStatus;

	Node();
	virtual ~Node();

	// Overwrite this function with your code for setting up a node or check something before the BHT ticks,
	// if you are not able to do this in the constructor.
	// Should called once when the programm starts in the main function.
	// Return false if something goes wrong. Then the code should be halted in the main function.
	// If you use the Action node in more than one subtrees, the onSetup is called for every subtree. In this case
	// onSetup will be called more than once. If you want to prevent this, you have to use a flag in onSetup, that the
	// function was already called. onSetup must return true, if all is OK.
	virtual bool onSetup(Blackboard& bb);

	// Every class derivated from Node must have implemented these functions
	virtual NodeStatus tick(Blackboard& bb) = 0;
	virtual void abort(Blackboard& bb) = 0;

	void reset();
	bool isTerminated() const;
	inline bool isRunning() const {
		return m_eNodeStatus == BH_RUNNING;
	}
	bool isInvalid() const;
	NodeStatus getNodeStatus() const;
	int16_t getNodeID() const;
	char* getNodeName() const;
	void setNodeName(char*);
};

// ============================================================================
/**
*  Action classes must be derivated from Action
*  ActionNodes are leaves and do not have any children. The user should implement their own ActionNodes to perform the actual tasks.
*  Here the function onUpdate must be overwritten. onSetup, onUpdate and onTerminate can be overwritten
*/
class Action : public Node {
public:

	// Overwrite this function with your code. Will be called every time the node has another current state than BH_RUNNING
	virtual void onInitialize(Blackboard& bb);
	// Overwrite this function with your code. This is the maincode of the node. Must return BH_RUNNING, BH_SUCCESS or BH_FAILIURE.
	virtual NodeStatus onUpdate(Blackboard& bb) = 0;
	// Overwrite this function with your code. Will be called every time onUpdate returns another state than BH_RUNNING
	// status tells the onTerminate if abort is calling (status=BH_ABORT) or the returnvalue of onUpdate (status=BH_SUCCESS or status==BH_FAILURE). 
	virtual void onTerminate(NodeStatus status, Blackboard& bb);

	Action();
	virtual ~Action();

	// This function will be called to run a leaf. It will then call onInitialize, onUpdate, onTerminate
	virtual NodeStatus tick(Blackboard& bb) override;

	virtual void abort(Blackboard& bb) override; // will be called from BehaviorTree classes to abort a node

	unsigned long getTimeInNode();
	void setTimeInNode(unsigned long t);


private:
	unsigned long _start_time_of_node;
};

// ============================================================================
/**
*  Action class with condition
*  Works likes a conditon and action added to a Sequence
*  The condition will always be called, before the Action will be done
*  If condition is BH_SUCCESS the Action node will be called.
*  If condition returns with BH_FAILURE, the node returns with BH_FAILURE. If the node is also in running atate, abort will be called before returning BH_FAILURE.
*/
class ActionCond : public Action {
public:
	virtual NodeStatus onCheckCondition(Blackboard& bb) = 0;
	virtual NodeStatus tick(Blackboard& bb) override;

};

// ============================================================================
/**
*  Action class with memory condition
*  Works likes a conditon and action added to a MemSequence
*  The condition will not be called if the node is in status RUNNING
*  If condition returns BH_FAILURE, the node returns with FAIURE
*/
class ActionMemCond : public Action {
public:
	virtual NodeStatus onCheckCondition(Blackboard& bb) = 0;
	virtual NodeStatus tick(Blackboard& bb) override;

};

// ============================================================================
/**
*   Condition classes must be derivated from Condition
*   ConditionNodes are equivalent to ActionNodes, but they are always atomic and synchronous, i.e. they must not return RUNNING. They should not alter the state of the system.
*   The condition node has only onCheckCondition function. It should only query things and never change things!
*   Must return BH_SUCCESS or BH FAILIURE
*   Never change variables outside of the Condition. This can lead to strange behaviour of the tree. Because a condition is called all the time
*   and then could set/reset values which needed in other nodes which will also use this variables.
*/
class Condition : public Node {
public:

	Condition();
	virtual ~Condition();

	virtual NodeStatus onCheckCondition(Blackboard& bb) = 0;

	virtual NodeStatus tick(Blackboard& bb) override;

	virtual void abort(Blackboard& bb) override; // will be called from BehaviorTree classes to abort a node

};
// ============================================================================
/**
*  Service Class
*  Services must be derivated from this class.
*  A services must be assigned to a composit node. Every time the composit node is executed, it will check if
*  the service should be run (interval expired). Therefore the intervall must be set in milli seconds.
*  A service will not be runned if the associated composit node is not executed.
*  The service will be runned, before the childs of the compositnodes are executed.
*  Services are often used to make checks and to update variables on the Blackboard. These may take the place of Parallel nodes.
*/

#ifdef BHT_USE_SERVICES
class CompNodeService {
	protected:
		// Desired interval between runs
		unsigned long interval;
		// Last runned time in Ms
		unsigned long last_run;
	public:
		CompNodeService(unsigned long _interval = 0);
		virtual ~CompNodeService();

		/*
		IMPORTANT! Run runned() after all calls to onRun()
		Updates last_run
		NOTE: This MUST be called if extending
		this class and implementing run() method
		MORE IMPORTANT:
		Will in comopsit nodes automatically be called.
		There is no need to call this in the classes derivited
		from CompNodeService.
		*/
		// Default is to mark it runned "now"
		void runned(); 
		// Set the desired interval for calls, and update _cached_next_run
		virtual void setInterval(unsigned long _interval);
		// Default is to check whether it should run "now"
		bool shouldRun();
		// Run thread
		virtual void onRun(Blackboard& bb) = 0;
	};
#endif

// ============================================================================
/**
*  CompositeNode
*  CompositeNode are nodes which can have 1 to N children. Once a tick is received, this tick may be propagated to one or more of the children.
*  A CompositeNode can have one or more children. The node is responsible to propagate the Blackboard signal to its children, respecting some order.
*  A CompositeNode node also must decide which and when to return the state values of its children.
*/
class CompositeNode : public Node {  //  This type of Node follows the Composite Pattern, containing a list of other Nodes.

private:

protected:
	int m_children_size;
	Node** m_children;

#ifdef BHT_USE_SERVICES
	int m_service_size;
	CompNodeService** m_services;
#endif

	// before calling addChild, setupNumberOfChildren must be called with the number of children 
	void setupNumberOfChildren(const int& n);
	void _clearChildren();
	void addChild(Node* child);
	
	void abortChild(Blackboard& bb, int i);
	void abortChildren(Blackboard& bb, int i);

#ifdef BHT_USE_SERVICES
	void setupNumberOfServices(const int& n);
	void _clearServices();
	void addService(CompNodeService* _pService);

	void runService(Blackboard& bb);
#endif


public:

	CompositeNode();
	virtual ~CompositeNode();

	bool onSetup(Blackboard& bb);

	void setServiceInterval(int idx, unsigned long _interval);

	// addChildren will call setupNumberOfChildren before insert the children
	void addChildren(Node* child1);
	void addChildren(Node* child1, Node* child2);
	void addChildren(Node* child1, Node* child2, Node* child3);
	void addChildren(Node* child1, Node* child2, Node* child3, Node* child4);
	void addChildren(Node* child1, Node* child2, Node* child3, Node* child4, Node* child5);
	void addChildren(Node* child1, Node* child2, Node* child3, Node* child4, Node* child5, Node* child6);
	void addChildren(Node* child1, Node* child2, Node* child3, Node* child4, Node* child5, Node* child6, Node* child7);
	void addChildren(Node* child1, Node* child2, Node* child3, Node* child4, Node* child5, Node* child6, Node* child7, Node* child8);
	void addChildren(Node* child1, Node* child2, Node* child3, Node* child4, Node* child5, Node* child6, Node* child7, Node* child8, Node* child9);
	void addChildren(Node* child1, Node* child2, Node* child3, Node* child4, Node* child5, Node* child6, Node* child7, Node* child8, Node* child9, Node* child10);
	void addChildren(Node* child1, Node* child2, Node* child3, Node* child4, Node* child5, Node* child6, Node* child7, Node* child8, Node* child9, Node* child10, Node* child11);
	void addChildren(Node* child1, Node* child2, Node* child3, Node* child4, Node* child5, Node* child6, Node* child7, Node* child8, Node* child9, Node* child10, Node* child11, Node* child12);

#ifdef BHT_USE_SERVICES
	void addServices(CompNodeService* service1);
	void addServices(CompNodeService* service1, CompNodeService* service2);
	void addServices(CompNodeService* service1, CompNodeService* service2, CompNodeService* service3);
#endif
};

// ============================================================================
/**
*  The sequence node calls its children sequentially until one of them returns FAILURE or RUNNING. If all children return the success state, the sequence also returns SUCCESS.
*/
class Sequence : public CompositeNode {
	uint8_t m_idxLastRunnedNode;
public:
	Sequence();
	virtual NodeStatus tick(Blackboard& bb) override;
	virtual void abort(Blackboard& bb) override;
};

// ============================================================================
/**
*  A filter is a branch in the tree that will not execute its child behavior under specific conditions.
*  For instance, if an attack has a cooldown timer to prevent it from executing
*  too often, or a behavior that is only valid at a specific distance away from a target, etc.
*/
class Filter : public Sequence {
public:
	// For inserting one can also use addChildren(...) of the composit class
	// then the following functions are not needed
	void setNumberOfChildren(int n);
	void addCondition(Condition* cond);
	void addAction(Node* action);
};

// ============================================================================
/**
*  MemSequence is similar to Sequence node, but when a child returns a RUNNING state, its index is recorded and in the next tick the
*  MemSequence call the child recorded directly, without calling previous children again.
*/
class MemSequence : public CompositeNode {
	uint8_t m_idxLastRunnedNode;
public:
	MemSequence();
	virtual NodeStatus tick(Blackboard& bb) override;
	virtual void abort(Blackboard& bb) override;
};

// ============================================================================

/** StarSequence Use this ControlNode when you don't want to tick children again that already returned SUCCESS.
* StarSequence tries to executes all children one after another
* StarSequence will return FAILURE immediately if any child returns FAILURE. But if StarSequence is called again, the child which has returned FAILURE is called directly again.
* StarSequence will return RUNNING immediately if any child returns RUNNING. But if StarSequence is called again, the child which has returned RUNNING is called directly again.
* StarSequence will return SUCCESS if all childs returned SUCCESS
* If a child return SUCCESS, the next child will be executed (StarSequence will not return)
*
* Currently the StarSequence has the problem, that m_idxLastRunnedNode=0 if the node is running and then will be aborted.
* But if it returns FAILURE and a higher priority branch is executed, then m_idxLastRunnedNode will not be set to 0.
*
*/
/*
class StarSequence : public CompositeNode {
	int m_idxLastRunnedNode;
	NodeStatus originalStatus;
public:
	StarSequence();
	virtual NodeStatus tick(Blackboard& bb) override;
	virtual void abort(Blackboard& bb) override;
};
*/
// ============================================================================
/**
*  The Selector node (sometimes called priority) Blackboards its children sequentially until one of them returns SUCCESS, RUNNING. If all children return the failure state, the priority also returns FAILURE.
*/
class Selector : public CompositeNode {
	uint8_t m_idxLastRunnedNode;
public:
	Selector();
	virtual NodeStatus tick(Blackboard& bb) override;
	virtual void abort(Blackboard& bb) override;
};

// ============================================================================

/**
*  MemSelector is similar to Selector node, but when a child returns a  RUNNING state, its index is recorded and in the next Blackboard the,
*  MemSelector  calls the child recorded directly, without calling previous children again.
*/
class MemSelector : public CompositeNode {
	uint8_t m_idxLastRunnedNode;
public:
	MemSelector();
	virtual NodeStatus tick(Blackboard& bb) override;
	virtual void abort(Blackboard& bb) override;
};

// ============================================================================
/**
* Parallel executes all children
* Parallel will return FAILURE immediately if any child returns FAILURE
* Parallel will return SUCCESS immediately if any child returns SUCCESS
* Parallel will return RUNNING if all children return RUNNING
*/
class Parallel : public CompositeNode {

public:
	Parallel();
	virtual NodeStatus tick(Blackboard& bb) override;
	virtual void abort(Blackboard& bb) override;
};


// ============================================================================
/**
* ParallelUntilFail executes the children parallel
* ParallelUntilFail will return FAILURE immediately if any child returns FAILURE
* ParallelUntilFail will return SUCCESS if all children return SUCCESS
* ParallelUntilFail will return RUNNING if at least one child returns RUNNING and the other childs return SUCCESS
*/
class ParallelUntilFail : public CompositeNode {

public:
	ParallelUntilFail();
	virtual NodeStatus tick(Blackboard& bb) override;
	virtual void abort(Blackboard& bb) override;
};


// ============================================================================
/**
* Monitor
* Arguably, continuously checking if assumptions are valid (i.e., monitoring conditions)
* is the most useful pattern that involves running behaviors in parallel. Many behaviors
* tend to have assumptions that should be maintained while a behavior is active, and if
* those assumptions are found invalid the whole sub-tree should exit. Some examples of this
* include using an object (assumes the object exists) or melee attacks (assumes the enemy is
* in range), and many others.
*/
class Monitor : public ParallelUntilFail {
public:
	// For inserting one can also use addChildren(...) of the composit class
	// then the following functions are not needed
	void setNumberOfChildren(int n);
	void addCondition(Condition* cond);
	void addAction(Node* action);
};
// ============================================================================
/**
* Decorators
* Decorators are special nodes that can have only a single child. The goal of the decorator is to change the
* behavior of the child by manipulating the returning value or changing its Blackboarding frequency.
*/
class DecoratorNode : public Node    // Function is either to transform the result it receives from its child node's status, to terminate the child, or repeat processing of the child, depending on the type of decorator node.
{
protected:
	Node* m_pChild;  // Only one child allowed
public:
	DecoratorNode();
	DecoratorNode(Node* child);
	bool onSetup(Blackboard& bb);
	void setChild(Node* newChild);
	virtual void abort(Blackboard& myBlackboard) override;
};

// ============================================================================
/**
*  Check a Blackboard flag, wich is given as pointer. If it is true, the child node will be executed.
*  If it is false, the node will be aborted if running
*  Use setFlagToFalseOnTerminate() if the flag should be set false on termiating (BH_SUCCESS, BH_FALSE)
*/
class ExecuteOnTrue : public DecoratorNode {
private:
	bool* m_pFlag;
	bool  m_flag_last_value;
	bool m_setFalseOnTerminate;
	void setChangedStatusNodes(Blackboard& bb);
	void setChangedStatusNodesSetFlagFalse(Blackboard& bb);
public:
	ExecuteOnTrue();
	ExecuteOnTrue(Node* child, bool* _pFlag);
	bool onSetup(Blackboard& bb) override;
	NodeStatus tick(Blackboard& bb) override;
	void abort(Blackboard& bb) override;
	void setFlag(bool * _flag);
	void setFlagToFalseOnTerminate();

};

// ============================================================================
/**
*  Condition decorator
*  Works likes a conditon and action added to a Sequence
*  The condition will always be called, before the child->tick will be called
*  If condition is BH_SUCCESS the child->tick will be called.
*  If condition returns with BH_FAILURE, the node returns with BH_FAILURE.  If
*  the child is also in running satate, bort will be called before returning BH_FAILURE which aborts the child.
*/

class ConditionDeco : public DecoratorNode {
private:
	NodeStatus  m_result;
public:
	ConditionDeco();
	ConditionDeco(Node* child);
	virtual NodeStatus onCheckCondition(Blackboard& bb) = 0;
	NodeStatus tick(Blackboard& bb) override;
};


// ============================================================================
/**
*  Condition decorator with memory
*  Works likes a conditon and action added to a MemSequence
*  The condition will not be called if the child is in status RUNNING. But the child->tick is called.
*  If condition returns BH_FSUCCESS, the child->tick is called.
*  If condition returns BH_FAILURE, the node returns with BH_FAILURE
*/

class ConditionMemDeco : public DecoratorNode {
private:
	NodeStatus  m_result;
public:
	ConditionMemDeco();
	ConditionMemDeco(Node* child);
	virtual NodeStatus onCheckCondition(Blackboard& bb) = 0;
	NodeStatus tick(Blackboard& bb) override;
};

// ============================================================================
/**
*   Like the NOT operator, the inverter decorator negates the result of its child node, i.e., SUCCESS state becomes FAILURE, and FAILURE becomes SUCCESS.
*   Notice that, inverter does not change RUNNING or ERROR states, as described in algorithm below.
*/
class Inverter : public DecoratorNode {
public:
	Inverter();
	Inverter(Node* child);
	virtual NodeStatus tick(Blackboard& bb) override;
};

// ============================================================================
/**
* A succeeder will always return success, irrespective of what the child node actually returned.
* These are useful in cases where you want to process a branch of a tree where a failure is expected or anticipated, but you donâ€™t want to abandon processing of a sequence that branch sits on.
*/
class Succeeder : public DecoratorNode {
public:
	Succeeder();
	Succeeder(Node* child);
	virtual NodeStatus tick(Blackboard& bb) override;
};

// ============================================================================
/**
* The opposite of a Succeeder, always returning fail.  Note that this can be achieved also by using an Inverter and setting its child to a Succeeder.
*/
class Failer : public DecoratorNode {
public:
	Failer();
	Failer(Node* child);
	virtual NodeStatus tick(Blackboard& bb) override;
};


// ============================================================================
/**
* A repeater will reprocess its child node each time its child returns a result. These are often used at the very base of the tree,
* to make the tree to run continuously. Repeaters may optionally run their children a set number of times before returning to their parent.
*/
class Repeat : public DecoratorNode {
public:
	Repeat();
	Repeat(Node* child);

	void setCount(int count);
	virtual NodeStatus tick(Blackboard& bb) override;

protected:
	int m_iLimit;
	int m_iCounter;
};


// ============================================================================
/**
* Like a repeater, these decorators will continue to reprocess their child. That is until the child finally returns a failure, at which point the repeater will return success to its parent.
*/
class RepeatUntilFail : public DecoratorNode {
public:
	virtual NodeStatus tick(Blackboard& bb) override;
};

// ============================================================================
/**
*  WaitDecorator node. Waits the configured amount of time until the child will be executed.
*  As long as the child returns BH_RUNNING, the status waittimeExpired will be latched
*/
class WaitDecorator : public DecoratorNode //Wait a few milli seconds. Notice that, in this node, we need to define a parameter in the initialization!
{
private:
	uint32_t m_ulWaitMillis;
	uint32_t m_ulStartTime;
	bool     m_waittimeExpired;

public:
	WaitDecorator();
	virtual NodeStatus tick(Blackboard& bb) override;
	void setWaitMillis(uint32_t millis);
	virtual void abort(Blackboard& myBlackboard) override;
};


// ============================================================================
/**
*  WaitDecorator node. Waits the configured amount of time until the child will be executed.
*  The time is set in a variable at the black board
*  As long as the child returns BH_RUNNING, the status waittimeExpired will be latched
*/
class WaitBBTimeDecorator : public DecoratorNode //Wait a few milli seconds. Notice that, in this node, we need to define a parameter in the initialization!
{
private:
	uint32_t* m_ulpWaitMillis;
	uint32_t m_ulStartTime;
	bool     m_waittimeExpired;

public:
	WaitBBTimeDecorator();
	bool onSetup(Blackboard& bb) override;
	virtual NodeStatus tick(Blackboard& bb) override;
	void setWaitBBPointer(uint32_t* pWaitTimePointer);
	virtual void abort(Blackboard& myBlackboard) override;
};

// ============================================================================
/**
*  BehaviourTree
*  In this class the root node must be set
*/

class BehaviourTree : public Node {
private:
	Node* root;
	
public:
	bool flagLogChangedNode; // is only used to deactivate the showing of state changes of the nodes at the terminal 
	BehaviourTree();
	void setRootNode(Node* newChild);
	virtual NodeStatus tick(Blackboard& myBlackboard) override;
	virtual void abort(Blackboard& myBlackboard) override;
	void reset(Blackboard& myBlackboard);
	bool onSetup(Blackboard& bb) override;
};

#endif

