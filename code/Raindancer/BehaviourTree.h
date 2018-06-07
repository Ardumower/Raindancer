/*
Robotic Lawn Mower
Copyright (c) 2017 by Kai WÃ¼rtz

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

#include <string>
#include "Blackboard.h"
#include "NodeStack.h"



/**
 * Return values of and valid states for behaviors.
 */
enum NodeStatus
{
    BH_FAILURE = 0,
    BH_SUCCESS = 1,
    BH_RUNNING = 2,
    BH_ABORTED = 3,
    BH_INVALID = 4,
    BH_RESET_SUCCSESS = 5,
    BH_RESET_FAILURE = 6,
    BH_FREEZ_SUCCSESS = 7,
    BH_FREEZ_FAILURE = 8
};

extern const char* enumNodeStatusStrings[];

// ============================================================================
/**
 * Base class for actions, conditions and composites.
 */
class Node
{
public:
    int nodeId;
    char* nodeName;

    /// Overwrite this function with your code. This is the maincode of the node. Must return BH_RUNNING, BH_SUCCESS or BH_FAILIURE.
    virtual NodeStatus onUpdate(Blackboard& bb)= 0;
    /// Overwrite this function with your code. Will be called every time the node has another current state than BH_RUNNING
    virtual void onInitialize(Blackboard& bb);
    /** Overwrite this function with your code. Will be called every time onUpdate returns another state than BH_RUNNING
    *   status tells the function  if abort is calling or terminating by node itself
    */
    virtual void onTerminate(NodeStatus status,Blackboard& bb); 

    Node();
    virtual ~Node();

    /// This function in base node will be called to run a node. It will then call onInitialize, onUpdate, onTerminate and do some other stuff.
    NodeStatus tick(Blackboard& bb);

    void reset();
    void abort(Blackboard& bb); // will be called from BehaviorTree class to abort a node
    bool isTerminated() const;
    bool isRunning() const;
    NodeStatus getNodeStatus() const;
    unsigned long getTimeInNode();
    void setTimeInNode(unsigned long t);

    NodeStatus m_eNodeStatus;
private:
   unsigned long _start_time_of_node;
};

// ============================================================================

/**
*  Composites
*  A composite node can have one or more children. The node is responsible to propagate the Blackboard signal to its children, respecting some order.
*  A composite node also must decide which and when to return the state values of its children, when the value is SUCCESS or FAILURE.
*  Notice that, when a child returns RUNNING, the composite node must return the state immediately.
*/
class CompositeNode : public Node    //  This type of Node follows the Composite Pattern, containing a list of other Nodes.
{
    
private:
    void addChild(Node* child);   
    void setupNumberOfChildren( const int& bufferSize );
protected:
    int _size;
    Node** _children;

    void _clear();

public:
    CompositeNode();
    virtual ~CompositeNode();

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

};

// ============================================================================

/**
*  The sequence node calls its children sequentially until one of them returns FAILURE or RUNNING. If all children return the success state, the sequence also returns SUCCESS.
*/
class Sequence : public CompositeNode
{
public:
    virtual NodeStatus onUpdate(Blackboard& bb);
};

// ============================================================================


/**
*  MemSequence is similar to Sequence node, but when a child returns a RUNNING state, its index is recorded and in the next tick the
*  MemSequence call the child recorded directly, without calling previous children again.
*/
class MemSequence : public CompositeNode
{
    int runningChild;
public:
    MemSequence();
    virtual void onInitialize(Blackboard& bb);
    virtual NodeStatus onUpdate(Blackboard& bb);
};

// ============================================================================

// MemRing executes one node and returns the result back to the parent. When called again, the next child will be executed if not running is returned.
// Will not be aborted like a sequence when returning Failure or Success or by Stack abort. Returns always the result of the child. 
// A child node can reset the MemRing while returning BH_RESETx or Freez it with  BH_FREEZ_SUCCSESS, BH_FREEZ_FAILURE
class MemRing : public CompositeNode
{
    int runningChild;
public:
    MemRing();
    virtual void onInitialize(Blackboard& bb);
    virtual NodeStatus onUpdate(Blackboard& bb);
};

// ============================================================================

/**
*  The Selector node (sometimes called priority) Blackboards its children sequentially until one of them returns SUCCESS, RUNNING. If all children return the failure state, the priority also returns FAILURE.
*/
class Selector : public CompositeNode
{
public:
    virtual NodeStatus onUpdate(Blackboard& bb);
};

// ============================================================================

/**
*  MemSelector is similar to Selector node, but when a child returns a  RUNNING state, its index is recorded and in the next Blackboard the,
*  MemSelector  calls the child recorded directly, without calling previous children again.
*/
class MemSelector : public CompositeNode
{
    int runningChild;
public:
    MemSelector();
    virtual void onInitialize(Blackboard& bb);
    virtual NodeStatus onUpdate(Blackboard& bb);
};

// ============================================================================
/**
*  Parallel 
* Parallel executes the children parallel.
* returns running if all children return running
* if one child suceed or fail the entire operation suceed or fail
*/
class Parallel : public CompositeNode
{
	int runningChild;
public:
	Parallel();
	virtual void onInitialize(Blackboard& bb);
	virtual NodeStatus onUpdate(Blackboard& bb);
	virtual void onTerminate(NodeStatus status, Blackboard& bb);
};

// ============================================================================
/**
* TimeSwitch switches from node1 to node2 after a specified amount of time
* If node1 returns BH_FAILURE, then TimeSwitch will be reseted and returns BH_FAILURE also.
* If node1 returns BH_RUNNING or BH_SUCCESS, then TimeSwitch will always return BH_RUNNING.
* If node2 returns BH_SUCCESS or BH_FAILURE, waittimeExpired will be reseted and node1 starts again.
* If node2 returns BH_RUNNING, then TimeSwitch will returning BH_RUNNING also.
* Use addChildren(Node* child1, Node* child2); to set the child nodes
*/
class TimeSwitch : public CompositeNode
{
private:
    uint32_t m_ulWaitMillis;
    uint32_t m_ulStartTime;
    bool     waittimeExpired;

public:
    TimeSwitch();
    virtual void onInitialize(Blackboard& bb);
    virtual NodeStatus onUpdate(Blackboard& bb);
    void setWaitMillis(unsigned long millis);
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
    void setChild (Node* newChild);
};

// ============================================================================
/**
*   Like the NOT operator, the inverter decorator negates the result of its child node, i.e., SUCCESS state becomes FAILURE, and FAILURE becomes SUCCESS.
*   Notice that, inverter does not change RUNNING or ERROR states, as described in algorithm below.
*/
class Inverter : public DecoratorNode
{
public:
    Inverter();
    Inverter(Node* child);
    virtual NodeStatus onUpdate(Blackboard& bb);
};

// ============================================================================
/**
* A succeeder will always return success, irrespective of what the child node actually returned.
* These are useful in cases where you want to process a branch of a tree where a failure is expected or anticipated, but you donâ€™t want to abandon processing of a sequence that branch sits on.
*/
class Succeeder : public DecoratorNode
{
public:
    Succeeder();
    Succeeder(Node* child);
    virtual NodeStatus onUpdate(Blackboard& bb);
};

// ============================================================================
/**
* The opposite of a Succeeder, always returning fail.  Note that this can be achieved also by using an Inverter and setting its child to a Succeeder.
*/
class Failer : public DecoratorNode
{
public:
    Failer();
    Failer(Node* child);
    virtual NodeStatus onUpdate(Blackboard& bb);
};
// ============================================================================
/**
* A repeater will reprocess its child node each time its child returns a result. These are often used at the very base of the tree,
* to make the tree to run continuously. Repeaters may optionally run their children a set number of times before returning to their parent.
*/
class Repeat : public DecoratorNode
{
public:
    Repeat();
    Repeat(Node* child);

    void setCount(int count);
    virtual void onInitialize(Blackboard& bb);
    virtual NodeStatus onUpdate(Blackboard& bb);

protected:
    int m_iLimit;
    int m_iCounter;
};


// ============================================================================
/**
* Like a repeater, these decorators will continue to reprocess their child. That is until the child finally returns a failure, at which point the repeater will return success to its parent.
*/
class RepeatUntilFail : public DecoratorNode
{
public:
    virtual NodeStatus onUpdate(Blackboard& bb);
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
    bool     waittimeExpired;

public:
    WaitDecorator();
    virtual void onInitialize(Blackboard& bb);
    virtual NodeStatus onUpdate(Blackboard& bb);
    void setWaitMillis(unsigned long millis);
};

// ============================================================================
/**
*  The tree must keep a list of open nodes of the last tick, in order to close them if another branch breaks the execution
* (e.g., when a priority branch returns RUNNING.). After each tick, the tree must close all open nodes that werenâ€™t executed.
*/

class BehaviourTree
{
private:
    Node* root;
    NodeStack lastRunningNodes;
public:
    bool flagLogLastNode;
    BehaviourTree();
    void setRootNode(Node* newChild);
    NodeStatus tick(Blackboard& myBlackboard);
    void reset(Blackboard& myBlackboard);
};




#endif

