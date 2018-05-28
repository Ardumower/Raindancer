/*
Robotic Lawn Mower
Copyright (c) 2017 by Kai Würtz

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

#include "BehaviourTree.h"

//#define DEBUG_BHT_TREE 1
//#define DEBUG_BHT_STACK 1
//#define DEBUG_BHT_ABORT 1

#ifdef DEBUG_BHT_TREE
#  define DTREE(x) x
#else
#  define DTREE(x)
#endif

#ifdef DEBUG_BHT_STACK
#  define DSTACK(x) x
#else
#  define DSTACK(x)
#endif

#ifdef DEBUG_BHT_ABORT
#  define DABORT(x) x
#else
#  define DABORT(x)
#endif

static int nodeIdCounter = 0;

const char* enumNodeStatusStrings[] = { "BH_FAILURE", "BH_SUCCESS", "BH_RUNNING", "BH_ABORTED", "BH_INVALID","BH_RESET_SUCCSESS","BH_RESET_FAILURE","BH_FREEZ_SUCCSESS","BH_FREEZ_FAILURE" };


// ============================================================================
/** Node Class
 *  Base class for actions, conditions and composites.
 */
void Node::onInitialize(Blackboard& bb)
{
}

void Node::onTerminate(NodeStatus status,Blackboard& bb)
{
    //if(status != BH_ABORTED) {
    //}
}

Node::Node()
    :   m_eNodeStatus(BH_INVALID)
{
    nodeId = nodeIdCounter++;
}

Node::~Node()
{
}

// This function will be called to run a node.
NodeStatus Node::tick(Blackboard& bb)
{
    // push to stack on entering
    DSTACK(errorHandler.setInfoNoLog(F("PUSH %s \r\n"),nodeName);)
    bb.runningNodes.Push(this);

    if (m_eNodeStatus != BH_RUNNING) {
        _start_time_of_node = millis();
        DTREE(debug->printf("onInitialize %s %d\r\n",nodeName, nodeId);)
        onInitialize(bb);
    }


    // only for showing the current last active node. Every called node in the tree writes its instance in the variable until the last one is in there.
    bb.lastNodeCurrentRun = this;
   
    m_eNodeStatus = onUpdate(bb);

    if (m_eNodeStatus != BH_RUNNING) {
        DTREE(debug->printf("onTerminate %s %d status %s\r\n",nodeName, nodeId, enumNodeStatusStrings[m_eNodeStatus]);)
        onTerminate(m_eNodeStatus, bb);
        // pop from stack because status is != running
        DSTACK(errorHandler.setInfoNoLog(F("POP %s \r\n"),nodeName);)
        bb.runningNodes.Pop();
    }

    DTREE(debug->printf("%s %d return: %s\r\n",nodeName, nodeId, enumNodeStatusStrings[m_eNodeStatus]);)
    return m_eNodeStatus;
}

//  called from Repeat::onUpdate(Blackboard& bb) to run node again
void Node::reset()
{
    m_eNodeStatus = BH_INVALID;
}

// will be called from BehaviorTree class to abort a node
void Node::abort(Blackboard& bb)
{
    // Only abort if in state running
    if (m_eNodeStatus == BH_RUNNING) {
        DTREE(debug->printf( "Node aborted: %s %d\r\n", nodeName,nodeId);)
        onTerminate(BH_ABORTED, bb);
        m_eNodeStatus = BH_ABORTED;
    }
}

bool Node::isTerminated() const
{
    return m_eNodeStatus == BH_SUCCESS  ||  m_eNodeStatus == BH_FAILURE;
}

bool Node::isRunning() const
{
    return m_eNodeStatus == BH_RUNNING;
}

NodeStatus Node::getNodeStatus() const
{
    return m_eNodeStatus;
}


unsigned long Node::getTimeInNode()
{
    return millis() - _start_time_of_node;
}

void Node::setTimeInNode(unsigned long t)
{
    _start_time_of_node = t;
}

// ============================================================================

/*
Composites
==========
A composite node can have one or more children. The node is responsible to propagate the Blackboard signal to its children, respecting some order.
A composite node also must decide which and when to return the state values of its children, when the value is SUCCESS or FAILURE.
Notice that, when a child returns RUNNING, the composite node must return the state immediately.
*/


void CompositeNode::_clear()
{
    for (int i = 0; i < _size; i++) {
        _children[i] = NULL;
    }
}

CompositeNode::CompositeNode():_size(0) {}

CompositeNode::~CompositeNode()
{
    delete[] _children;
}

void CompositeNode::setupNumberOfChildren( const int& bufferSize )
{
    if(_size == 0) {
        _children = new Node*[bufferSize];
        _size = bufferSize;
        _clear();
    } else {
        sprintf(errorHandler.msg,"!03,ERROR CompositeNode Array already filled %s\r\n",nodeName);
        errorHandler.setError(); 
    }
}

void CompositeNode::addChild(Node* child)
{
    // Find an empty slot
    for (int i = 0; i < _size; i++) {
        if (!_children[i]) {
            // Found an empty slot, now add child
            _children[i] = child;
            return;
        }
    }
    // Array is full
    sprintf(errorHandler.msg,"!03,ERROR CompositeNode Array is full %s\r\n",nodeName);
    errorHandler.setError();
    return;
}

void CompositeNode::addChildren(Node* child1)
{
    setupNumberOfChildren(1);
    addChild(child1);
}
void CompositeNode::addChildren(Node* child1, Node* child2)
{
    setupNumberOfChildren(2);
    addChild(child1);
    addChild(child2);
}
void CompositeNode::addChildren(Node* child1, Node* child2, Node* child3)
{
    setupNumberOfChildren(3);
    addChild(child1);
    addChild(child2);
    addChild(child3);
}
void CompositeNode::addChildren(Node* child1, Node* child2, Node* child3, Node* child4)
{
    setupNumberOfChildren(4);
    addChild(child1);
    addChild(child2);
    addChild(child3);
    addChild(child4);
}
void CompositeNode::addChildren(Node* child1, Node* child2, Node* child3, Node* child4, Node* child5)
{
    setupNumberOfChildren(5);
    addChild(child1);
    addChild(child2);
    addChild(child3);
    addChild(child4);
    addChild(child5);
}

void CompositeNode::addChildren(Node* child1, Node* child2, Node* child3, Node* child4, Node* child5, Node* child6)
{
    setupNumberOfChildren(6);
    addChild(child1);
    addChild(child2);
    addChild(child3);
    addChild(child4);
    addChild(child5);
    addChild(child6);
}


void CompositeNode::addChildren(Node* child1, Node* child2, Node* child3, Node* child4, Node* child5, Node* child6, Node* child7)
{
    setupNumberOfChildren(7);
    addChild(child1);
    addChild(child2);
    addChild(child3);
    addChild(child4);
    addChild(child5);
    addChild(child6);
    addChild(child7);
}

void CompositeNode::addChildren(Node* child1, Node* child2, Node* child3, Node* child4, Node* child5, Node* child6, Node* child7, Node* child8)
{
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

void CompositeNode::addChildren(Node* child1, Node* child2, Node* child3, Node* child4, Node* child5, Node* child6, Node* child7, Node* child8, Node* child9)
{
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

void CompositeNode::addChildren(Node* child1, Node* child2, Node* child3, Node* child4, Node* child5, Node* child6, Node* child7, Node* child8, Node* child9, Node* child10)
{
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


// ============================================================================
//The sequence node calls its children sequentially until one of them returns FAILURE or RUNNING. If all children return the success state, the sequence also returns SUCCESS.
NodeStatus Sequence::onUpdate(Blackboard& bb)
{
    NodeStatus s;
    for (int i = 0; i < _size; i++) {
        // Object exists?
        if (_children[i]) {
            s = _children[i]->tick(bb);
            if (s != BH_SUCCESS) {
                // If one child fails, then enter operation run() fails.  Success only results if all children succeed.
                return s;
            }
        }
    }
    return BH_SUCCESS;  // All children failed so the entire run() operation fails.
}

// ============================================================================
// MemSequence is similar to Sequence node, but when a child returns a RUNNING state, its index is recorded and in the next tick the
// MemSequence call the child recorded directly, without calling previous children again.

MemSequence::MemSequence(): runningChild(0) {}

void MemSequence::onInitialize(Blackboard& bb)
{
    runningChild = 0;
    Node::onInitialize(bb);  // Print out that onInitialize was called. For debugging.
}

NodeStatus MemSequence::onUpdate(Blackboard& bb)
{
    NodeStatus s;
    for(int i = runningChild; i < _size; i++) {
        // Object exists?
        if(_children[i]) {
            s = _children[i]->tick(bb);
            if (s != BH_SUCCESS) { // If one child fails, then enter operation run() fails.  Success only results if all children succeed.
                if (s == BH_RUNNING) {
                    runningChild = i;
                }
                return s;
            }
        }
    }
    return BH_SUCCESS;  // All children failed so the entire run() operation fails.
}

// ============================================================================
// MemRing executes one node and returns the result back to the parent. When called again, the next child will be executed if not running is returned.
// Will not be aborted like a sequence when returning Failure or Success or by Stack abort. Returns always the result of the child. 
// A child node can reset the MemRing while returning BH_RESETx or Freez it with  BH_FREEZ_SUCCSESS, BH_FREEZ_FAILURE

// Eigentlich mÃ¼sste bei abort runningChild weitergezÃ¤hlt werden, damit nÃ¤chste sequenz beim nÃ¤chsten Aufruf lÃ¤uft
 
MemRing::MemRing(): runningChild(0) {}

void MemRing::onInitialize(Blackboard& bb)
{
    Node::onInitialize(bb);  // Print out that onInitialize was called. For debugging.
}

NodeStatus MemRing::onUpdate(Blackboard& bb)
{
    NodeStatus s;
    // Object exists?
    if(_children[runningChild]) {
        
        s = _children[runningChild]->tick(bb);
        
        if (s == BH_RUNNING) {
            //runningChild = runningChild;
        }
        else if (s == BH_FAILURE) {
            runningChild++;
        }
        else if (s == BH_SUCCESS) {
            runningChild++;
        }
        else if (s == BH_RESET_SUCCSESS) {
            runningChild = 0;
            s = BH_SUCCESS;
        }
        else if (s == BH_RESET_FAILURE) {
            runningChild = 0;
            s = BH_FAILURE;
        }
        else if (s == BH_FREEZ_SUCCSESS) {
            s = BH_SUCCESS;
        }
        else if (s == BH_FREEZ_FAILURE) {
            s = BH_FAILURE;
        }
        
        if (runningChild >= _size) {
            runningChild = 0;
        }

        return s;
    }
    return BH_SUCCESS;  // Will never been reached.
}

// ============================================================================
//The Selector node (sometimes called priority) Blackboards its children sequentially until one of them returns SUCCESS, RUNNING. 
//If all children return the failure state, the priority also returns FAILURE.

NodeStatus Selector::onUpdate(Blackboard& bb)
{
    NodeStatus s;
    for (int i = 0; i < _size; i++) {
        // Object exists?
        if (_children[i]) {
            s = _children[i]->tick(bb);
            if (s != BH_FAILURE) {
                // If one child succeeds/running, the entire operation run() succeeds/running.  Failure only results if all children fail.
                return s;
            }
        }
    }
    return BH_FAILURE;  // All children failed so the entire run() operation fails.
}

// ============================================================================

// MemSelector is similar to Selector node, but when a child returns a  RUNNING state, its index is recorded and in the next Blackboard the,
// MemSelector  calls the child recorded directly, without calling previous children again.

MemSelector::MemSelector(): runningChild(0) {}

void MemSelector::onInitialize(Blackboard& bb)
{
    runningChild = 0;
    Node::onInitialize(bb);
}

NodeStatus MemSelector::onUpdate(Blackboard& bb)
{
    NodeStatus s;
    for(int i = runningChild; i < _size; i++) {
        // Object exists?
        if(_children[i]) {
            s = _children[i]->tick(bb);
            if (s != BH_FAILURE) { // If one child succeeds/running, the entire operation run() succeeds/running.  Failure only results if all children fail.
                if (s == BH_RUNNING) {
                    runningChild = i;
                }
                return s;
            }
        }
    }
    return BH_FAILURE;  // All children failed so the entire run() operation fails.
}


// ============================================================================

// Parallel executes the children parallel.
// returns running if all children return running
// if one child suceed or fail the entire operation suceed or fail

Parallel::Parallel() : runningChild(0) {}

void Parallel::onInitialize(Blackboard& bb)
{
	runningChild = 0;
	Node::onInitialize(bb);
}

NodeStatus Parallel::onUpdate(Blackboard& bb)
{
	NodeStatus s;
	for (int i = runningChild; i < _size; i++) {
		// Object exists?
		if (_children[i]) {
			s = _children[i]->tick(bb);
			if (s != BH_RUNNING) { // If one child succeeds/fails, the entire operation run() succeeds/fails. 
				return s;
			}
		}
	}
	return BH_RUNNING;  // All children failed so the entire run() operation fails.
}


void Parallel::onTerminate(NodeStatus status, Blackboard& bb) {

	for (int i = runningChild; i < _size; i++) {
		// Object exists?
		if (_children[i]) {
				_children[i]->abort(bb);
		}
	}
}


// ============================================================================
/**
* TimeSwitch switches from node1 to node2 after a specified amount of time
* If node1 returns BH_FAILURE, then TimeSwitch will be reseted and returns BH_FAILURE also.
* If node1 returns BH_RUNNING or BH_SUCCESS, then TimeSwitch will always return BH_RUNNING.
* If node2 returns BH_SUCCESS or BH_FAILURE, node1 will next call start again.
* If node2 returns BH_RUNNING, then TimeSwitch will returning BH_RUNNING also and node2 will next time executed again.
* Use addChildren(Node* child1, Node* child2); to set the child nodes
*/
TimeSwitch::TimeSwitch(): m_ulWaitMillis(0), m_ulStartTime(0), waittimeExpired(false) {}

void TimeSwitch::onInitialize(Blackboard& bb)
{
    waittimeExpired = false;
    m_ulStartTime = millis();
    Node::onInitialize(bb);
}

NodeStatus TimeSwitch::onUpdate(Blackboard& bb)
{
    NodeStatus s;

    if(  (millis() - m_ulStartTime > m_ulWaitMillis)  && !waittimeExpired) {
        waittimeExpired = true;
        _children[0]->abort(bb);
    }

    if(waittimeExpired) {
        s = _children[1]->tick(bb);
    } else {
        s = _children[0]->tick(bb);
        if(s == BH_SUCCESS) {
            s = BH_RUNNING;
        }
    }
    return s;
}

void TimeSwitch::setWaitMillis(unsigned long millis)
{
    m_ulWaitMillis = millis;
}


// ============================================================================
/*
Decorators
Decorators are special nodes that can have only a single child. The goal of the decorator is to change the
behavior of the child by manipulating the returning value or changing its Blackboarding frequency.
*/
DecoratorNode::DecoratorNode() : m_pChild(NULL) {}
DecoratorNode::DecoratorNode(Node* child) : m_pChild(child) {}

void DecoratorNode::setChild (Node* newChild)
{
    m_pChild = newChild;
}

// ============================================================================
// Like the NOT operator, the inverter decorator negates the result of its child node, i.e., SUCCESS state becomes FAILURE, and FAILURE becomes SUCCESS.
// Notice that, inverter does not change RUNNING or ERROR states, as described in algorithm below.
Inverter::Inverter() : DecoratorNode() {}
Inverter::Inverter(Node* child) :  DecoratorNode(child) {}

NodeStatus Inverter::onUpdate(Blackboard& bb)
{

    if (m_pChild == NULL) {
        errorHandler.setError("!03,Inverter child == NULL\r\n");
        return BH_FAILURE;
    }

    NodeStatus s = m_pChild->tick(bb);
    if( s == BH_FAILURE) {
        s= BH_SUCCESS;
    } else if( s == BH_SUCCESS) {
        s =  BH_FAILURE;
    }
    return s;
}

// ============================================================================
// A succeeder will always return success, irrespective of what the child node actually returned.
// These are useful in cases where you want to process a branch of a tree where a failure is expected or anticipated, but you donâ€™t want to abandon processing of a sequence that branch sits on.
Succeeder::Succeeder(): DecoratorNode() {}
Succeeder::Succeeder(Node* child) : DecoratorNode(child) {}

NodeStatus Succeeder::onUpdate(Blackboard& bb)
{

    if (m_pChild == NULL) {
        errorHandler.setError("!03,Succeeder child == NULL\r\n");
        return BH_FAILURE;
    }

	NodeStatus s = m_pChild->tick(bb);
    if( s == BH_FAILURE) {
        s= BH_SUCCESS;
    } else if( s == BH_SUCCESS) {
        s =  BH_SUCCESS;
    }

    return s;
}


// ============================================================================
// The opposite of a Succeeder, always returning fail.  Note that this can be achieved also by using an Inverter and setting its child to a Succeeder.
Failer::Failer(): DecoratorNode() {}
Failer::Failer(Node* child) : DecoratorNode(child) {}

NodeStatus Failer::onUpdate(Blackboard& bb)
{

    if (m_pChild == NULL) {
        errorHandler.setError("!03,Failer child == NULL\r\n");
        return BH_FAILURE;
    }

	NodeStatus s = m_pChild->tick(bb);
    if( s == BH_FAILURE) {
        s= BH_FAILURE;
    } else if( s == BH_SUCCESS) {
        s =  BH_FAILURE;
    }

    return s;

}


// ============================================================================
// A repeater will reprocess its child node each time its child returns a result. These are often used at the very base of the tree,
// to make the tree to run continuously. Repeaters may optionally run their children a set number of times before returning to their parent.
//

Repeat::Repeat(): DecoratorNode() {}
Repeat::Repeat(Node* child) : DecoratorNode(child) {}

void Repeat::setCount(int count)
{
    m_iLimit = count;
}

void Repeat::onInitialize(Blackboard& bb)
{
    m_iCounter = 0;
}

NodeStatus Repeat::onUpdate(Blackboard& bb)
{
    for (;;) {
        m_pChild->tick(bb);
        if (m_pChild->getNodeStatus() == BH_RUNNING) break;
        if (m_pChild->getNodeStatus() == BH_FAILURE) return BH_FAILURE;
        if (++m_iCounter == m_iLimit) return BH_SUCCESS;
        m_pChild->reset();
    }
    return BH_RUNNING;
}


// ============================================================================
// Like a repeater, these decorators will continue to reprocess their child. That is until the child finally returns a failure, at which point the repeater will return success to its parent.

NodeStatus RepeatUntilFail::onUpdate(Blackboard& bb)
{
    while (!(BH_FAILURE == m_pChild->tick(bb))) {}
    return BH_SUCCESS;
}


// ============================================================================
// WaitDecorator node. Waits the configured amount of time until the child will be executed.
// As long as the child returns BH_RUNNING, the status waittimeExpired will be latched

WaitDecorator::WaitDecorator(): m_ulWaitMillis(0), m_ulStartTime(0), waittimeExpired(false) {}

void WaitDecorator::onInitialize(Blackboard& bb)
{
    waittimeExpired = false;
    m_ulStartTime = millis();
    //debug->printf("set start time mm_ulStartTime: %lu\r\n",m_ulStartTime);
}
NodeStatus WaitDecorator::onUpdate(Blackboard& bb)
{
    if( (millis() - m_ulStartTime > m_ulWaitMillis) && (waittimeExpired == false)) {
        waittimeExpired = true;
        //debug->printf("millis(): %lu  m_ulStartTime: %lu\r\n",millis(), m_ulStartTime);
    }

    if(waittimeExpired) {
        NodeStatus s = m_pChild->tick(bb);
        return s;
    }
    return BH_RUNNING;
}

void WaitDecorator::setWaitMillis(unsigned long millis)
{
    m_ulWaitMillis = millis;
}



// ============================================================================
// The tree must keep a list of open nodes of the last tick, in order to close them if another branch breaks the execution
// (e.g., when a priority branch returns RUNNING.). After each tick, the tree must close all open nodes that werenâ€™t executed.

BehaviourTree::BehaviourTree():flagLogLastNode(true) {}

void BehaviourTree::setRootNode(Node* newChild)
{
    root = newChild;
}

NodeStatus BehaviourTree::tick(Blackboard& bb)
{
    NodeStatus status;
    int nodeIdLastRun = -1;

    DTREE(debug->printf("**TREE BEGIN**\r\n");)

    // run the tree
    status = root->tick(bb);

    // Log current node only if node changes from last run and if flagShowLastNode is set
    if(flagLogLastNode && bb.lastNodeCurrentRun!=NULL) {

        if(bb.lastNodeLastRun != NULL) {
            nodeIdLastRun = bb.lastNodeLastRun->nodeId;
        } else {
            nodeIdLastRun = -1;
        }

        if(nodeIdLastRun != bb.lastNodeCurrentRun->nodeId) {
            NodeStatus s = bb.lastNodeCurrentRun->m_eNodeStatus;
            //sprintf(errorHandler.msg,"!02,%s %d %s\r\n", bb.lastNodeCurrentRun->nodeName, bb.lastNodeCurrentRun->nodeId ,enumNodeStatusStrings[s]);
            sprintf(errorHandler.msg,"!02,%s %s\r\n", bb.lastNodeCurrentRun->nodeName, enumNodeStatusStrings[s]);
			if (bb.flagBHTShowLastNode) {
				errorHandler.setInfo();
			}
			else {
				errorHandler.writeToLogOnly();
			}
	

            bb.lastNodeLastRun = bb.lastNodeCurrentRun ;
        }
    }


#ifdef DEBUG_BHT_STACK
	errorHandler.setInfoNoLog(F("######Stack lastRunningNodes\r\n"));
    int end1;
    end1 = lastRunningNodes.size;
	errorHandler.setInfoNoLog(F("lastRunningNodes StackSize: %d\r\n"), end1);
    for (int i=0; i<end1; i++) {
        Node* n = lastRunningNodes[i];
		errorHandler.setInfoNoLog(F("Node: %s %d\r\n"), n->nodeName, n->nodeId);
    }

	errorHandler.setInfoNoLog(F("######Stack runningNodes\r\n"));
    end1 = bb.runningNodes.size;
	errorHandler.setInfoNoLog(F("runningNodes StackSize: %d\r\n"), end1);
    for (int i=0; i<end1; i++) {
        Node* n = bb.runningNodes[i];
		errorHandler.setInfoNoLog(F("Node: %s %d\r\n"),n->nodeName, n->nodeId);
    }
	errorHandler.setInfoNoLog(F("######STACK END\r\n"));
#endif

    // Es kann vorkommen, das im aktuellen durchlauf eine node von running auf success gegangen ist.
    // Diese sich aber trotzdem noch im last Stack befindet. Hier wird dann trotzdem die abort function aufgerufen
    // abort() prÃ¼ft , ob die node noch im running state ist und terminiert diese nur dann
    // wenn diese tatsÃ¤chlich noch im running state ist.

    /* CLOSE NODES FROM LAST TICK, IF NEEDED */
    int start = -1;  // go through stack and find first node unequal
    int end = min(lastRunningNodes.size, bb.runningNodes.size);
    for (int i=0; i< end; i++) {
        Node* last  = lastRunningNodes[i];
        Node* current =  bb.runningNodes[i];
        if (last->nodeId != current->nodeId ) {
            start = i;
            DSTACK(errorHandler.setInfoNoLog(F("lastNode %s %d != currentNode %s %d\r\n"), last->nodeName, last->nodeId, current->nodeName, current->nodeId);)
            break;
        }
    }


    if(start!=-1) {
        // close last running nodes from the end of the stack
        for (int  i=lastRunningNodes.size-1; i>=start; i--) {
            Node* n = lastRunningNodes[i]; //.Get(i);
            DABORT(debug->printf( "Node called to abort (A): %s %d\r\n", n->nodeName.c_str(),n->nodeId);)
            n->abort(bb);
        }
    }


    // Wenn bei aktuellem Durchlauf kein runningNode zurÃ¼ckgemeldet wurde, in lastRunningNodes aber noch EintrÃ¤ge
    // vorhanden sind, diese aborten. Dies kann vorkommen, wenn ein Selector einen vorherigen anderen Zweig aufruft der mit Success beendet wird.
    // oder der letzte Zweig mit success/failure beendet wird.
    if(bb.runningNodes.size == 0 && lastRunningNodes.size > 0) {
        // close last running nodes from the end of the stack
        for (int  i=lastRunningNodes.size-1; i>=0; i--) {
            Node* n = lastRunningNodes[i]; //.Get(i);
            DABORT(debug->printf( "Node called to abort(B): %s %d\r\n", n->nodeName.c_str(),n->nodeId);)
            n->abort(bb);
        }
    }

    /* POPULATE lastRunningNodes with current runningNodes*/
    memcpy(&lastRunningNodes.data[0], &bb.runningNodes.data[0], NODE_STACK_MAX * sizeof(Node*));
    lastRunningNodes.size = bb.runningNodes.size;

#ifdef DEBUG_BHT_STACK
	errorHandler.setInfoNoLog(F("######Stack lastRunningNodes populated\r\n"));
    end1 = lastRunningNodes.size;
	errorHandler.setInfoNoLog(F("lastRunningNodes StackSize: %d\r\n"), end1);
    for (int i=0; i<end1; i++) {
        Node* n = lastRunningNodes[i];
		errorHandler.setInfoNoLog(F("Node: %s %d\r\n"), n->nodeName, n->nodeId);
    }
	errorHandler.setInfoNoLog(F("######STACK END\r\n"));
#endif

    // clear current stack to prepare for next run
    bb.runningNodes.Clear();
    DTREE(debug->printf("**TREE END**\r\n");)
    return status;
}


void BehaviourTree::reset(Blackboard& bb)
{

    bb.lastNodeLastRun = NULL;
    bb.lastNodeCurrentRun = NULL;


    // close last running nodes from the end of the stack
    for (int  i=lastRunningNodes.size-1; i>=0; i--) {
        Node* n = lastRunningNodes[i];
        DABORT(debug->printf( "Node called to abort (R): %s %d\r\n", n->nodeName.c_str(),n->nodeId);)
        n->abort(bb);
    }
    lastRunningNodes.Clear();

    // close current  blackboard running nodes from the end of the stack
    for (int  i=bb.runningNodes.size-1; i>=0; i--) {
        Node* n = bb.runningNodes[i];
        DABORT(debug->printf( "Node called to abort (R): %s %d\r\n", n->nodeName.c_str(),n->nodeId);)
        n->abort(bb);
    }
    bb.runningNodes.Clear();

}



