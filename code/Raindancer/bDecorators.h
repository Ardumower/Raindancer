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
*/
#ifndef BH_DECORATORS_H
#define BH_DECORATORS_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "BehaviourTree.h"

//#define DEBUG_DECORATOR_NODE_TERMINATION 1
#ifdef DEBUG_DECORATOR_NODE_TERMINATION
#  define DDNT(x) x
#else
#  define DDNT(x)
#endif


class TdnCruiseSpiral: public DecoratorNode
{
public:

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if ( bb.flagCruiseSpiral ) {
            NodeStatus s = m_pChild->tick(bb);
            return s;
        }

        return BH_FAILURE;
    }

    virtual void onTerminate(NodeStatus status, Blackboard& bb) {
        /* 
        bb.flagCruiseSpiral = false;

        errorHandler.setInfo("TdnCruiseSpiral ot diable flagCruiseSpiral\r\n");
        if (status == BH_ABORTED) {
            errorHandler.setInfo("TdnCruiseSpiral BH_ABORTED\r\n");
        } 
        */  
    }
};


class TdnBumperActivated: public DecoratorNode
{
public:

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if (bb.flagBumperInsidePerActivated ) {
            NodeStatus s = m_pChild->tick(bb);
            return s;
        }

        return BH_FAILURE;
    }

    virtual void onTerminate(NodeStatus status, Blackboard& bb) {
        //if (status != BH_ABORTED) {
            bb.flagBumperInsidePerActivated  = false;
           
           //if (status != BH_FAILURE) 
           //    errorHandler.setInfo("!03,bb.flagBumperInsidePerActivated  = false;\r\n");
               
            DDNT(debug->printf("bb.flagBumperInsidePerActivated  = false;\r\n");)
        //}
    }
};


class TdnBumpPeriActivated: public DecoratorNode
{
public:

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if (bb.flagBumperOutsidePerActivated) {
            //errorHandler.setInfo("!03,TdnBumpPeriActivated m_pChild->tick(bb)\r\n");
            NodeStatus s = m_pChild->tick(bb);
            return s;
        }

        return BH_FAILURE;
    }

    virtual void onTerminate(NodeStatus status, Blackboard& bb) {
        //if (status != BH_ABORTED) {
            bb.flagBumperOutsidePerActivated = false;
            DDNT(debug->printf("bb.flagBumperOutsidePerActivated  = false;\r\n");)
        //}
    }
};



class TdnMowing: public DecoratorNode
{
public:

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if (bb.flagEnableMowing) {
            NodeStatus s = m_pChild->tick(bb);
            return s;
        }

        return BH_FAILURE;
    }


};

class TdnPermeterTracking: public DecoratorNode
{
public:

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if (bb.flagEnablePerimetertracking) {
            NodeStatus s = m_pChild->tick(bb);
            return s;
        }

        return BH_FAILURE;
    }


};


class TdnRestoreHistory: public DecoratorNode
{
public:

	virtual NodeStatus onUpdate(Blackboard& bb) {

		if (bb.flagEnableRestoreHistory) {
			NodeStatus s = m_pChild->tick(bb);
			return s;
		}

		return BH_FAILURE;
	}

};

class TdnCharging: public DecoratorNode
{
public:

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if (bb.flagEnableCharging) {
            NodeStatus s = m_pChild->tick(bb);
            return s;
        }

        return BH_FAILURE;
    }

};


class TdnGotoAreaX: public DecoratorNode
{
public:

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if (bb.flagEnableGotoAreaX) {
            NodeStatus s = m_pChild->tick(bb);
            return s;
        }

        return BH_FAILURE;
    }

};


class TdnFindPerimeter: public DecoratorNode
{
public:

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if (bb.flagEnableFindPerimeter) {
            NodeStatus s = m_pChild->tick(bb);
            return s;
        }

        return BH_FAILURE;
    }

};

class TdnSetbbShortWayCounter : public DecoratorNode
{
public:

    virtual NodeStatus onUpdate(Blackboard& bb) {

        if (m_pChild == NULL) {
            errorHandler.setInfo(F("!03,dnSetbbShortWayCounter child == NULL\r\n"));
            return BH_FAILURE;
        }

        NodeStatus s = m_pChild->tick(bb);
        return s;
    }

    virtual void onTerminate(NodeStatus status, Blackboard& bb) {
 
            
    }

};

#endif

