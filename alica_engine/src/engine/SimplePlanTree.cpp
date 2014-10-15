/*
 * SimplePlanTree.cpp
 *
 *  Created on: Jun 16, 2014
 *      Author: Stefan Jakob
 */

#include <engine/SimplePlanTree.h>
#include "engine/model/EntryPoint.h"
#include "engine/model/Plan.h"

namespace alica
{

	SimplePlanTree::SimplePlanTree()
	{
		this->state = nullptr;
		this->newSimplePlanTree = false;
		this->receiveTime = 0;
		this->entryPoint = nullptr;
		this->parent =nullptr;
	}

	SimplePlanTree::~SimplePlanTree()
	{

	}

	bool SimplePlanTree::containsPlan(AbstractPlan* plan)
	{
		if(this->getEntryPoint()->getPlan() == plan)
		{
			return true;
		}
		for(shared_ptr<SimplePlanTree> spt : this->getChildren())
		{
			if(spt->containsPlan(plan))
			{
				return true;
			}
		}
		return false;
	}

	EntryPoint* SimplePlanTree::getEntryPoint()
	{
		return this->entryPoint;
	}

	void SimplePlanTree::setEntryPoint(EntryPoint* entryPoint)
	{
		this->entryPoint = entryPoint;
	}

	State* SimplePlanTree::getState()
	{
		return state;
	}

	void SimplePlanTree::setState(State* state)
	{
		this->state = state;
	}

	unordered_set<shared_ptr<SimplePlanTree> > SimplePlanTree::getChildren()
	{
		return children;
	}

	void SimplePlanTree::setChildren(unordered_set<shared_ptr<SimplePlanTree> > children)
	{
		this->children = children;
	}

	int SimplePlanTree::getRobotId()
	{
		return robotId;
	}

	void SimplePlanTree::setRobotId(int robotId)
	{
		this->robotId = robotId;
	}

	bool SimplePlanTree::isNewSimplePlanTree() const
	{
		return newSimplePlanTree;
	}

	void SimplePlanTree::setNewSimplePlanTree(bool newSimplePlanTree)
	{
		this->newSimplePlanTree = newSimplePlanTree;
	}
	long SimplePlanTree::getReceiveTime() const
	{
		return receiveTime;
	}

	void SimplePlanTree::setReceiveTime(long receiveTime)
	{
		this->receiveTime = receiveTime;
	}

	list<long>& SimplePlanTree::getStateIds()
	{
		return stateIds;
	}

	void SimplePlanTree::setStateIds(list<long>& stateIds)
	{
		this->stateIds = stateIds;
	}

} /* namespace alica */


