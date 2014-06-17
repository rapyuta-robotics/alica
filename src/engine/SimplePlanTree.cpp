/*
 * SimplePlanTree.cpp
 *
 *  Created on: Jun 16, 2014
 *      Author: stefan
 */

#include <engine/SimplePlanTree.h>

namespace alica
{

	SimplePlanTree::SimplePlanTree()
	{
		// TODO Auto-generated constructor stub

	}

	SimplePlanTree::~SimplePlanTree()
	{
		// TODO Auto-generated destructor stub
	}

	EntryPoint* SimplePlanTree::getEntryPoint() const
	{
		return entryPoint;
	}

	void SimplePlanTree::setEntryPoint(EntryPoint* entryPoint)
	{
		this->entryPoint = entryPoint;
	}

	State* SimplePlanTree::getState() const
	{
		return state;
	}

	void SimplePlanTree::setState(State* state)
	{
		this->state = state;
	}

	unordered_set<SimplePlanTree*>& SimplePlanTree::getChildren()
	{
		return children;
	}

	void SimplePlanTree::setChildren(unordered_set<SimplePlanTree*>& children)
	{
		this->children = children;
	}

	int SimplePlanTree::getRobotId() const
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

	const list<long>& SimplePlanTree::getStateIds() const
	{
		return stateIds;
	}

	void SimplePlanTree::setStateIds(const list<long>& stateIds)
	{
		this->stateIds = stateIds;
	}

} /* namespace alica */


