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

	const unordered_set<SimplePlanTree*>& SimplePlanTree::getChildren() const
	{
		return children;
	}

	void SimplePlanTree::setChildren(const unordered_set<SimplePlanTree*>& children)
	{
		this->children = children;
	}

} /* namespace alica */
