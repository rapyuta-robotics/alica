/*
 * RunningPlan.cpp
 *
 *  Created on: Jun 10, 2014
 *      Author: Stefan Jakob
 */

#include "engine/RunningPlan.h"

namespace alica
{

	RunningPlan::RunningPlan()
	{
		// TODO Auto-generated constructor stub

	}

	RunningPlan::~RunningPlan()
	{
		// TODO Auto-generated destructor stub
	}

	bool RunningPlan::isBehaviour() const
	{
		return behaviour;
	}

	void RunningPlan::setBehaviour(bool behaviour)
	{
		this->behaviour = behaviour;
	}

	const list<RunningPlan*>& RunningPlan::getChildren() const
	{
		return children;
	}

	void RunningPlan::setChildren(const list<RunningPlan*>& children)
	{
		this->children = children;
	}

	AbstractPlan* RunningPlan::getPlan() const
	{
		return plan;
	}

	void RunningPlan::setPlan(AbstractPlan* plan)
	{
		this->plan = plan;
	}

	shared_ptr<BasicBehaviour> RunningPlan::getBasicBehaviour()
	{
		return this->basicBehaviour;
	}

	void RunningPlan::setBasicBehaviour(shared_ptr<BasicBehaviour> basicBehaviour)
	{
		this->basicBehaviour = basicBehaviour;
	}


} /* namespace alica */

