/*
 * RunningPlan.cpp
 *
 *  Created on: Jun 10, 2014
 *      Author: Stefan Jakob
 */

#include "engine/RunningPlan.h"
#include "engine/model/AbstractPlan.h"
#include "engine/AlicaEngine.h"
#include "engine/ITeamObserver.h"
#include "engine/model/Plan.h"


namespace alica
{

	RunningPlan::RunningPlan()
	{
		this->to = AlicaEngine::getInstance()->getTeamObserver();
		this->ownId = to->getOwnId();
		this->children = list<RunningPlan*>();
		this->robotsAvail = unique_ptr<list<int> >();
		this->status = PlanStatus::Running;
		this->failCount = 0;
		this->basicBehaviour = nullptr;
		this->failedSubPlans = map<AbstractPlan*, int>();
		this->active = false;
		this->allocationNeeded = false;
		this->failHandlingNeeded = false;

	}
	RunningPlan::RunningPlan(Plan* plan)
	{
		this->plan = plan;
		vector<EntryPoint*> epCol;
		transform(plan->getEntryPoints().begin(), plan->getEntryPoints().end(), back_inserter(epCol), [](map<long, EntryPoint*>::value_type& val){return val.second;} );

	}

	void RunningPlan::setAllocationNeeded(bool allocationNeeded)
	{
		this->allocationNeeded = allocationNeeded;
	}

	RunningPlan::~RunningPlan()
	{
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

	void RunningPlan::printRecursive()
	{
		cout << this << endl;
		for(RunningPlan* c : this->children)
		{
			c->printRecursive();
		}
		if(this->children.size() > 0)
		{
			cout << "END CHILDREN of " << (this->plan==nullptr?"NULL":this->plan->getName()) << endl;
		}
	}
	Assignment* RunningPlan::getAssignment()
	{
		return assignment;
	}

	void RunningPlan::setAssignment(Assignment* assignment)
	{
		this->assignment = assignment;
	}

	unsigned long RunningPlan::getPlanStartTime()
	{
		return planStartTime;
	}

	unsigned long RunningPlan::getStateStartTime()
	{
		return stateStartTime;
	}

	bool RunningPlan::isActive()
	{
		return active;
	}

	void RunningPlan::setActive(bool active)
	{
		this->active = active;
	}

	void RunningPlan::setRobotsAvail(unique_ptr<list<int> >  robots)
	{
		this->robotsAvail->clear();
		this->robotsAvail = move(robots);
	}

} /* namespace alica */


