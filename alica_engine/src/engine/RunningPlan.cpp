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
#include "engine/model/EntryPoint.h"
#include "engine/Assignment.h"
#include "engine/constraintmodul/ConstraintStore.h"
#include "engine/allocationauthority/CycleManager.h"
#include "engine/model/State.h"
#include "engine/rules/RuleBook.h"

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
		this->constraintStore = new ConstraintStore(this);
		this->cycleManagement = new CycleManager(this);

	}
	RunningPlan::RunningPlan(Plan* plan) :
			RunningPlan()
	{
		this->plan = plan;
		vector<EntryPoint*> epCol;
		transform(plan->getEntryPoints().begin(), plan->getEntryPoints().end(), back_inserter(epCol),
					[](map<long, EntryPoint*>::value_type& val)
					{	return val.second;});

		sort(epCol.begin(), epCol.end(), EntryPoint::compareTo);
		unordered_set<int> robots[epCol.size()];

		for (int i = 0; i < epCol.size(); i++)
		{
			robots[i] = unordered_set<int>();
		}

		this->setBehaviour(false);

	}

	bool RunningPlan::getFailHandlingNeeded() const
	{
		return this->failHandlingNeeded;
	}
	void RunningPlan::setParent(RunningPlan* s)
	{
		this->parent = s;
	}
	RunningPlan* RunningPlan::getParent() const
	{
		return this->parent;
	}

	/**
	 * Called once per Engine iteration, performs all neccessary checks and executes rules from the rulebook.
	 * @param rules
	 * @return PlanChange
	 */
	PlanChange RunningPlan::tick(RuleBook* rules){

		this->cycleManagement->update();
		PlanChange myChange = rules->visit(this);

		PlanChange childChange = PlanChange::NoChange;
		for(RunningPlan* rp : this->children)
		{
			childChange = rules->updateChange(childChange, rp->tick(rules));
		}
		if(childChange != PlanChange::NoChange && childChange != PlanChange::InternalChange)
		{
			myChange = rules->updateChange(myChange, rules->visit(this));
		}

		return myChange;
	}

	bool RunningPlan::isAllocationNeeded() const
	{
		return this->allocationNeeded;
	}
	void RunningPlan::setAllocationNeeded(bool need)
	{
		this->allocationNeeded = need;
	}
	void RunningPlan::moveState(State* nextState)
	{
		//TODO
	}

	CycleManager* RunningPlan::getCycleManager() const
	{
		return this->cycleManagement;
	}
	ConstraintStore* RunningPlan::getConstraintStore() const
	{
		return this->constraintStore;
	}

	State* RunningPlan::getActiveState() const
	{
		return this->activeState;
	}

	RunningPlan::~RunningPlan()
	{
	}

	bool RunningPlan::isBehaviour() const
	{
		return behaviour;
	}
	void RunningPlan::setFailHandlingNeeded(bool failHandlingNeeded)
	{
		this->failHandlingNeeded = failHandlingNeeded;
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
		for (RunningPlan* c : this->children)
		{
			c->printRecursive();
		}
		if (this->children.size() > 0)
		{
			cout << "END CHILDREN of " << (this->plan == nullptr ? "NULL" : this->plan->getName()) << endl;
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

	void RunningPlan::setRobotsAvail(unique_ptr<list<int> > robots)
	{
		this->robotsAvail->clear();
		this->robotsAvail = move(robots);
	}

	EntryPoint* RunningPlan::getOwnEntryPoint() const
	{
		return this->activeEntryPoint;
	}
	void RunningPlan::setActiveState(State* s)
	{
		this->activeState = s;
	}

	void RunningPlan::setOwnEntryPoint(EntryPoint* value)
	{
		if (this->activeEntryPoint != value)
		{
			this->assignment->removeRobot(ownId);
			this->activeEntryPoint = value;
			if (this->activeEntryPoint != nullptr)
			{
				this->activeState = this->activeEntryPoint->getState();
				this->assignment->addRobot(ownId, this->activeEntryPoint, this->activeState);
			}
		}
	}

	PlanStatus RunningPlan::getStatus() const
	{
		return status;
	}
	PlanType* RunningPlan::getPlanType()
	{
		return planType;

	}

} /* namespace alica */

