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
#include "engine/model/BehaviourConfiguration.h"
#include "engine/IPlanTreeVisitor.h"
#include "engine/IBehaviourPool.h"
#include "engine/model/State.h"
#include "engine/collections/SuccessCollection.h"


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

	RunningPlan::RunningPlan(PlanType* pt) :
			RunningPlan()
	{
		this->plan = nullptr;
		this->planType = pt;
		this->setBehaviour(false);
	}

	RunningPlan::RunningPlan(BehaviourConfiguration* bc) :
			RunningPlan()
	{
		this->plan = bc;
		this->setBehaviour(true);
		this->bp = AlicaEngine::getInstance()->getBehaviourPool();
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

	bool RunningPlan::isAllocationNeeded()
	{
		return this->allocationNeeded;
	}
	void RunningPlan::setAllocationNeeded(bool need)
	{
		this->allocationNeeded = need;
	}
	bool RunningPlan::evalPreCondition()
	{
	}

	bool RunningPlan::evalRuntimeCondition()
	{
	}

	State* RunningPlan::getActiveState()
	{
	}

	void RunningPlan::addChildren(shared_ptr<list<RunningPlan*> > runningPlans)
	{
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

	bool RunningPlan::isBehaviour()
	{
		return behaviour;
	}
	void RunningPlan::setFailHandlingNeeded(bool failHandlingNeeded)
	{
		//TODO finish
		this->failHandlingNeeded = failHandlingNeeded;
	}

	void RunningPlan::setBehaviour(bool behaviour)
	{
		this->behaviour = behaviour;
	}

	list<RunningPlan*> RunningPlan::getChildren()
	{
		return children;
	}

	void RunningPlan::setChildren(list<RunningPlan*> children)
	{
		this->children = children;
	}

	AbstractPlan* RunningPlan::getPlan()
	{
		return plan;
	}

	void RunningPlan::setPlan(AbstractPlan* plan)
	{
		//TODO finish
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
		//TODO finish
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
		//TODO finish
		return status;
	}
	PlanType* RunningPlan::getPlanType()
	{
		return planType;

	}

	void RunningPlan::clearFailures()
	{
	}

	void RunningPlan::clearFailedChildren()
	{
		this->failedSubPlans.clear();
	}

	void RunningPlan::addFailure()
	{
	}

	void RunningPlan::addChildren(list<RunningPlan*> children)
	{
	}

	int RunningPlan::getFailure()
	{
	}

	void RunningPlan::deactivateChildren()
	{
		for (RunningPlan* r : this->children)
		{
			r->deactivate();
		}
	}

	void RunningPlan::clearChildren()
	{
		this->children.clear();
	}

	void RunningPlan::setFailedChildren(AbstractPlan* p)
	{
	}

	void RunningPlan::adaptAssignment(RunningPlan* r)
	{
	}

	unique_ptr<list<int> > RunningPlan::getRobotsAvail()
	{
		return move(robotsAvail);
	}

	EntryPoint* RunningPlan::getActiveEntryPoint()
	{
		return activeEntryPoint;
	}

	void RunningPlan::setActiveEntryPoint(EntryPoint* activeEntryPoint)
	{
		//TODO finish
		this->activeEntryPoint = activeEntryPoint;
	}

	CycleManager* RunningPlan::getCycleManagement()
	{
		return cycleManagement;
	}

	void RunningPlan::setCycleManagement(CycleManager* cycleManagement)
	{
		this->cycleManagement = cycleManagement;
	}

	void RunningPlan::setFailedChild(AbstractPlan* child)
	{
		if (this->failedSubPlans.find(child) != this->failedSubPlans.end())
		{
			this->failedSubPlans.at(child)++;}
		else
		{
			this->failedSubPlans.insert(pair<AbstractPlan*, int>(child, 1));
		}

	}

	void RunningPlan::setRobotAvail(int robot)
	{
		auto iter = find(this->robotsAvail->begin(), this->robotsAvail->end(), robot);
		if (iter != this->robotsAvail->end())
		{
			return;
		}
		this->robotsAvail->push_back(robot);
	}

	void RunningPlan::setRobotUnAvail(int robot)
	{
		auto iter = find(this->robotsAvail->begin(), this->robotsAvail->end(), robot);
		if (iter != this->robotsAvail->end())
		{
			this->robotsAvail->erase(iter);
		}

	}

	void RunningPlan::accept(IPlanTreeVisitor* vis)
	{
		vis->visit(this);
		for (int i = 0; i < this->children.size(); i++)
		{
			auto iter = this->children.begin();
			advance(iter, i);
			(*iter)->accept(vis);
		}
	}

	void RunningPlan::deactivate()
	{
		this->active = false;
		if (this->isBehaviour())
		{
			bp->stopBehaviour(shared_from_this());
		}
		else
		{
			this->to->notifyRobotLeftPlan(this->plan);
		}
	}

	bool RunningPlan::allChildrenStatus(PlanStatus ps)
	{
		for (int i = 0; i < this->children.size(); i++)
		{
			auto iter = this->children.begin();
			advance(iter, i);
			if (ps != (*iter)->getStatus())
			{
				return false;
			}
		}
		return true;
	}

	bool RunningPlan::anyChildrenTaskSuccess()
	{
		for(int i = 0; i < this->children.size(); i++)
		{
			auto iter = this->children.begin();
			advance(iter, i);
			if((*iter)->isBehaviour())
			{
				if((*iter)->getStatus() == PlanStatus::Success)
				{
					return true;
				}
			}
			else if((*iter)->getActiveState() != nullptr && (*iter)->getActiveState()->isSuccessState())
			{
				return true;
			}
			for(shared_ptr<list<int> > successes : (*iter)->getAssignment()->getEpSuccessMapping()->getRobots())
			{
				if(find(successes->begin(), successes->end(), this->ownId) != successes->end())
				{
					return true;
				}
			}
		}
		return false;
	}

	bool RunningPlan::anyChildrenTaskFailure()
	{
		for(int i = 0; i < this->children.size(); i++)
		{
			auto iter = this->children.begin();
			advance(iter, i);
			if((*iter)->getStatus() == PlanStatus::Failed)
			{
				return true;
			}
			if((*iter)->getActiveState() != nullptr && (*iter)->getActiveState()->isFailureState())
			{
				return true;
			}
		}
		return false;
	}

	bool RunningPlan::anyChildrenTaskTerminated()
	{
		for(int i = 0; i < this->children.size(); i++)
		{
			auto iter = this->children.begin();
			advance(iter, i);
			if((*iter)->isBehaviour())
			{
				if((*iter)->getStatus() == PlanStatus::Failed || (*iter)->getStatus() == PlanStatus::Success)
				{
					return true;
				}
			}
			else if((*iter)->getActiveState() != nullptr && (*iter)->getActiveState()->isTerminal())
			{
				return true;
			}
		}
		return false;
	}

	void RunningPlan::activate()
	{
	}

	void RunningPlan::limitToRobots(vector<int> robots)
	{
	}

	void RunningPlan::revokeAllConstraints()
	{
	}

	void RunningPlan::attachPlanConstraints()
	{
	}

	bool RunningPlan::recursiveUpdateAssignment(list<SimplePlanTree*> spts, list<int> availableAgents,
												list<int> noUpdates, unsigned long now)
	{
	}

	void RunningPlan::ToMessage(list<long> message, RunningPlan& deepestNode, int& depth, int curDepth)
	{
	}

	string RunningPlan::toString()
	{
	}

	bool RunningPlan::anyChildrenStatus(PlanStatus ps)
	{
		for (int i = 0; i < this->children.size(); i++)
		{
			auto iter = this->children.begin();
			advance(iter, i);
			if (ps == (*iter)->getStatus())
			{
				return true;
			}
		}
		return false;
	}

	void RunningPlan::setConstraintStore(ConstraintStore* constraintStore)
	{
		this->constraintStore = constraintStore;
	}

} /* namespace alica */

