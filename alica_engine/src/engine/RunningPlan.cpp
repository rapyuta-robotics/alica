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
#include "engine/collections/StateCollection.h"
#include "engine/collections/RobotEngineData.h"
#include "engine/collections/SuccessMarks.h"
#include "engine/BasicBehaviour.h"
#include "engine/model/PreCondition.h"
#include "engine/model/RuntimeCondition.h"
#include "engine/model/PlanType.h"
#include "engine/model/Task.h"
#include "engine/IAlicaClock.h"


namespace alica
{

	RunningPlan::RunningPlan()
	{
		this->to = AlicaEngine::getInstance()->getTeamObserver();
		this->ownId = to->getOwnId();
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
		if (this->plan == nullptr)
		{
			cerr << "Cannot Eval Condition, Plan is null" << endl;
			throw new exception();
		}
		if (this->plan->getPreCondition() == nullptr)
		{
			return true;
		}
		try
		{
			return this->plan->getPreCondition()->eval(this);
		}
		catch (exception & e)
		{
			cerr << "Exception in precondition: " << e.what() << endl;
			return false;
		}
	}

	bool RunningPlan::evalRuntimeCondition()
	{
		if (this->plan == nullptr)
		{
			cerr << "Cannot Eval Condition, Plan is null" << endl;
			throw new exception();
		}
		if (this->plan->getRuntimeCondition() == nullptr)
		{
			return true;
		}
		try
		{
			return this->plan->getRuntimeCondition()->eval(this);
		}
		catch (exception & e)
		{
			cerr << "Exception in runtimecondition: " << this->plan->getName() << e.what() << endl;
			return false;
		}
	}

	State* RunningPlan::getActiveState()
	{
		return this->activeState;
	}

	void RunningPlan::addChildren(shared_ptr<list<RunningPlan*> > runningPlans)
	{
		for (RunningPlan* r : (*runningPlans))
		{
			r->setParent(this);
			this->children.push_back(r);
			int f = 0;
			auto iter = this->failedSubPlans.find(r->plan);
			if (iter != this->failedSubPlans.end())
			{
				f = iter->second;
				r->failCount = f;
			}
			if (this->active)
			{
				r->activate();
			}
		}
	}

	void RunningPlan::moveState(State* nextState)
	{
		deactivateChildren();
		clearChildren();
		this->assignment->moveRobots(this->activeState, nextState);
		this->activeState = nextState;
		this->failedSubPlans.clear();
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
		if (failHandlingNeeded)
		{
			this->status = PlanStatus::Failed;
		}
		else
		{
			if (this->status == PlanStatus::Failed)
			{
				this->status = PlanStatus::Running;
			}
		}
		this->failHandlingNeeded = failHandlingNeeded;
	}

	void RunningPlan::setBehaviour(bool behaviour)
	{
		this->behaviour = behaviour;
	}

	list<RunningPlan*> RunningPlan::getChildren()
	{
		return this->children;
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
		if (this->plan != plan)
		{
			this->planStartTime = AlicaEngine::getInstance()->getIAlicaClock()->now();
			this->constraintStore->clear();
		}
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
		if (this->activeState != s)
		{
			this->activeState = s;
			this->stateStartTime = AlicaEngine::getInstance()->getIAlicaClock()->now();
			if (this->activeState != nullptr)
			{
				if (this->activeState->isFailureState())
				{
					this->status = PlanStatus::Failed;
				}
				else if (this->activeState->isSuccessState())
				{
					this->assignment->getEpSuccessMapping()->getRobots(this->activeEntryPoint)->push_back(this->ownId);
					this->to->getOwnEngineData()->getSuccessMarks()->markSuccessfull(this->plan,
																						this->activeEntryPoint);
				}
			}
		}
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
		if (this->basicBehaviour != nullptr)
		{
			if (this->basicBehaviour->isSuccess())
				return PlanStatus::Success;
			if (this->basicBehaviour->isFailure())
				return PlanStatus::Failed;
			return PlanStatus::Running;
		}
		if (this->assignment != nullptr && this->assignment->isSuccessfull())
			return PlanStatus::Success;
		return this->status;
	}
	PlanType* RunningPlan::getPlanType()
	{
		return planType;

	}

	void RunningPlan::clearFailures()
	{
		this->failCount = 0;
	}

	void RunningPlan::clearFailedChildren()
	{
		this->failedSubPlans.clear();
	}

	void RunningPlan::addFailure()
	{
		this->failCount++;
		this->failHandlingNeeded = true;
	}

	void RunningPlan::addChildren(list<RunningPlan*> children)
	{
	}

	int RunningPlan::getFailure()
	{
		return this->failCount;
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
		State* newState = r->getAssignment()->getRobotStateMapping()->getState(this->ownId);
		r->getAssignment()->getRobotStateMapping()->reconsiderOldAssignment(this->assignment, r->getAssignment());
		bool reactivate = false;

		if (this->activeState != newState)
		{
			this->active = false;
			this->deactivateChildren();
			this->revokeAllConstraints();
			this->clearChildren();
			this->addChildren(r->getChildren());
			reactivate = true;
		}
		else
		{
			auto robotsJoined = r->getAssignment()->getRobotStateMapping()->getRobotsInState(newState);
			for (RunningPlan* r : this->children)
			{
				r->limitToRobots(robotsJoined);
			}
		}

		this->plan = r->getPlan();
		this->activeEntryPoint = r->getOwnEntryPoint();
		this->assignment = r->assignment;
		this->activeState = newState;
		if (reactivate)
		{
			this->activate();
		}
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
		if (this->activeEntryPoint != activeEntryPoint)
		{
			this->assignment->removeRobot(ownId);
			this->activeEntryPoint = activeEntryPoint;
			if (this->activeEntryPoint != nullptr)
			{
				this->activeState = this->activeEntryPoint->getState();
				this->assignment->addRobot(ownId, this->activeEntryPoint, this->activeState);
			}

		}
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
		for (int i = 0; i < this->children.size(); i++)
		{
			auto iter = this->children.begin();
			advance(iter, i);
			if ((*iter)->isBehaviour())
			{
				if ((*iter)->getStatus() == PlanStatus::Success)
				{
					return true;
				}
			}
			else if ((*iter)->getActiveState() != nullptr && (*iter)->getActiveState()->isSuccessState())
			{
				return true;
			}
			for (shared_ptr<list<int> > successes : (*iter)->getAssignment()->getEpSuccessMapping()->getRobots())
			{
				if (find(successes->begin(), successes->end(), this->ownId) != successes->end())
				{
					return true;
				}
			}
		}
		return false;
	}

	bool RunningPlan::anyChildrenTaskFailure()
	{
		for (int i = 0; i < this->children.size(); i++)
		{
			auto iter = this->children.begin();
			advance(iter, i);
			if ((*iter)->getStatus() == PlanStatus::Failed)
			{
				return true;
			}
			if ((*iter)->getActiveState() != nullptr && (*iter)->getActiveState()->isFailureState())
			{
				return true;
			}
		}
		return false;
	}

	bool RunningPlan::anyChildrenTaskTerminated()
	{
		for (int i = 0; i < this->children.size(); i++)
		{
			auto iter = this->children.begin();
			advance(iter, i);
			if ((*iter)->isBehaviour())
			{
				if ((*iter)->getStatus() == PlanStatus::Failed || (*iter)->getStatus() == PlanStatus::Success)
				{
					return true;
				}
			}
			else if ((*iter)->getActiveState() != nullptr && (*iter)->getActiveState()->isTerminal())
			{
				return true;
			}
		}
		return false;
	}

	void RunningPlan::activate()
	{
		this->active = true;
		if (this->isBehaviour())
		{
			bp->startBehaviour(shared_from_this());
		}
		this->attachPlanConstraints();
		for (RunningPlan* r : this->children)
		{
			r->activate();
		}
	}

	void RunningPlan::limitToRobots(unordered_set<int> robots)
	{
		if (this->isBehaviour())
		{
			return;
		}
		if (!this->cycleManagement->mayDoUtilityCheck())
		{
			return;
		}
		bool recurse = false;
		auto curRobots = this->assignment->getAllRobots();
		for (int r : (*curRobots))
		{
			if (find(curRobots->begin(), curRobots->end(), r) == curRobots->end())
			{
				if (this->activeState != nullptr
						&& this->assignment->getRobotStateMapping()->stateOfRobot(r) == this->activeState)
				{
					recurse = true;
				}
				this->assignment->removeRobot(r);
			}
		}
		if (recurse)
		{
			for (RunningPlan* c : this->children)
			{
				c->limitToRobots(robots);
			}
		}
	}

	void RunningPlan::revokeAllConstraints()
	{
		this->constraintStore->clear();
	}

	void RunningPlan::attachPlanConstraints()
	{
		this->constraintStore->addCondition(this->plan->getPreCondition());
		this->constraintStore->addCondition(this->plan->getRuntimeCondition());
	}

	bool RunningPlan::recursiveUpdateAssignment(list<SimplePlanTree*> spts, list<int> availableAgents,
												list<int> noUpdates, unsigned long now)
	{
		if(this->isBehaviour())
		{
			return false;
		}

	}

	void RunningPlan::ToMessage(list<long> message, RunningPlan* deepestNode, int depth, int curDepth)
	{
		if (this->isBehaviour())
		{
			return;
		}
		if (this->activeState != nullptr)
		{
			message.push_back(this->activeState->getId());
		}
		else
		{
			return;
		}
		if (curDepth > depth)
		{
			depth = curDepth;
			deepestNode = this;
		}
		if (this->children.size() > 0)
		{
			message.push_back(-1);
			for (RunningPlan* r : this->children)
			{
				r->ToMessage(message, deepestNode, depth, curDepth + 1);
			}
			message.push_back(-2);
		}
	}

	string RunningPlan::toString()
	{
		stringstream ss;
		ss << "######## RP ##########" << endl;
		ss << "Plan: " + (plan != nullptr ? plan->getName() : "NULL") << endl;
		ss << "PlanType: " << (planType != nullptr ? planType->getName() : "NULL") << endl;
		ss << "ActState: " << (activeState != nullptr ? activeState->getName() : "NULL") << endl;
		ss << "Task: " << (this->getOwnEntryPoint() != nullptr ? this->getOwnEntryPoint()->getTask()->getName() : "NULL")
				<< endl;
		ss << "IsBehaviour: " << this->isBehaviour() << "\t";
		if (this->isBehaviour())
		{
			ss << "Behaviour: " << (this->basicBehaviour == nullptr ? "NULL" : this->basicBehaviour->getName()) << endl;
		}
		ss << "AllocNeeded: " << this->allocationNeeded << endl;
		ss << "FailHNeeded: " << this->failHandlingNeeded << "\t";
		ss << "FailCount: " << this->failCount << endl;
		ss << "IsActive: " << this->active << endl;
		ss << "Status: "
				<< (this->status == PlanStatus::Running ? "RUNNING" :
						(this->status == PlanStatus::Success ? "SUCCESS" : "FAILED")) << endl;
		ss << "AvailRobots: ";
		for (int r : (*this->robotsAvail))
		{
			ss << " " << r;
		}
		ss << "\n";
		if (this->assignment != nullptr)
		{
			ss << "Assignment:" << this->assignment->toString();
		}
		else
			ss << "Assignment is null." << endl;
		ss << "Children: " << this->children.size();
		if (this->children.size() > 0)
		{
			ss << " ( ";
			for (RunningPlan* r : this->children)
			{
				if (r->plan == nullptr)
				{
					ss << "NULL PLAN - ";
				}
				else
					ss << r->plan->getName() + " - ";
			}
			ss << ")";
		}
		ss << "\n########## ENDRP ###########" << endl;
		return ss.str();
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

