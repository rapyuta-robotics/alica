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
#include "engine/SimplePlanTree.h"
#include "engine/allocationauthority/EntryPointRobotPair.h"

namespace alica
{

	RunningPlan::RunningPlan()
	{
		this->planType = nullptr;
		this->plan = nullptr;
		this->bp = nullptr;
		this->activeState = nullptr;
		this->activeEntryPoint = nullptr;
		this->behaviour = nullptr;
		this->planStartTime = 0;
		this->stateStartTime = 0;
		this->assignment = nullptr;
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
		this->robotsAvail = unique_ptr<list<int> >(new list<int>);
	}

	RunningPlan::~RunningPlan()
	{
		this->basicBehaviour.reset();
		delete cycleManagement;
		delete constraintStore;
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

	/**
	 * Indicates whether this plan needs failure handling
	 */
	bool RunningPlan::getFailHandlingNeeded() const
	{
		return this->failHandlingNeeded;
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

	/**
	 * Gets/Sets the parent RunningPlan of this RunningPlan. Null in case this is the top-level element.
	 */
	void RunningPlan::setParent(weak_ptr<RunningPlan> s)
	{
		this->parent = s;
	}
	weak_ptr<RunningPlan> RunningPlan::getParent() const
	{
		return this->parent;
	}

	/**
	 * Called once per Engine iteration, performs all neccessary checks and executes rules from the rulebook.
	 * @param rules a RuleBook
	 * @return PlanChange a PlanChange
	 */
	PlanChange RunningPlan::tick(RuleBook* rules)
	{

		this->cycleManagement->update();
		PlanChange myChange = rules->visit(shared_from_this());

		PlanChange childChange = PlanChange::NoChange;
		for (shared_ptr<RunningPlan> rp : this->children)
		{
			childChange = rules->updateChange(childChange, rp->tick(rules));
		}
		if (childChange != PlanChange::NoChange && childChange != PlanChange::InternalChange)
		{
			myChange = rules->updateChange(myChange, rules->visit(shared_from_this()));
		}

		return myChange;
	}

	/**
	 * Indicates whether an allocation is needed in the RunningPlan.ActiveState.
	 * If set to true, the next engine iteration will perform a task allocation and set it to false.
	 * true if allocation is needed, otherwise false
	 */
	bool RunningPlan::isAllocationNeeded()
	{
		return this->allocationNeeded;
	}
	void RunningPlan::setAllocationNeeded(bool need)
	{
		this->allocationNeeded = need;
	}

	/**
	 * Evaluates the precondition of the associated plan.
	 * @return Whether the precondition currently holds or not.
	 */
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
			return this->plan->getPreCondition()->evaluate(shared_from_this());
		}
		catch (exception & e)
		{
			cerr << "Exception in precondition: " << e.what() << endl;
			return false;
		}
	}

	/**
	 * Evals the runtime condition of the associated plan.
	 * @return Whether the runtime currently holds or not.
	 */
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
			return this->plan->getRuntimeCondition()->evaluate(shared_from_this());
		}
		catch (exception & e)
		{
			cerr << "Exception in runtimecondition: " << this->plan->getName() << e.what() << endl;
			return false;
		}
	}

	/**
	 * Gets the state currently inhabited by the local agent. Null if none exists.
	 */
	State* RunningPlan::getActiveState()
	{
		return this->activeState;
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

	void RunningPlan::addChildren(shared_ptr<list<shared_ptr<RunningPlan>>> runningPlans)
	{
		for (shared_ptr<RunningPlan> r : (*runningPlans))
		{
			r->setParent(shared_from_this());
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
				cout << "RP: active " << endl;
				r->activate();
			}
		}
	}

	/**
	 * Move this very robot to another state. Performs all neccessary operations, such as updating the assignment.
	 * @param nextState A State
	 */
	void RunningPlan::moveState(State* nextState)
	{
		deactivateChildren();
		clearChildren();
		this->assignment->moveRobots(this->activeState, nextState);
		this->activeState = nextState;
		this->failedSubPlans.clear();
	}

	/**
	 * Gets/Sets the constraint store, which contains all constrains associated with this RunningPlan.
	 */
	ConstraintStore* RunningPlan::getConstraintStore() const
	{
		return this->constraintStore;
	}
	void RunningPlan::setConstraintStore(ConstraintStore* constraintStore)
	{
		this->constraintStore = constraintStore;
	}

	/**
	 * Indicates whether this running plan represents a behaviour.
	 * true if this instance is representing a behaviour; otherwise, false.
	 */
	bool RunningPlan::isBehaviour()
	{
		return behaviour;
	}
	void RunningPlan::setBehaviour(bool behaviour)
	{
		this->behaviour = behaviour;
	}

	/**
	 * The children of this RunningPlan.
	 */
	list<shared_ptr<RunningPlan>>& RunningPlan::getChildren()
	{
		return this->children;
	}
	void RunningPlan::setChildren(list<shared_ptr<RunningPlan>> children)
	{
		this->children = children;
	}

	/**
	 * The abstract plan associated with this running plan, a model element.
	 */
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

	/**
	 * The behaviour represented by this running plan, in case there is any, otherwise null.
	 */
	shared_ptr<BasicBehaviour> RunningPlan::getBasicBehaviour()
	{
		return this->basicBehaviour;
	}
	void RunningPlan::setBasicBehaviour(shared_ptr<BasicBehaviour> basicBehaviour)
	{
		this->basicBehaviour = basicBehaviour;
	}

	/**
	 * Simple method to recursively print the plan-tree.
	 */
	void RunningPlan::printRecursive()
	{
		for (shared_ptr<RunningPlan> c : this->children)
		{
			c->printRecursive();
		}
		if (this->children.size() > 0)
		{
			cout << "END CHILDREN of " << (this->plan == nullptr ? "NULL" : this->plan->getName()) << endl;
		}
	}

	/**
	 * The current assignment of robots to EntryPoints.
	 */
	shared_ptr<Assignment> RunningPlan::getAssignment()
	{
		return assignment;
	}
	void RunningPlan::setAssignment(shared_ptr<Assignment> assignment)
	{
		this->assignment = assignment;
	}

	alicaTime RunningPlan::getPlanStartTime()
	{
		return planStartTime;
	}

	alicaTime RunningPlan::getStateStartTime()
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

	/**
	 * Sets the set of robots currently participating in this plan.
	 */
	void RunningPlan::setRobotsAvail(unique_ptr<list<int> > robots)
	{
		this->robotsAvail->clear();
		this->robotsAvail = move(robots);
	}

	/**
	 * The robot's current EntryPoint. Null if it is idling
	 */
	EntryPoint* RunningPlan::getOwnEntryPoint() const
	{
		return this->activeEntryPoint;
	}

	/**
	 * The robot's current EntryPoint. Null if it is idling
	 */
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

	/**
	 * Gets the PlanStatus of the currently executed plan.
	 */
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

	/**
	 * Gets the PlanType of the currently executed plan. Null if the AbstractPlan associated does not belong to a PlanType.
	 */
	PlanType* RunningPlan::getPlanType()
	{
		return planType;

	}

	void RunningPlan::clearFailures()
	{
		this->failCount = 0;
	}

	/**
	 * Clears the failure history of failed plans.
	 */
	void RunningPlan::clearFailedChildren()
	{
		this->failedSubPlans.clear();
	}

	void RunningPlan::addFailure()
	{
		this->failCount++;
		this->failHandlingNeeded = true;
	}

	void RunningPlan::addChildren(list<shared_ptr<RunningPlan>>& children)
	{
		for (shared_ptr<RunningPlan> r : children)
		{
			r->setParent(shared_from_this());
			this->children.push_back(r);
			int f = 0;
			auto iter = this->failedSubPlans.find(r->getPlan());
			if (iter != this->failedSubPlans.end())
			{
				r->failCount = iter->second;
			}
			if (this->active)
			{
				r->activate();
			}
		}
	}

	/**
	 * Returns the number of failures detected while this RunningPlan was executed.
	 */
	int RunningPlan::getFailure()
	{
		return this->failCount;
	}

	void RunningPlan::deactivateChildren()
	{
		for (shared_ptr<RunningPlan> r : this->children)
		{
			r->deactivate();
		}
	}

	/**
	 * Remove all children without passing any command to them.
	 */
	void RunningPlan::clearChildren()
	{
		this->children.clear();
	}

	/**
	 * Adapt the assignment of this plan to the one supplied. This can also change plan
	 * @param r A RunningPlan
	 */
	void RunningPlan::adaptAssignment(shared_ptr<RunningPlan> r)
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
			for (shared_ptr<RunningPlan> r : this->children)
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

	/**
	 * Returns all robots currently participating in this plan.
	 */
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

	/**
	 * Indicates whether this running plan represents a behaviour.
	 */
	CycleManager* RunningPlan::getCycleManagement()
	{
		return cycleManagement;
	}

	void RunningPlan::setCycleManagement(CycleManager* cycleManagement)
	{
		this->cycleManagement = cycleManagement;
	}

	/**
	 * Indicate that an AbstractPlan has failed while being a child of this plan.
	 * @param child a AbstractPlan
	 */
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

	/**
	 *  General Visitor pattern for the plan graph.
	 *  @param vis A IPlanTreeVisitor
	 */
	void RunningPlan::accept(IPlanTreeVisitor* vis)
	{
		vis->visit(shared_from_this());
		for (int i = 0; i < this->children.size(); i++)
		{
			auto iter = this->children.begin();
			advance(iter, i);
			(*iter)->accept(vis);
		}
	}

	/**
	 *  Deactivate this plan, to be called before the plan is removed from the graph.
	 * Ensures that all sub-behaviours are stopped and all constraints are revoked.
	 */
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
		revokeAllConstraints();
		deactivateChildren();
	}

	/**
	 * Tests whether all child has a specific status.
	 * @param A PlanStatus
	 * @returns bool
	 */
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

	/**
	 * Tests whether for any child, the robot completed a task
	 * @return bool
	 */
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

	/**
	 * Tests whether for any child, the robot failed a task
	 * @returns bool
	 */
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

	/**
	 * Tests whether for any child, the robot reached a terminal state
	 * @return bool
	 */
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

	/**
	 * Activate this plan, called when it is inserted into the plan graph.
	 */
	void RunningPlan::activate()
	{
		this->active = true;
		if (this->isBehaviour())
		{
			bp->startBehaviour(shared_from_this());
		}
		this->attachPlanConstraints();
		for (shared_ptr<RunningPlan> r : this->children)
		{
			r->activate();
		}
	}

	/**
	 * Removes any robot not in robots
	 * @param robots The set of robots that can participate in this running plan.
	 */
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
			for (shared_ptr<RunningPlan> c : this->children)
			{
				c->limitToRobots(robots);
			}
		}
	}

	/**
	 * //convenience method as recursive case might have been called for the children already
	 */
	void RunningPlan::revokeAllConstraints()
	{
		this->constraintStore->clear();
	}

	void RunningPlan::attachPlanConstraints()
	{
		this->constraintStore->addCondition(this->plan->getPreCondition());
		this->constraintStore->addCondition(this->plan->getRuntimeCondition());
	}

	bool RunningPlan::recursiveUpdateAssignment(list<shared_ptr<SimplePlanTree> > spts, vector<int> availableAgents,
												list<int> noUpdates, alicaTime now)
	{
		if (this->isBehaviour())
		{
			return false;
		}
		bool keepTask = (this->planStartTime + assignmentProtectionTime > now);
		bool auth = this->cycleManagement->haveAuthority();

		//if keepTask, the task Assignment should not be changed!
		bool ret = false;
		AllocationDifference* aldif = new AllocationDifference();
		for (shared_ptr<SimplePlanTree> spt : spts)
		{
			if (spt->getState()->getInPlan() != this->plan)
			{ //the robot is no longer participating in this plan
				if (!keepTask & !auth)
				{
					EntryPoint* ep = this->getAssignment()->entryPointOfRobot(spt->getRobotId());
					if (ep != nullptr)
					{
						this->getAssignment()->removeRobot(spt->getRobotId());
						ret = true;
						aldif->getSubtractions().push_back(new EntryPointRobotPair(ep, spt->getRobotId()));
					}
				}
			}
			else
			{
				if (keepTask || auth)
				{ //Update only state, and that only if it is in the reachablity graph of its current entrypoint, else ignore
					EntryPoint* cep = this->getAssignment()->entryPointOfRobot(spt->getRobotId());
					if (cep != nullptr)
					{
						if (cep->getReachableStates().find(spt->getState()) != cep->getReachableStates().end())
						{
							this->getAssignment()->getRobotStateMapping()->setState(spt->getRobotId(), spt->getState());
						}
					}
					else
					{ //robot was not expected to be here during protected assignment time, add it.
						this->getAssignment()->addRobot(spt->getRobotId(), spt->getEntryPoint(), spt->getState());
						aldif->getAdditions().push_back(new EntryPointRobotPair(spt->getEntryPoint(), spt->getRobotId()));

					}
				}
				else
				{ //Normal Update
					EntryPoint* ep = this->getAssignment()->entryPointOfRobot(spt->getRobotId());
					ret |= this->getAssignment()->updateRobot(spt->getRobotId(), spt->getEntryPoint(), spt->getState());
					if (spt->getEntryPoint() != ep)
					{
						aldif->getAdditions().push_back(new EntryPointRobotPair(spt->getEntryPoint(), spt->getRobotId()));
						if (ep != nullptr)
							aldif->getSubtractions().push_back(new EntryPointRobotPair(ep, spt->getRobotId()));
					}

				}
			}
		}
		auto eps = this->getAssignment()->getEntryPoints();
		list<int> rem = list<int>();
		if (!keepTask)
		{ //remove any robot no longer available in the spts (auth flag obey here, as robot might be unavailable)
		  //EntryPoint[] eps = this.Assignment.GetEntryPoints();

			for (int i = 0; i < eps->size(); i++)
			{
				rem.clear();
				auto robs = this->getAssignment()->getRobotsWorking(eps->at(i));
				for (int rob : (*robs))
				{
					if (rob == ownId)
						continue;
					bool found = false;
					if (find(noUpdates.begin(), noUpdates.end(), rob) != noUpdates.end())
					{
						//found = true;
						continue;
					}
					for (shared_ptr<SimplePlanTree> spt : spts)
					{
						if (spt->getRobotId() == rob)
						{
							found = true;
							break;
						}
					}
					if (!found)
					{
						rem.push_back(rob);
						//this.Assignment.RemoveRobot(rob);
						aldif->getSubtractions().push_back(new EntryPointRobotPair(eps->at(i), rob));
						ret = true;
					}
				}
				for (int rob : rem)
				{
					this->getAssignment()->removeRobot(rob);
				}
			}
		}

		//enforce consistency between RA and PlanTree by removing robots deemed inactive:
		if (!auth)
		{ //under authority do not remove robots from assignment
			for (int i = 0; i < eps->size(); i++)
			{
				rem.clear();
				auto robs = this->getAssignment()->getRobotsWorking(eps->at(i));
				for (int rob : (*robs))
				{
					//if (rob==ownId) continue;
					if (find(availableAgents.begin(), availableAgents.end(), rob) == availableAgents.end())
					{
						rem.push_back(rob);
						//this.Assignment.RemoveRobot(rob);
						aldif->getSubtractions().push_back(new EntryPointRobotPair(eps->at(i), rob));
						ret = true;
					}
				}

				for (int rob : rem)
				{
					this->getAssignment()->removeRobot(rob);
				}
			}
		}

		aldif->setReason(AllocationDifference::Reason::message);
		this->cycleManagement->setNewAllocDiff(this, aldif);
//Update Success Collection:
		this->to->updateSuccessCollection((Plan*)this->getPlan(), this->getAssignment()->getEpSuccessMapping());

//If Assignment Protection Time for newly started plans is over, limit available robots to those in this active state.
		if (this->stateStartTime + assignmentProtectionTime > now)
		{
			auto robotsJoined = this->getAssignment()->getRobotStateMapping()->getRobotsInState(this->getActiveState());
			for (int i = 0; i < availableAgents.size(); i++)
			{
				if (find(robotsJoined.begin(), robotsJoined.end(), availableAgents[i]) == robotsJoined.end())
				{
					availableAgents.erase(availableAgents.begin() + i);
					i--;
				}
			}
		}
		else if (auth)
		{ // in case of authority, remove all that are not assigned to same task
			auto robotsJoined = this->getAssignment()->getRobotsWorking(this->getOwnEntryPoint());
			for (int i = 0; i < availableAgents.size(); i++)
			{
				if (find(robotsJoined->begin(), robotsJoined->end(), availableAgents[i]) == robotsJoined->end())
				{
					availableAgents.erase(availableAgents.begin() + i);
					i--;
				}
			}
		}
//Give Plans to children
		for (shared_ptr<RunningPlan> r : this->children)
		{
			if (r->isBehaviour())
			{
				continue;
			}
			list<shared_ptr<SimplePlanTree> > newcspts = list<shared_ptr<SimplePlanTree> >();
			for(shared_ptr<SimplePlanTree> spt : spts)
			{
				if(spt->getState() == this->activeState)
				{
					for(shared_ptr<SimplePlanTree> cspt : spt->getChildren())
					{
						if(cspt->getState()->getInPlan() == r->getPlan())
						{
							newcspts.push_back(cspt);
							break;
						}
					}
				}
			}
			ret |= r->recursiveUpdateAssignment(newcspts, availableAgents, noUpdates, now);
		}
		return ret;

	}

	void RunningPlan::toMessage(list<long>& message, shared_ptr<RunningPlan>& deepestNode, int& depth, int curDepth)
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
			deepestNode = shared_from_this();
		}
		if (this->children.size() > 0)
		{
			message.push_back(-1);
			for (shared_ptr<RunningPlan> r : this->children)
			{
				r->toMessage(message, deepestNode, depth, curDepth + 1);
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
		ss << "Task: "
				<< (this->getOwnEntryPoint() != nullptr ? this->getOwnEntryPoint()->getTask()->getName() : "NULL")
				<< endl;
		ss << "IsBehaviour: " << this->isBehaviour() << "\t";
		if (this->isBehaviour())
		{
			ss << "Behaviour: " << (this->basicBehaviour == nullptr ? "NULL" : this->basicBehaviour->getName()) << endl;
		}
		ss << "AllocNeeded: " << this->allocationNeeded << endl;
		ss << "FailHandlingNeeded: " << this->failHandlingNeeded << "\t";
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
			for (shared_ptr<RunningPlan> r : this->children)
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

	/**
	 * Tests whether any child has a specific status.
	 * @param A PlanStatus
	 * @return bool
	 */
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

	int RunningPlan::getOwnID() {
		return this->ownId;
	}

} /* namespace alica */

