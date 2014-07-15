/*
 * RunningPlan.h
 *
 *  Created on: Jun 10, 2014
 *      Author: Stefan Jakob
 */

#ifndef RUNNINGPLAN_H_
#define RUNNINGPLAN_H_

using namespace std;

#include <map>
#include <list>
#include <memory>
#include <iostream>
#include <SystemConfig.h>

#include "engine/PlanStatus.h"
#include "engine/PlanChange.h"

using namespace std;

namespace alica
{

	class BasicBehaviour;
	class AbstractPlan;
	class Assignment;
	class State;
	class EntryPoint;
	class PlanType;
	class IBehaviourPool;
	class ITeamObserver;
	class Plan;
	class RuleBook;
	class ConstraintStore;
	class CycleManager;

	class RunningPlan
	{
	public:
		RunningPlan();
		RunningPlan(Plan* plan);
		virtual ~RunningPlan();
		bool isBehaviour() const;
		void setBehaviour(bool behaviour);
		bool isAllocationNeeded() const;
		const list<RunningPlan*>& getChildren() const;
		void setChildren(const list<RunningPlan*>& children);
		AbstractPlan* getPlan() const;
		void setPlan(AbstractPlan* plan);
		shared_ptr<BasicBehaviour> getBasicBehaviour();
		void setBasicBehaviour(shared_ptr<BasicBehaviour> basicBehaviour);
		Assignment* getAssignment();
		void setAssignment(Assignment* assignment);
		void printRecursive();
		unsigned long getPlanStartTime();
		unsigned long getStateStartTime();bool isActive();
		void setActive(bool active);
		void setRobotsAvail(unique_ptr<list<int> > robots);
		void setAllocationNeeded(bool allocationNeeded);
		void setFailHandlingNeeded(bool failHandlingNeeded);
		void setOwnEntryPoint(EntryPoint* value);
		PlanChange tick(RuleBook* rules);
		State* getActiveState() const;
		CycleManager* getCycleManager() const;
		ConstraintStore* getConstraintStore() const;
		EntryPoint* getOwnEntryPoint() const;
		void setActiveState(State* s);
		void setParent(RunningPlan* s);
		RunningPlan* getParent() const;bool getFailHandlingNeeded() const;
		PlanStatus getStatus() const;

		//TODO: Nicht implementiert
		void moveState(State* nextState);
		void clearFailures();
		void clearFailedChildren();
		void addFailure();
		void addChildren(list<RunningPlan*> children);
		bool evalRuntimeCondition();
		int getFailure();
		void deactivateChildren(bool allAreLeaving);
		void clearChildren();
		void setFailedChildren(AbstractPlan* p);
		void adaptAssignment(RunningPlan* r);

	protected:
		RunningPlan* parent;
		bool behaviour;
		AbstractPlan* plan;
		shared_ptr<BasicBehaviour> basicBehaviour;
		list<RunningPlan*> children;
		Assignment* assignment;
		State* activeState;
		EntryPoint* activeEntryPoint;
		PlanStatus status;
		unsigned long stateStartTime;
		unsigned long planStartTime;
		int ownId;
		unique_ptr<list<int> > robotsAvail;
		map<AbstractPlan*, int> failedSubPlans;
		PlanType* planType;
		int failCount;
		bool failHandlingNeeded;
		bool active;
		IBehaviourPool* bp;
		ITeamObserver* to;
		bool allocationNeeded;
		unsigned long assignmentProtectionTime = (((*supplementary::SystemConfig::getInstance())["Alica"]->get<unsigned long>("Alica.AssignmentProtectionTime")) * 1000000);
		CycleManager* cycleManagement;
		ConstraintStore* constraintStore;
	};

} /* namespace alica */

#endif /* RUNNINGPLAN_H_ */
