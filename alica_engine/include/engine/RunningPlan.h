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
#include <string>
#include <sstream>
#include <unordered_set>
#include <algorithm>
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
	class BehaviourConfiguration;
	class IPlanTreeVisitor;
	class SimplePlanTree;

	class RunningPlan : public enable_shared_from_this<RunningPlan>
	{
	public:
		RunningPlan();
		RunningPlan(Plan* plan);
		RunningPlan(PlanType* pt);
		RunningPlan(BehaviourConfiguration* bc);
		virtual ~RunningPlan();
		bool isBehaviour();
		void setBehaviour(bool behaviour);
		bool isAllocationNeeded();
		list<RunningPlan*>* getChildren();
		void setChildren(list<RunningPlan*>* children);
		AbstractPlan* getPlan();
		void setPlan(AbstractPlan* plan);
		shared_ptr<BasicBehaviour> getBasicBehaviour();
		void setBasicBehaviour(shared_ptr<BasicBehaviour> basicBehaviour);
		Assignment* getAssignment();
		void setAssignment(Assignment* assignment);
		void printRecursive();
		unsigned long getPlanStartTime();
		unsigned long getStateStartTime();
		bool isActive();
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
		void setParent(RunningPlan* s);
		RunningPlan* getParent() const;
		bool getFailHandlingNeeded() const;
		PlanStatus getStatus() const;
		PlanType* getPlanType();
		bool evalPreCondition();
		bool evalRuntimeCondition();
		State* getActiveState();
		void setActiveState(State* activeState);
		void addChildren(shared_ptr<list<RunningPlan*> > runningPlans);
		void moveState(State* nextState);
		void clearFailures();
		void clearFailedChildren();
		void addFailure();
		void addChildren(list<RunningPlan*>* children);
		int getFailure();
		void deactivateChildren();
		void clearChildren();
		void setFailedChildren(AbstractPlan* p);
		void adaptAssignment(RunningPlan* r);
		void setFailedChild(AbstractPlan* child);
		unique_ptr<list<int> > getRobotsAvail();
		void setRobotAvail(int robot);
		void setRobotUnAvail(int robot);
		void accept(IPlanTreeVisitor* vis);
		void deactivate();
		bool anyChildrenStatus(PlanStatus ps);
		bool allChildrenStatus(PlanStatus ps);
		bool anyChildrenTaskSuccess();
		bool anyChildrenTaskFailure();
		bool anyChildrenTaskTerminated();
		void activate();
		EntryPoint* getActiveEntryPoint();
		void setActiveEntryPoint(EntryPoint* activeEntryPoint);
		void limitToRobots(unordered_set<int> robots);
		CycleManager* getCycleManagement();
		void setCycleManagement(CycleManager* cycleManagement);
		void revokeAllConstraints();
		void attachPlanConstraints();
		bool recursiveUpdateAssignment(list<SimplePlanTree*> spts, list<int> availableAgents,list<int> noUpdates, unsigned long now);
		void ToMessage(list<long> message, RunningPlan* deepestNode, int depth, int curDepth);
		string toString();

	private:
		void setConstraintStore(ConstraintStore* constraintStore);

	protected:
		RunningPlan* parent;
		bool behaviour;
		AbstractPlan* plan;
		shared_ptr<BasicBehaviour> basicBehaviour;
		list<RunningPlan*>* children;
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
		ITeamObserver* to;bool allocationNeeded;
		unsigned long assignmentProtectionTime = (((*supplementary::SystemConfig::getInstance())["Alica"]->get<
				unsigned long>("Alica.AssignmentProtectionTime", NULL)) * 1000000);
		CycleManager* cycleManagement;
		ConstraintStore* constraintStore;
	};

}
/* namespace alica */

#endif /* RUNNINGPLAN_H_ */
