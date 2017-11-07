#pragma once

#include "engine/PlanStatus.h"
#include "engine/PlanChange.h"
#include "engine/IAlicaClock.h"
#include "supplementary/IAgentID.h"

#include <SystemConfig.h>

#include <map>
#include <list>
#include <memory>
#include <iostream>
#include <string>
#include <sstream>
#include <unordered_set>
#include <algorithm>

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
	class ConditionStore;
	class CycleManager;
	class BehaviourConfiguration;
	class IPlanTreeVisitor;
	class SimplePlanTree;
	class AlicaEngine;

	/**
	 * A RunningPlan represents a plan or a behaviour in execution, holding all information relevant at runtime.
	 */
	class RunningPlan : public enable_shared_from_this<RunningPlan>
	{
	public:
		RunningPlan(AlicaEngine* ae);
		RunningPlan(AlicaEngine* ae, Plan* plan);
		RunningPlan(AlicaEngine* ae, PlanType* pt);
		RunningPlan(AlicaEngine* ae, BehaviourConfiguration* bc);
		virtual ~RunningPlan();
		bool isBehaviour();
		void setBehaviour(bool behaviour);
		bool isAllocationNeeded();
		list<shared_ptr<RunningPlan>>* getChildren();
		void setChildren(list<shared_ptr<RunningPlan>> children);
		AbstractPlan* getPlan();
		void setPlan(AbstractPlan* plan);
		shared_ptr<BasicBehaviour> getBasicBehaviour();
		void setBasicBehaviour(shared_ptr<BasicBehaviour> basicBehaviour);
		shared_ptr<Assignment> getAssignment();
		void setAssignment(shared_ptr<Assignment> assignment);
		void printRecursive();
		AlicaTime getPlanStartTime();
		AlicaTime getStateStartTime();
		bool isActive();
		void setActive(bool active);
		void setRobotsAvail(unique_ptr<list<const supplementary::IAgentID*> > robots);
		void setAllocationNeeded(bool allocationNeeded);
		void setFailHandlingNeeded(bool failHandlingNeeded);
		void setOwnEntryPoint(EntryPoint* value);
		PlanChange tick(RuleBook* rules);
		std::shared_ptr<ConditionStore> getConstraintStore() const;
		EntryPoint* getOwnEntryPoint() const;
		void setParent(weak_ptr<RunningPlan> s);
		weak_ptr<RunningPlan> getParent() const;
		bool getFailHandlingNeeded() const;
		PlanStatus getStatus() const;
		PlanType* getPlanType();
		bool evalPreCondition();
		bool evalRuntimeCondition();
		State* getActiveState();
		void setActiveState(State* activeState);
		void addChildren(shared_ptr<list<shared_ptr<RunningPlan>>>& runningPlans);
                void addChildren(list<shared_ptr<RunningPlan>>& children);
		void moveState(State* nextState);
		void clearFailures();
		void clearFailedChildren();
		void addFailure();
		int getFailure();
		void deactivateChildren();
		void clearChildren();
		void adaptAssignment(shared_ptr<RunningPlan> r);
		void setFailedChild(AbstractPlan* child);
		void setRobotAvail(const supplementary::IAgentID* robot);
		void setRobotUnAvail(const supplementary::IAgentID* robot);
		void accept(IPlanTreeVisitor* vis);
		void deactivate();
		bool anyChildrenStatus(PlanStatus ps);
		bool allChildrenStatus(PlanStatus ps);
		bool anyChildrenTaskSuccess();
		void activate();
		EntryPoint* getActiveEntryPoint();
		void setActiveEntryPoint(EntryPoint* activeEntryPoint);
		void limitToRobots(unordered_set<const supplementary::IAgentID*, supplementary::IAgentIDHash, supplementary::IAgentIDEqualsComparator> robots);
		std::shared_ptr<CycleManager> getCycleManagement();
		void revokeAllConstraints();
		void attachPlanConstraints();
		bool recursiveUpdateAssignment(list<shared_ptr<SimplePlanTree> > spts, list<const supplementary::IAgentID*> availableAgents,list<const supplementary::IAgentID*> noUpdates, AlicaTime now);
		void toMessage(list<long>& message, shared_ptr<RunningPlan>& deepestNode, int& depth, int curDepth);
		string toString();
		const supplementary::IAgentID* getOwnID();
		AlicaEngine* getAlicaEngine();

		void sendLogMessage(int level, string& message);

	protected:
		AlicaEngine* ae;
		weak_ptr<RunningPlan> parent;
		bool behaviour;
		AbstractPlan* plan;
		shared_ptr<BasicBehaviour> basicBehaviour;
		list<shared_ptr<RunningPlan> > children;
		shared_ptr<Assignment> assignment;
		State* activeState;
		EntryPoint* activeEntryPoint;
		PlanStatus status;
		/**
		 * The (ROS-)timestamp referring to when the local robot entered the ActiveState.
		 */
		AlicaTime stateStartTime;
		/**
		 * The timestamp referring to when this plan was started by the local robot
		 */
		AlicaTime planStartTime;
		const supplementary::IAgentID* ownId;
		unique_ptr<list<const supplementary::IAgentID*> > robotsAvail;
		map<AbstractPlan*, int> failedSubPlans;
		PlanType* planType;
		int failCount;
		bool failHandlingNeeded;
		/**
		 * Whether or not this running plan is active or has been removed from the plan tree
		 */
		bool active;
		IBehaviourPool* bp;
		ITeamObserver* to;
		bool allocationNeeded;
		AlicaTime assignmentProtectionTime;
		std::shared_ptr<CycleManager> cycleManagement;
		std::shared_ptr<ConditionStore> constraintStore;
	};

}/* namespace alica */
