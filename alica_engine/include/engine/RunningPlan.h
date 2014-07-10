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

	class RunningPlan
	{
	public:
		/**
		 * Captures the result of a rule application.
		 */
		enum PlanChange {
			NoChange,      //!< NoChange occurred, rule was not applicable
			InternalChange,//!< InternalChange, change occurred but is of no interest to upper level plans
			SuccesChange,  //!< SuccesChange, change occurred and led to a success, upper level can react
			FailChange     //!< FailChange, change occurred and led to a failure, upper level should react
		};
		RunningPlan();
		RunningPlan(Plan* plan);
		virtual ~RunningPlan();
		bool isBehaviour() const;
		void setBehaviour(bool behaviour);
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
		unsigned long getStateStartTime();
		bool isActive();
		void setActive(bool active);
		void setRobotsAvail(unique_ptr<list<int> >  robots);
		void setAllocationNeeded(bool allocationNeeded);
		void setOwnEntryPoint(EntryPoint* value);
		PlanChange tick(RuleBook* rules);

	protected:
		bool behaviour;
		AbstractPlan* plan;
		shared_ptr<BasicBehaviour> basicBehaviour;
		list<RunningPlan*> children;
		Assignment* assignment;
		RunningPlan* parent;
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

};

} /* namespace alica */

#endif /* RUNNINGPLAN_H_ */
