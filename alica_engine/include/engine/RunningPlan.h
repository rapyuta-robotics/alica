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

	class RunningPlan
	{
	public:
		RunningPlan();
		virtual ~RunningPlan();
		bool isBehaviour() const;
		enum planChange{NoCHange, InternalChange, SuccessChange, FailChange};
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
		list<int> robotsAvail;
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
