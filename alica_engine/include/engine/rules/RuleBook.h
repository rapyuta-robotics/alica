/*
 * RuleBook.h
 *
 *  Created on: Jun 17, 2014
 *      Author: Paul Panin
 */

#ifndef RULEBOOK_H_
#define RULEBOOK_H_
#define RULE_debug

using namespace std;

#include "engine/PlanChange.h"
#include <memory>

namespace alica
{
	class ISyncModul;
	class IPlanSelector;
	class Logger;
	class ITeamObserver;
	class RunningPlan;
	class Plan;
	class EntryPoint;
	class Transition;
	class State;
	class EntryPoint;
	class ConstraintStore;
	class StateCollection;
	class CycleManager;
	class UtilityFunction;

	class RuleBook
	{
	public:
		RuleBook();
		virtual ~RuleBook();
		bool isChangeOccured() const;
		void setChangeOccured(bool changeOccured);
		PlanChange visit(shared_ptr<RunningPlan> r);
		PlanChange updateChange(PlanChange cur, PlanChange update);
		shared_ptr<RunningPlan> initialisationRule(Plan* masterPlan);

	protected:
		ITeamObserver* to;
		ISyncModul* sm;
		int maxConsecutiveChanges;
		IPlanSelector* ps;
		Logger* log;
		bool changeOccured;
		PlanChange synchTransitionRule(shared_ptr<RunningPlan> r);
		PlanChange transitionRule(shared_ptr<RunningPlan> r);
		PlanChange topFailRule(shared_ptr<RunningPlan> r);
		PlanChange allocationRule(shared_ptr<RunningPlan> r);
		PlanChange authorityOverrideRule(shared_ptr<RunningPlan> r);
		PlanChange planAbortRule(shared_ptr<RunningPlan> r);
		PlanChange planRedoRule(shared_ptr<RunningPlan> r);
		PlanChange planReplaceRule(shared_ptr<RunningPlan> r);
		PlanChange planPropagationRule(shared_ptr<RunningPlan> r);
		PlanChange dynamicAllocationRule(shared_ptr<RunningPlan> r);
	};

#endif /* RULEBOOK_H_ */
}
