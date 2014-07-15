/*
 * RuleBook.h
 *
 *  Created on: Jun 17, 2014
 *      Author: Paul Panin
 */

#ifndef RULEBOOK_H_
#define RULEBOOK_H_

#include "engine/PlanChange.h"

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
		PlanChange visit(RunningPlan* r);
		PlanChange updateChange(PlanChange cur, PlanChange update);
		bool changeOccured;
		RunningPlan* initialisationRule(Plan* masterPlan);

	protected:
		ITeamObserver* to;
		ISyncModul* sm;
		int maxConsecutiveChanges;
		IPlanSelector* ps;
		Logger* log;
		PlanChange synchTransitionRule(RunningPlan* r);
		PlanChange transitionRule(RunningPlan* r);
		PlanChange topFailRule(RunningPlan* r);
		PlanChange allocationRule(RunningPlan* r);
		PlanChange authorityOverrideRule(RunningPlan* r);
		PlanChange planAbortRule(RunningPlan* r);
		PlanChange planRedoRule(RunningPlan* r);
		PlanChange planReplaceRule(RunningPlan* r);
		PlanChange planPropagationRule(RunningPlan* r);
		PlanChange dynamicAllocationRule(RunningPlan* r);
	};

#endif /* RULEBOOK_H_ */
}
