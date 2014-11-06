/*
 * PlanSelector.h
 *
 *  Created on: Jul 4, 2014
 *      Author: Stefan Jakob
 */

#ifndef PLANSELECTOR_H_
#define PLANSELECTOR_H_

#define PSDEBUG

#include <vector>
#include <list>
#include <unordered_set>
#include <memory>
#include <sstream>

#include "engine/IPlanSelector.h"

using namespace std;

namespace alica
{
	class RunningPlan;
	class AbstractPlan;
	class ITeamObserver;
	class RunningPlan;
	class AbstractPlan;
	class PlanType;
	class Plan;
	class AlicaEngine;
	class PartialAssignmentPool;

	/**
	 * Implements the task allocation algorithm
	 */
	class PlanSelector : public virtual IPlanSelector
	{
	public:
		PlanSelector(AlicaEngine* ae, PartialAssignmentPool* pap);
		virtual ~PlanSelector();

		virtual shared_ptr<RunningPlan> getBestSimilarAssignment(shared_ptr<RunningPlan> rp);
		virtual shared_ptr<RunningPlan> getBestSimilarAssignment(shared_ptr<RunningPlan> rp, shared_ptr<vector<int> > robots);
		virtual shared_ptr<list<shared_ptr<RunningPlan>> > getPlansForState(shared_ptr<RunningPlan>planningParent, list<alica::AbstractPlan*>* plans, shared_ptr<vector<int> > robotIDs);
		shared_ptr<RunningPlan> createRunningPlan(weak_ptr<RunningPlan> planningParent, list<Plan*> plans, shared_ptr<vector<int> >  robotIDs, shared_ptr<RunningPlan> oldRp, PlanType* relevantPlanType);

	private:
		PartialAssignmentPool* pap;
		ITeamObserver* to;
		AlicaEngine* ae;
		shared_ptr<list<shared_ptr<RunningPlan>> > getPlansForStateInternal(shared_ptr<RunningPlan> planningParent, list<alica::AbstractPlan*>* plans, shared_ptr<vector<int> > robotIDs);
	};

} /* namespace alica */

#endif /* PLANSELECTOR_H_ */
