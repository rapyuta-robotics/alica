/*
 * PlanSelector.h
 *
 *  Created on: Jul 4, 2014
 *      Author: Stefan Jakob
 */

#ifndef PLANSELECTOR_H_
#define PLANSELECTOR_H_

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

	class PlanSelector : public virtual IPlanSelector
	{
	public:
		PlanSelector();
		virtual ~PlanSelector();

		virtual shared_ptr<RunningPlan> getBestSimilarAssignment(shared_ptr<RunningPlan> rp);
//		virtual RunningPlan* getBestSimilarAssignment(RunningPlan* rp, unordered_set<int> robots);
//		virtual list<RunningPlan*> getPlansForState(RunningPlan* planningParent, list<AbstractPlan*> plans, unordered_set<int> robotIDs);
		virtual shared_ptr<RunningPlan> getBestSimilarAssignment(shared_ptr<RunningPlan> rp, shared_ptr<vector<int> > robots);
		virtual shared_ptr<list<shared_ptr<RunningPlan>> > getPlansForState(shared_ptr<RunningPlan>planningParent, list<alica::AbstractPlan*>* plans, shared_ptr<vector<int> > robotIDs);
		shared_ptr<RunningPlan> createRunningPlan(weak_ptr<RunningPlan> planningParent, list<Plan*> plans, shared_ptr<vector<int> >  robotIDs, shared_ptr<RunningPlan> oldRp, PlanType* relevantPlanType);

	private:
		ITeamObserver* to;
		shared_ptr<list<shared_ptr<RunningPlan>> > getPlansForStateInternal(shared_ptr<RunningPlan> planningParent, list<alica::AbstractPlan*>* plans, shared_ptr<vector<int> > robotIDs);
	};

} /* namespace alica */

#endif /* PLANSELECTOR_H_ */
