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
#include <memory>
#include <sstream>
#include "engine/IPlanSelector.h"

using namespace std;

namespace alica
{
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

		virtual RunningPlan* getBestSimilarAssignment(RunningPlan* rp);
		virtual RunningPlan* getBestSimilarAssignment(RunningPlan* rp, shared_ptr<vector<int> > robots);
		virtual shared_ptr<list<RunningPlan*> > getPlansForState(RunningPlan* planningParent, list<AbstractPlan*> plans, shared_ptr<vector<int> > robotIDs);
		RunningPlan* createRunningPlan(RunningPlan* planningParent, list<Plan*> plans, shared_ptr<vector<int> >  robotIDs, RunningPlan* oldRp, PlanType* relevantPlanType);

	private:
		ITeamObserver* to;
		shared_ptr<list<RunningPlan*> > getPlansForStateInternal(RunningPlan* planningParent, list<AbstractPlan*> plans, shared_ptr<vector<int> > robotIDs);
	};

} /* namespace alica */

#endif /* PLANSELECTOR_H_ */
