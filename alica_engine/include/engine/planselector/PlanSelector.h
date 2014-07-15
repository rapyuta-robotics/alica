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
#include "engine/IPlanSelector.h"


using namespace std;
namespace alica
{
	class RunningPlan;
	class AbstractPlan;

	class PlanSelector : public virtual IPlanSelector
	{
	public:
		PlanSelector();
		virtual ~PlanSelector();

		virtual RunningPlan* getBestSimilarAssignment(RunningPlan* rp);
		virtual RunningPlan* getBestSimilarAssignment(RunningPlan* rp, unordered_set<int> robots);
		virtual list<RunningPlan*> getPlansForState(RunningPlan* planningParent, list<AbstractPlan*> plans, unordered_set<int> robotIDs);
	};

} /* namespace alica */

#endif /* PLANSELECTOR_H_ */
