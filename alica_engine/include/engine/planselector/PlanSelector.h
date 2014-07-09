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
#include "engine/IPlanSelector.h"
#include "engine/RunningPlan.h"
#include "engine/model/AbstractPlan.h"

using namespace std;
namespace alica
{
	class PlanSelector : public virtual IPlanSelector
	{
	public:
		PlanSelector();
		virtual ~PlanSelector();

		virtual RunningPlan* getBestSimilarAssignment(RunningPlan* rp);
		virtual RunningPlan* getBestSimilarAssignment(RunningPlan* rp, vector<int> robots);
		virtual list<RunningPlan*> getPlansForState(RunningPlan* planningParent, list<AbstractPlan*> plans, vector<int> robotIDs);
	};

} /* namespace alica */

#endif /* PLANSELECTOR_H_ */
