/*
 * IPlanSelector.h
 *
 *  Created on: Jul 9, 2014
 *      Author: Paul Panin
 */

#ifndef IPLANSELECTOR_H_
#define IPLANSELECTOR_H_

#include <vector>
#include <list>
#include <unordered_set>

using namespace std;

namespace alica
{
	class RunningPlan;
	class AbstractPlan;

	class IPlanSelector
	{
	public:
		virtual ~IPlanSelector() {}
		//normal plan selection
//		virtual list<RunningPlan*> getPlansForState(RunningPlan* planningParent,list<AbstractPlan*> plans, unordered_set<int> robotIDs) = 0;
		virtual shared_ptr<list<shared_ptr<RunningPlan>> > getPlansForState(shared_ptr<RunningPlan> planningParent,list<alica::AbstractPlan*>* plans, shared_ptr<vector<int> > robotIDs) = 0;

		/**
		 * Get the best Assignment for this RP with its plan and old Assignment, which is also similar to the old Assignment
		 * @param rp The old RunningPlan
		 * @return The new and similar
		 */
		virtual shared_ptr<RunningPlan> getBestSimilarAssignment(shared_ptr<RunningPlan> rp) = 0;
//		virtual RunningPlan* getBestSimilarAssignment(RunningPlan* rp, unordered_set<int> robots) = 0;
		virtual shared_ptr<RunningPlan> getBestSimilarAssignment(shared_ptr<RunningPlan>rp, shared_ptr<vector<int> > robots) = 0;

	};
}
#endif /* IPLANSELECTOR_H_ */
