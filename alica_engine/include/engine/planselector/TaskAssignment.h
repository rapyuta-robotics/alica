/*
 * TaskAssignment.h
 *
 *  Created on: Jul 4, 2014
 *      Author: Stefan Jakob
 */

#ifndef TASKASSIGNMENT_H_
#define TASKASSIGNMENT_H_
#define EXPANSIONEVAL

#include "engine/ITaskAssignment.h"

#include <list>
#include <vector>
#include <map>
#include <string>
#include <sstream>

namespace alica
{

	class IAssignment;
	class Assignment;
	class Plan;
	class EntryPoint;
	class PartialAssignment;
	class SimplePlanTree;

	class TaskAssignment : virtual public ITaskAssignment
	{
	public:
		TaskAssignment();
		TaskAssignment(list<Plan*> planList, vector<int> paraRobots, bool preasingOtherRobots);
		virtual ~TaskAssignment();
		Assignment* getNextBestAssignment(IAssignment* oldAss);
		string toString();
#ifdef EXPANSIONEVAL
		int getExpansionCount() const;
		void setExpansionCount(int expansionCount);
#endif
	private:
		PartialAssignment* calcNextBestPartialAssignment(IAssignment* oldAss);

	protected:
		list<Plan*> planList;
		vector<int> robots;
		vector<EntryPoint*> entryPointVector;
		vector<PartialAssignment*> fringe;
		bool addAlreadyAssignedRobots(PartialAssignment* pa, map<int, SimplePlanTree*> simplePlanTreeMap);

#ifdef EXPANSIONEVAL
		int expansionCount;
#endif
	};

} /* namespace alica */

#endif /* TASKASSIGNMENT_H_ */
