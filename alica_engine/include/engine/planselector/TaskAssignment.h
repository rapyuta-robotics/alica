/*
 * TaskAssignment.h
 *
 *  Created on: Jul 4, 2014
 *      Author: Stefan Jakob
 */

#ifndef TASKASSIGNMENT_H_
#define TASKASSIGNMENT_H_
#define EXPANSIONEVAL
#define PSDEBUG

using namespace std;

#include "engine/ITaskAssignment.h"

#include <list>
#include <vector>
#include <memory>
#include <map>
#include <string>
#include <sstream>
#include <algorithm>
#include <memory>

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
		TaskAssignment(list<Plan*> planList, shared_ptr<vector<int> > paraRobots, bool preasingOtherRobots);
		virtual ~TaskAssignment();
		Assignment* getNextBestAssignment(IAssignment* oldAss);
		string toString();
#ifdef EXPANSIONEVAL
		int getExpansionCount();
		void setExpansionCount(int expansionCount);
#endif
	private:
		PartialAssignment* calcNextBestPartialAssignment(IAssignment* oldAss);

	protected:
		list<Plan*> planList;
		shared_ptr<vector<int> > robots;
		vector<EntryPoint*> entryPointVector;
		//TODO has to be sorted every time used
		vector<PartialAssignment*> fringe;
		bool addAlreadyAssignedRobots(PartialAssignment* pa, map<int, shared_ptr<SimplePlanTree> >* simplePlanTreeMap);

#ifdef EXPANSIONEVAL
		int expansionCount;
#endif
	};

} /* namespace alica */

#endif /* TASKASSIGNMENT_H_ */
