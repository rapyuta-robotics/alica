/*
 * TaskAssignment.h
 *
 *  Created on: Jul 4, 2014
 *      Author: Stefan Jakob
 */

#ifndef TASKASSIGNMENT_H_
#define TASKASSIGNMENT_H_

//#define EXPANSIONEVAL
//#define TA_DEBUG


#include "engine/ITaskAssignment.h"
#include "engine/IRobotID.h"
#include <list>
#include <vector>
#include <memory>
#include <map>
#include <string>
#include <sstream>
#include <algorithm>
#include <memory>

using namespace std;
namespace alica
{

	class IAssignment;
	class Assignment;
	class Plan;
	class EntryPoint;
	class PartialAssignment;
	class SimplePlanTree;
	class ITeamObserver;
	class PartialAssignmentPool;

	/**
	 * Represents an instance of an assignment problem for one plan or a plantype.
	 * All parameters, which are static for this problem, are stored here.
	 */
	class TaskAssignment : virtual public ITaskAssignment
	{
	public:
		TaskAssignment(PartialAssignmentPool* pap, ITeamObserver* to, list<Plan*> planList, shared_ptr<vector<const alica::IRobotID*> > paraRobots, bool preasingOtherRobots);
		virtual ~TaskAssignment();
		shared_ptr<Assignment> getNextBestAssignment(IAssignment* oldAss);
		string toString();
#ifdef EXPANSIONEVAL
		int getExpansionCount();
		void setExpansionCount(int expansionCount);
#endif
	private:
		PartialAssignment* calcNextBestPartialAssignment(IAssignment* oldAss);

	protected:
		// Plan to build an assignment for
		ITeamObserver* to;
		list<Plan*> planList;
		shared_ptr<vector<const alica::IRobotID*> > robots;
		vector<EntryPoint*> entryPointVector;
		// Fringe of the search tree
		vector<PartialAssignment*> fringe;
		bool addAlreadyAssignedRobots(PartialAssignment* pa, map<const alica::IRobotID*, shared_ptr<SimplePlanTree> >* simplePlanTreeMap);

#ifdef EXPANSIONEVAL
		int expansionCount;
#endif
	};

} /* namespace alica */

#endif /* TASKASSIGNMENT_H_ */
