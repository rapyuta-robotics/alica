/*
 * ConstraintCall.h
 *
 *  Created on: Oct 17, 2014
 *      Author: Philipp
 */

#ifndef CONSTRAINTCALL_H_
#define CONSTRAINTCALL_H_

#include <memory>
#include <vector>
#include <list>

using namespace std;

namespace alica
{
	class Condition;
	class RunningPlan;
	class Variable;

	class ConstraintCall
	{
	public:
		ConstraintCall(Condition* con, shared_ptr<RunningPlan> rp);

		bool hasVariable(Variable* v);

		Condition* getCondition();
		vector<list<vector<Variable* > > > getSortedVariables();
		shared_ptr<RunningPlan> getRunningPlan();
		vector<vector<int>> getAgentsInScope();
	private:
		Condition* condition;
		vector<list<vector<Variable* > > > sortedvariables;
		shared_ptr<RunningPlan> runningplan;
		vector<vector<int>> agentsinscope;
	};

}
/* namespace alica */

#endif /* CONSTRAINTCALL_H_ */
