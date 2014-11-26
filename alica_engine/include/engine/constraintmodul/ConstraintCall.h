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
		shared_ptr<vector<list<vector<Variable* > > >> getSortedVariables();
		shared_ptr<RunningPlan> getRunningPlan();
		shared_ptr<vector<vector<int>>> getAgentsInScope();
	private:
		Condition* condition;
		shared_ptr<vector<list<vector<Variable* > > >> sortedVariables;
		shared_ptr<RunningPlan> runningplan;
		shared_ptr<vector<vector<int>>> agentsinscope;
	};

}
/* namespace alica */

#endif /* CONSTRAINTCALL_H_ */
