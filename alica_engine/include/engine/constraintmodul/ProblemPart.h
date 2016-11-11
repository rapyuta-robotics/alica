/*
 * ConstraintCall.h
 *
 *  Created on: Oct 17, 2014
 *      Author: Philipp Sperber
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

	class ProblemPart
	{
	public:
		ProblemPart(Condition* con, shared_ptr<RunningPlan> rp);

		bool hasVariable(Variable* v);

		Condition* getCondition();
		shared_ptr<vector<list<vector<Variable* > > >> getDomainVariables();
		shared_ptr<RunningPlan> getRunningPlan();
		shared_ptr<vector<shared_ptr<vector<int>>>> getAgentsInScope();
	private:
		Condition* condition;
		shared_ptr<vector<list<vector<Variable* > > >> domainVariables;
		shared_ptr<RunningPlan> runningplan;
		shared_ptr<vector<shared_ptr<vector<int>>>> agentsInScope;
	};

}
/* namespace alica */

#endif /* CONSTRAINTCALL_H_ */
