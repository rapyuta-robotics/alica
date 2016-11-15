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
		/**
		 *  Hierarchie: 1.vector< 2.list< 3.vector< 4.Variable* > > >
		 * 1. Vector of Quantors, e.g., For all agents in state S variables X,Y exist.
		 * 2. List of Robots, e.g., An agent has variables X,Y.
		 * 3. Vector of Variables, e.g., variables X,Y.
		 * 4. Variable, e.g., variable X.
		 */
		shared_ptr<vector<list<vector<Variable* > > >> domainVariables;
		shared_ptr<RunningPlan> runningplan;
		shared_ptr<vector<shared_ptr<vector<int>>>> agentsInScope;
	};

}
/* namespace alica */

#endif /* CONSTRAINTCALL_H_ */
