/*
 * ConstraintQuery.h
 *
 *  Created on: Oct 17, 2014
 *      Author: Philipp
 */

#ifndef CONSTRAINTQUERY_H_
#define CONSTRAINTQUERY_H_

#include <memory>
#include <vector>

using namespace std;

namespace alica
{
	class ConstraintCall;
	class ITeamObserver;
	class RunningPlan;
	class Variable;
	class IAlicaClock;

	/**
	 * Encapsulates specific queries to constraint variable, usually used by behaviours.
	 */
	class ConstraintQuery : enable_shared_from_this<ConstraintQuery>
	{
	public:
		ConstraintQuery();

		void addVariable(Variable* v);
		void addVariable(int robot, string ident);
		void clearDomainVariables();
		void clearStaticVariables();
		bool existsSolution(shared_ptr<RunningPlan> rp);
		bool getSolution(shared_ptr<RunningPlan> rp, vector<double>& result);
//		bool getSolution(shared_ptr<RunningPlan> rp, vector<object>& result);

		vector<Variable*> getRelevantStaticVariables();
		void setRelevantStaticVariables(vector<Variable*> value);
		vector<Variable*> getRelevantDomainVariables();
		void setRelevantDomainVariables(vector<Variable*> value);
		void addConstraintCalls(vector<shared_ptr<ConstraintCall>> l);

		/**
		 * Internal class to deal with bindings in states and plantypes
		 */
		class UniqueVarStore
		{
		public:
			UniqueVarStore();

			void clear();
			void add(Variable* v);
			Variable* getRep(Variable* v);
			void addVarTo(Variable* representing, Variable* toAdd);
			vector<Variable*> getAllRep();
			int getIndexOf(Variable* v);

		private:
			vector<vector<Variable*>> store;
	};

	private:
		shared_ptr<UniqueVarStore> store;
		vector<Variable*> queriedStaticVariables;
		vector<Variable*> queriedDomainVariables;
		ITeamObserver* to;
		vector<shared_ptr<ConstraintCall>> calls;

		vector<Variable*> relevantStaticVariables;
		vector<Variable*> relevantDomainVariables;

		IAlicaClock* alicaClock;
};

}
/* namespace alica */

#endif /* CONSTRAINTQUERY_H_ */
