/*
 * ConstraintQuery.h
 *
 *  Created on: Oct 17, 2014
 *      Author: Philipp Sperber
 */

#ifndef CONSTRAINTQUERY_H_
#define CONSTRAINTQUERY_H_

#include <memory>
#include <vector>
#include <map>

#include "engine/AlicaEngine.h"
#include "engine/BasicBehaviour.h"
#include "engine/IAlicaClock.h"
#include "engine/ITeamObserver.h"
#include "engine/RunningPlan.h"
#include "engine/constraintmodul/ConstraintCall.h"
#include "engine/constraintmodul/ConstraintDescriptor.h"
#include "engine/constraintmodul/ConstraintStore.h"
#include "engine/constraintmodul/IConstraintSolver.h"
#include "engine/constraintmodul/IVariableSyncModule.h"
#include "engine/constraintmodul/SolverTerm.h"
#include "engine/constraintmodul/SolverVariable.h"
#include "engine/model/Condition.h"
#include "engine/model/State.h"
#include "engine/model/Parametrisation.h"
#include "engine/model/PlanType.h"
#include "engine/model/Variable.h"

using namespace std;

namespace alica
{
	class AlicaEngine;
	class ConstraintCall;
	class ITeamObserver;
	class RunningPlan;
	class Variable;
	class IAlicaClock;
	class BasicBehaviour;
	class IConstraintSolver;

	/**
	 * Encapsulates queries to variables (which are associated with specific solvers).
	 */
	class ConstraintQuery : public enable_shared_from_this<ConstraintQuery>
	{
	public:
		ConstraintQuery(AlicaEngine* ae);

		void addVariable(Variable* v);
		void addVariable(int robot, string ident);
		void clearDomainVariables();
		void clearStaticVariables();
		bool existsSolution(int solverType, shared_ptr<RunningPlan> rp);

		template<class T>
		bool getSolution(int solverType, shared_ptr<RunningPlan> rp, vector<T>& result);

		vector<Variable*> getRelevantStaticVariables();
		void setRelevantStaticVariables(vector<Variable*> value);
		vector<Variable*> getRelevantDomainVariables();
		void setRelevantDomainVariables(vector<Variable*> value);
		void addConstraintCalls(vector<shared_ptr<ConstraintCall>>& l);

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
			void sort();

		private:
			vector<vector<Variable*>> store;
		};

	private:
		bool collectProblemStatement(shared_ptr<RunningPlan> rp, IConstraintSolver* solver,
										vector<shared_ptr<ConstraintDescriptor>>& cds,
										vector<Variable*>& relevantVariables, int& domOffset);

		shared_ptr<UniqueVarStore> store;
		vector<Variable*> queriedStaticVariables;
		vector<Variable*> queriedDomainVariables;
		vector<shared_ptr<ConstraintCall>> calls;

		vector<Variable*> relevantStaticVariables;
		vector<Variable*> relevantDomainVariables;

		AlicaEngine* ae;
	};

	template<class T>
	bool ConstraintQuery::getSolution(int solverType, shared_ptr<RunningPlan> rp, vector<T>& result)
	{
		result.clear();
		IConstraintSolver* solver = this->ae->getSolver(solverType);

		vector<shared_ptr<ConstraintDescriptor>> cds = vector<shared_ptr<ConstraintDescriptor>>();
		vector<Variable*> relevantVariables;
		int domOffset;
		if (!collectProblemStatement(rp, solver, cds, relevantVariables, domOffset))
		{
			return false;
		}
		vector<void*> solverResult;

		bool ret = solver->getSolution(relevantVariables, cds, solverResult);

		//Create result filtered by the queried variables
		if (solverResult.size() > 0)
		{
			result.clear();

			// currently only synch variables if the result/their value is a double
			if (typeid(T) == typeid(double) && ret)
			{
				for (int i = 0; i < solverResult.size(); i++)
				{

					uint8_t* tmp = ((uint8_t*)solverResult.at(i));
					shared_ptr<vector<uint8_t>> result = make_shared<vector<uint8_t>>(sizeof(T));
					//If you have an Segfault/Error here you solver does not return what you are querying! ;)
					for (int s = 0; s < sizeof(T); s++)
					{
						result->at(s) = *tmp;
						tmp++;
					}

					solver->getAlicaEngine()->getResultStore()->postResult(relevantVariables.at(i)->getId(), result);
				}
			}

			//throw "Unexpected Result in Multiple Variables Query!";
			for (int i = 0; i < queriedStaticVariables.size(); ++i)
			{
				result.push_back(*((T*)solverResult.at(store->getIndexOf(queriedStaticVariables[i]))));
			}

			for (int i = 0; i < queriedDomainVariables.size(); ++i)
			{
				for (int j = 0; j < relevantDomainVariables.size(); ++j)
				{
					if (relevantDomainVariables[j] == queriedDomainVariables[i])
					{
						result.push_back(*((T*)solverResult.at(domOffset + j)));
						break;
					}
				}
			}
		}
		for (int i = 0; i < solverResult.size(); i++)
		{
			delete (T*)solverResult.at(i);
		}

		return ret;
	}

}
/* namespace alica */

#endif /* CONSTRAINTQUERY_H_ */
