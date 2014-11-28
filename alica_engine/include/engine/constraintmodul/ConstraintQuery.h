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
	 * Encapsulates specific queries to constraint variable, usually used by behaviours.
	 */
	class ConstraintQuery : public enable_shared_from_this<ConstraintQuery>
	{
	public:
		ConstraintQuery(BasicBehaviour* behaviour);

		void addVariable(Variable* v);
		void addVariable(int robot, string ident);
		void clearDomainVariables();
		void clearStaticVariables();
		bool existsSolution(int solverType, shared_ptr<RunningPlan> rp);

		template<class T>
		bool getSolution(int solverType, shared_ptr<RunningPlan> rp, vector<T>& result);
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
//		ITeamObserver* to;
		vector<shared_ptr<ConstraintCall>> calls;

		vector<Variable*> relevantStaticVariables;
		vector<Variable*> relevantDomainVariables;

		BasicBehaviour* behaviour;
//		IAlicaClock* alicaClock;
	};

	template<class T>
	bool ConstraintQuery::getSolution(int solverType, shared_ptr<RunningPlan> rp, vector<T>& result)
	{
#ifdef CQ_DEBUG
		long time = behaviour->getRunningPlan()->getAlicaEngine()->getIAlicaClock()->now();
#endif
		store->clear();
		relevantStaticVariables.clear();
		relevantDomainVariables.clear();
		calls.clear();
		relevantStaticVariables.insert(relevantStaticVariables.end(), queriedStaticVariables.begin(),
										queriedStaticVariables.end());
		relevantDomainVariables.insert(relevantDomainVariables.end(), queriedDomainVariables.begin(),
										queriedDomainVariables.end());
		for (Variable* v : relevantStaticVariables)
		{
			store->add(v);
		}
#ifdef CQ_DEBUG
		cout << "CQ: Starting Query with static Vars:";
		for (Variable* v : relevantStaticVariables)
		{
			cout << " " << v->getName();
		}
		cout << endl;
#endif
		while (rp != nullptr && (relevantStaticVariables.size() > 0 || relevantDomainVariables.size() > 0))
		{
#ifdef CQ_DEBUG
			cout << "CQ: Query at " << rp->getPlan()->getName() << endl;
#endif
			rp->getConstraintStore()->acceptQuery(shared_from_this(), rp);
			if (rp->getPlanType() != nullptr)
			{ //process bindings for plantype
				vector<Variable*> tmpVector = vector<Variable*>();
				for (Parametrisation* p : rp->getPlanType()->getParametrisation())
				{
					if (p->getSubPlan() == rp->getPlan()
							&& find(relevantStaticVariables.begin(), relevantStaticVariables.end(), p->getSubVar())
									!= relevantStaticVariables.end())
					{
						tmpVector.push_back(p->getVar());
						store->addVarTo(p->getSubVar(), p->getVar());
					}
				}
				relevantStaticVariables = tmpVector;
			}

			shared_ptr<RunningPlan> parent;
			if (rp->getParent().use_count() > 0)
			{
				parent = rp->getParent().lock();
			}
			if (parent && parent->getActiveState() != nullptr)
			{
				vector<Variable*> tmpVector = vector<Variable*>();
				for (Parametrisation* p : parent->getActiveState()->getParametrisation())
				{
					if ((p->getSubPlan() == rp->getPlan() || p->getSubPlan() == rp->getPlanType())
							&& find(relevantStaticVariables.begin(), relevantStaticVariables.end(), p->getSubVar())
									!= relevantStaticVariables.end())
					{
						tmpVector.push_back(p->getVar());
						store->addVarTo(p->getSubVar(), p->getVar());
					}
				}
				relevantStaticVariables = tmpVector;
			}
			rp = parent;
		}
		//now we have a list of ConstraintCalls in calls ready to be queried together with a store of unifications
//		result = null; TODO: ka wie
		if (calls.size() == 0)
		{
#ifdef CQ_DEBUG
			cout << "CQ: Empty Query!" << endl;
#endif
			return false;
		}

		vector<shared_ptr<ConstraintDescriptor>> cds = vector<shared_ptr<ConstraintDescriptor>>();
		for (shared_ptr<ConstraintCall> c : calls)
		{
			vector<Variable*> conditionVariables = vector<Variable*>(c->getCondition()->getVariables());

			auto varr = make_shared<vector<shared_ptr<SolverVariable>>>(conditionVariables.size());
			for (int j = 0; j < conditionVariables.size(); ++j)
			{
				varr->at(j) = store->getRep(conditionVariables[j])->getSolverVar();
			}
			auto sortedVars = make_shared<vector<shared_ptr<vector<shared_ptr<vector<shared_ptr<SolverTerm>>> >> >>();
			auto agentsInScope = make_shared<vector<shared_ptr<vector<int>>> >();
			for (int j = 0; j < c->getSortedVariables()->size(); ++j)
			{
				auto ll = make_shared<vector<shared_ptr<vector<shared_ptr<SolverTerm>>> >>();
				agentsInScope->push_back(c->getAgentsInScope()->at(j));
				sortedVars->push_back(ll);
				for (auto dvarr : c->getSortedVariables()->at(j))
				{
					auto dtvarr = make_shared<vector<shared_ptr<SolverTerm>>>(dvarr.size());
					for (int i = 0; i < dtvarr->size(); ++i)
					{
						dtvarr->at(i) = dvarr.at(i)->getSolverVar();
					}
					ll->push_back(dtvarr);
				}
			}
			shared_ptr<ConstraintDescriptor> cd = make_shared<ConstraintDescriptor>(varr, sortedVars);
			cd->setAgentsInScope(agentsInScope);
			c->getCondition()->getConstraint(cd, c->getRunningPlan());
			cds.push_back(cd);
		}
		vector<Variable*> qVars = store->getAllRep();
		int domOffset = qVars.size();
		qVars.insert(qVars.end(), relevantDomainVariables.begin(), relevantDomainVariables.end());

		vector<void*> solverResult;
#ifdef CQ_DEBUG
		cout << "CQ: PrepTime: "
		<< (behaviour->getRunningPlan()->getAlicaEngine()->getIAlicaClock()->now() - time) / 10000.0 << endl;
#endif
		bool ret = behaviour->getRunningPlan()->getAlicaEngine()->getSolver(solverType)->getSolution(qVars, cds,
																										solverResult);

		if (solverResult.size() > 0)
		{
			result.clear();

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
		for(int i=0; i<solverResult.size(); i++) {
			delete (T*)solverResult.at(i);
		}

		return ret;
	}

}
/* namespace alica */

#endif /* CONSTRAINTQUERY_H_ */
