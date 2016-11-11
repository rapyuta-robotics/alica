/*
 * ConstraintQuery.cpp
 *
 *  Created on: Oct 17, 2014
 *      Author: Philipp Sperber
 */
#include <engine/constraintmodul/ConditionStore.h>
#include <engine/constraintmodul/ISolver.h>
#include <engine/constraintmodul/ProblemDescriptor.h>
#include <engine/constraintmodul/ProblemPart.h>
#include <engine/constraintmodul/Query.h>
#include "engine/AlicaEngine.h"
#include "engine/BasicBehaviour.h"
#include "engine/IAlicaClock.h"
#include "engine/ITeamObserver.h"
#include "engine/RunningPlan.h"
#include "engine/collections/RobotEngineData.h"
#include "engine/constraintmodul/SolverTerm.h"
#include "engine/constraintmodul/SolverVariable.h"
#include "engine/model/Condition.h"
#include "engine/model/State.h"
#include "engine/model/Parametrisation.h"
#include "engine/model/PlanType.h"
#include "engine/model/Variable.h"

#include <iostream>

namespace alica
{
	Query::Query(AlicaEngine* ae) :
			ae(ae)
	{
		uniqueVarStore = make_shared<UniqueVarStore>();
	}

	void Query::addVariable(Variable* v)
	{
		queriedStaticVariables.push_back(v);
	}

	void Query::addVariable(int robot, string ident)
	{
		queriedDomainVariables.push_back(this->ae->getTeamObserver()->getRobotById(robot)->getSortedVariable(ident));
	}

	void Query::clearDomainVariables()
	{
		queriedDomainVariables.clear();
	}

	void Query::clearStaticVariables()
	{
		queriedStaticVariables.clear();
	}

	bool Query::existsSolution(int solverType, shared_ptr<RunningPlan> rp)
	{
		ISolver* solver = this->ae->getSolver(solverType);

		vector<shared_ptr<ProblemDescriptor>> cds = vector<shared_ptr<ProblemDescriptor>>();
		vector<Variable*> relevantVariables;
		int domOffset;
		if (!collectProblemStatement(rp, solver, cds, relevantVariables, domOffset))
		{
			return false;
		}
		return solver->existsSolution(relevantVariables, cds);
	}

	bool Query::collectProblemStatement(shared_ptr<RunningPlan> rp, ISolver* solver,
													vector<shared_ptr<ProblemDescriptor>>& pds,
													vector<Variable*>& relevantVariables, int& domOffset)
	{
#ifdef CQ_DEBUG
		long time = rp->getAlicaEngine()->getIAlicaClock()->now();
#endif
		// Clear variable stuff and calls
		uniqueVarStore->clear();
		relevantStaticVariables.clear();
		relevantDomainVariables.clear();
		problemParts.clear();

		// insert queried variables of this query
		relevantStaticVariables.insert(relevantStaticVariables.end(), queriedStaticVariables.begin(),
										queriedStaticVariables.end());
		relevantDomainVariables.insert(relevantDomainVariables.end(), queriedDomainVariables.begin(),
										queriedDomainVariables.end());

		// add static variables to unique variable store
		for (Variable* v : relevantStaticVariables)
		{
			uniqueVarStore->add(v);
		}

#ifdef CQ_DEBUG
		cout << "Query: Starting Query with static Vars:";
		for (Variable* v : relevantStaticVariables)
		{
			cout << " " << v->getName();
		}
		cout << endl;
#endif

		while (rp != nullptr && (relevantStaticVariables.size() > 0 || relevantDomainVariables.size() > 0))
		{
#ifdef CQ_DEBUG
			cout << "Query: Query at " << rp->getPlan()->getName() << endl;
#endif
			rp->getConstraintStore()->acceptQuery(shared_from_this(), rp);
			if (rp->getPlanType() != nullptr)
			{
				//process bindings for plantype
				vector<Variable*> tmpVector = vector<Variable*>();
				for (Parametrisation* p : rp->getPlanType()->getParametrisation())
				{
					if (p->getSubPlan() == rp->getPlan()
							&& find(relevantStaticVariables.begin(), relevantStaticVariables.end(), p->getSubVar())
									!= relevantStaticVariables.end())
					{
						tmpVector.push_back(p->getVar());
						uniqueVarStore->addVarTo(p->getSubVar(), p->getVar());
					}
				}
				relevantStaticVariables = tmpVector;
			}

			shared_ptr<RunningPlan> parent;
			if (!rp->getParent().expired())
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
						uniqueVarStore->addVarTo(p->getSubVar(), p->getVar());
					}
				}
				relevantStaticVariables = tmpVector;
			}
			rp = parent;
		}

		//now we have a list of ConstraintCalls in calls ready to be queried together with a store of unifications
		if (problemParts.size() == 0)
		{
#ifdef CQ_DEBUG
			cout << "CQ: Empty Query!" << endl;
#endif
			return false;
		}

		for (shared_ptr<ProblemPart> c : problemParts)
		{

			vector<Variable*> conditionVariables = vector<Variable*>(c->getCondition()->getVariables());

			auto varr = make_shared<vector<shared_ptr<SolverVariable>>>(conditionVariables.size());
			for (int j = 0; j < conditionVariables.size(); ++j)
			{
				if (uniqueVarStore->getRep(conditionVariables[j])->getSolverVar() == nullptr)
				{
					uniqueVarStore->getRep(conditionVariables[j])->setSolverVar(
							solver->createVariable(uniqueVarStore->getRep(conditionVariables[j])->getId()));
				}
				varr->at(j) = uniqueVarStore->getRep(conditionVariables[j])->getSolverVar();
			}
			auto sortedVars = make_shared<vector<shared_ptr<vector<shared_ptr<vector<shared_ptr<SolverVariable>>> >> >>();
			auto agentsInScope = make_shared<vector<shared_ptr<vector<int>>> >();
			for (int j = 0; j < c->getDomainVariables()->size(); ++j)
			{
				auto ll = make_shared<vector<shared_ptr<vector<shared_ptr<SolverVariable>>> >>();
				agentsInScope->push_back(c->getAgentsInScope()->at(j));
				sortedVars->push_back(ll);
				for (auto dvarr : c->getDomainVariables()->at(j))
				{
					auto dtvarr = make_shared<vector<shared_ptr<SolverVariable>>>(dvarr.size());
					for (int i = 0; i < dtvarr->size(); ++i)
					{
						if(dvarr.at(i)->getSolverVar() == nullptr /*!dvarr.at(i)->getSolverVar().operator bool()*/)
						{
							dvarr.at(i)->setSolverVar(solver->createVariable(dvarr.at(i)->getId()));
						}

						dtvarr->at(i) = dvarr.at(i)->getSolverVar();
					}
					ll->push_back(dtvarr);
				}
			}
			shared_ptr<ProblemDescriptor> cd = make_shared<ProblemDescriptor>(varr, sortedVars);
			cd->setAgentsInScope(agentsInScope);
			c->getCondition()->getConstraint(cd, c->getRunningPlan());
			pds.push_back(cd);
		}
		relevantVariables = uniqueVarStore->getAllRep();
		domOffset = relevantVariables.size();
		relevantVariables.insert(relevantVariables.end(), relevantDomainVariables.begin(),
									relevantDomainVariables.end());

#ifdef CQ_DEBUG
		cout << "CQ: PrepTime: " << (ae->getIAlicaClock()->now() - time) / 10000.0 << endl;
#endif
		return true;
	}

	vector<Variable*> Query::getRelevantStaticVariables()
	{
		return relevantStaticVariables;
	}

	void Query::setRelevantStaticVariables(vector<Variable*> value)
	{
		relevantStaticVariables = value;
	}

	vector<Variable*> Query::getRelevantDomainVariables()
	{
		return relevantDomainVariables;
	}

	void Query::setRelevantDomainVariables(vector<Variable*> value)
	{
		relevantDomainVariables = value;
	}

	void Query::addProblemParts(vector<shared_ptr<ProblemPart>>& l)
	{
		problemParts.insert(problemParts.end(), l.begin(), l.end());
	}

	Query::UniqueVarStore::UniqueVarStore()
	{
		store = vector<vector<Variable*>>();
	}

	void Query::UniqueVarStore::clear()
	{
		store.clear();
	}

	/**
	 * Initializes a list with the given variable and put that list into the internal store.
	 */
	void Query::UniqueVarStore::add(Variable* v)
	{
		vector<Variable*> l = vector<Variable*>();
		l.push_back(v);
		store.push_back(l);
	}

	Variable* Query::UniqueVarStore::getRep(Variable* v)
	{
		for (vector<Variable*> l : store)
		{
			for (Variable* s : l)
			{
				if (s == v)
				{
					return l.front();
				}
			}
		}
		add(v);
		return v;
	}

	void Query::UniqueVarStore::addVarTo(Variable* representing, Variable* toAdd)
	{
		for (vector<Variable*>& l : store)
		{
			for (Variable*& cv : l)
			{
				if (representing == cv)
				{
					l.insert(l.begin(), toAdd);
					return;
				}
			}
		}
		vector<Variable*> nl = vector<Variable*>();
		nl.insert(nl.begin(), representing);
		nl.insert(nl.begin(), toAdd);
		store.push_back(nl);
	}

	vector<Variable*> Query::UniqueVarStore::getAllRep()
	{
		vector<Variable*> ret = vector<Variable*>();
		for (vector<Variable*> l : store)
		{
			ret.push_back(l.front());
		}
		return ret;
	}

	int Query::UniqueVarStore::getIndexOf(Variable* v)
	{
		for (int i = 0; i < store.size(); ++i)
		{
			for (Variable* c : store[i])
			{
				if (c == v)
				{
					return i;
				}
			}
		}
		return -1;
	}
} /* namespace alica */
