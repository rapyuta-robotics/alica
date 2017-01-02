/*
 * ConstraintStore.cpp
 *
 *  Created on: Jul 15, 2014
 *      Author: Paul Panin
 */

#include <engine/constraintmodul/ConditionStore.h>
#include <engine/constraintmodul/ProblemPart.h>
#include <engine/constraintmodul/Query.h>
#include "engine/RunningPlan.h"
#include "engine/model/Condition.h"
#include "engine/model/Variable.h"
#include "engine/model/AbstractPlan.h"

#include <iostream>

//#define CS_DEBUG

namespace alica
{

	/**
	 * Default constructor
	 */
	ConditionStore::ConditionStore()
	{

	}

	ConditionStore::~ConditionStore()
	{

	}

	/**
	 * Clear store, revoking all constraints
	 */
	void ConditionStore::clear()
	{
		activeVar2CondMap.clear();
		mtx.lock();
		activeConditions.clear();
		mtx.unlock();
	}

	/**
	 * Add a condition to the store
	 * @param con A Condition
	 */
	void ConditionStore::addCondition(Condition* con)
	{
		if (con == nullptr || (con->getVariables().size() == 0 && con->getQuantifiers().size() == 0))
		{
			return;
		}

		bool modified = false;
		mtx.lock();
		if (find(activeConditions.begin(), activeConditions.end(), con) == activeConditions.end())
		{
			modified = true;
			activeConditions.push_back(con);
		}
		mtx.unlock();
		if (modified)
		{
			for (Variable* variable : con->getVariables())
			{
				auto it = activeVar2CondMap.find(variable);
				if (it != activeVar2CondMap.end())
				{
					it->second->push_back(con);
				}
				else
				{
					shared_ptr<vector<Condition*>> condList = make_shared<vector<Condition*>>();
					condList->push_back(con);
					activeVar2CondMap.insert(pair<Variable*, shared_ptr<vector<Condition*>> >(variable, condList));
				}
			}
		}
#ifdef CS_DEBUG
		cout << "CS: Added condition in " << con->getAbstractPlan()->getName() << " with " << con->getVariables().size() << " vars" << endl;
#endif
	}

	/**
	 * Revoke a specific condition from the constraint store
	 *
	 * @param con The condition to be removed
	 */
	void ConditionStore::removeCondition(Condition* con)
	{
		if (con == nullptr)
		{
			return;
		}
		bool modified = false;
		mtx.lock();
		if (find(activeConditions.begin(), activeConditions.end(), con) != activeConditions.end())
		{
			modified = true;
			activeConditions.remove(con);
		}
		mtx.unlock();
		if (modified)
		{
			for (Variable* v : con->getVariables())
			{
				remove(activeVar2CondMap[v]->begin(), activeVar2CondMap[v]->end(), con);
			}
		}

#ifdef CS_DEBUG
		cout << "CS: Removed condition in " << con->getAbstractPlan()->getName() << " with " << con->getVariables().size() << " vars" << endl;
#endif
	}

	/**
	 * Writes static and domain variables, as well as, problem parts into the query.
	 */
	void ConditionStore::acceptQuery(shared_ptr<Query> query, shared_ptr<RunningPlan> rp)
	{
#ifdef CS_DEBUG
		cout << "CS: Accepting Query - Store size is " << activeConditions.size() << endl;
#endif
		if (activeConditions.size() == 0)
		{
			return;
		}

		vector<Variable*> relStaticVars = query->getRelevantStaticVariables();
		vector<Variable*> relDomainVars = query->getRelevantDomainVariables();
		if (relStaticVars.size() == 0 && relDomainVars.size() == 0)
		{
			return; //nothing to do
		}

		map<Condition*, shared_ptr<ProblemPart>> newCondProbPartMap = map<Condition*, shared_ptr<ProblemPart>>();
		map<Condition*, shared_ptr<ProblemPart>> allCondProbPartMap = map<Condition*, shared_ptr<ProblemPart>>();
		mtx.lock();
		for (Condition* cond : activeConditions)
		{
			//allCondProbPartMap.insert(pair<Condition*, shared_ptr<ProblemPart>>(cond, make_shared<ProblemPart>(cond, rp)));
			allCondProbPartMap.emplace(cond, make_shared<ProblemPart>(cond, rp));
		}
		mtx.unlock();

#ifdef CS_DEBUG
		std::cout << "CS: Relevant Static Vars: ";
		for (Variable* v : relStaticVars)
		{
			 std::cout << v->getName() << ", ";
		}
		std::cout << std::endl;
#endif

		vector<Variable*> staticVarsToCheck = relStaticVars;
		vector<Variable*> domVarsToCheck = relDomainVars;
		vector<Variable*> staticVarsChecked = vector<Variable*>();
		vector<Variable*> domVarsChecked = vector<Variable*>();
		while (newCondProbPartMap.size() < allCondProbPartMap.size() && (domVarsToCheck.size() > 0 || staticVarsToCheck.size() > 0))
		{
			if (staticVarsToCheck.size() > 0)
			{
				Variable* curStaticVariable = staticVarsToCheck[staticVarsToCheck.size() - 1];
				staticVarsToCheck.pop_back();
				staticVarsChecked.push_back(curStaticVariable);

#ifdef CS_DEBUG
				std::cout << "CS: Checking static variable " << curStaticVariable->getName() << "(" << curStaticVariable->getId() << ")" << std::endl;
#endif

				auto it = activeVar2CondMap.find(curStaticVariable);
				if (it == activeVar2CondMap.end())
				{
					// the current variable wasn't active
					continue;
				}

#ifdef CS_DEBUG
				std::cout << "CS: Conditions active under variable " << it->first->getName() << ": " << it->second->size() << std::endl;
#endif
				for (Condition* c : *(it->second))
				{
					if (newCondProbPartMap.find(c) != newCondProbPartMap.end())
					{
						// condition was already inserted into the newCondProbPartMap
						continue;
					}

					shared_ptr<ProblemPart> problemPart = allCondProbPartMap[c];
					newCondProbPartMap.emplace(c, problemPart);
					/**
					 *  Hierarchie: 1.vector< 2.list< 3.vector< 4.Variable* > > >
					 * 1. Vector of Quantors, e.g., For all agents in state S variables X,Y exist.
					 * 2. List of Robots, e.g., An agent has variables X,Y.
					 * 3. Vector of Variables, e.g., variables X,Y.
					 * 4. Variable, e.g., variable X.
					 */
					auto domainVariables = problemPart->getDomainVariables();
					for (auto& listOfRobots : (*domainVariables))
					{
						for (vector<Variable*> variables : listOfRobots)
						{
							for(auto variable : variables)
							{
								if (find(domVarsChecked.begin(), domVarsChecked.end(), variable) == domVarsChecked.end()
										&& find(domVarsToCheck.begin(), domVarsToCheck.end(), variable) == domVarsToCheck.end())
								{
									cout << "CS: Adding DomVar : " << variable->getName() << endl;
									domVarsToCheck.push_back(variable);
								}
							}
						}
					}
					for (Variable* variable : c->getVariables())
					{
						if (find(staticVarsChecked.begin(), staticVarsChecked.end(), variable) == staticVarsChecked.end()
								&& find(staticVarsToCheck.begin(), staticVarsToCheck.end(), variable) == staticVarsToCheck.end())
						{
							staticVarsToCheck.push_back(variable);
						}
					}
				}
			}
			else if (domVarsToCheck.size() > 0)
			{
				Variable* curDomainVariable = domVarsToCheck[domVarsToCheck.size() - 1];
				domVarsToCheck.pop_back();
				domVarsChecked.push_back(curDomainVariable);

				for (auto& condProbPartPair : allCondProbPartMap)
				{
					if (newCondProbPartMap.find(condProbPartPair.first) != newCondProbPartMap.end())
					{
						// condition was already in the newCondProbPartMap
						continue;
					}

					if (!condProbPartPair.second->hasVariable(curDomainVariable))
					{
						// curDomainVariable does not exist in the problem part
						continue;
					}

					newCondProbPartMap.emplace(condProbPartPair.first, condProbPartPair.second);

					/**
					 *  Hierarchie: 1.vector< 2.list< 3.vector< 4.Variable* > > >
					 * 1. Vector of Quantors, e.g., For all agents in state S variables X,Y exist.
					 * 2. List of Robots, e.g., An agent has variables X,Y.
					 * 3. Vector of Variables, e.g., variables X,Y.
					 * 4. Variable, e.g., variable X.
					 */
					for (auto& listOfRobots : (*condProbPartPair.second->getDomainVariables()))
					{
						for (auto& variables : listOfRobots)
						{
							for (auto& variable : variables)
							{
								if (find(domVarsChecked.begin(), domVarsChecked.end(), variable) == domVarsChecked.end()
										&& find(domVarsToCheck.begin(), domVarsToCheck.end(), variable) == domVarsToCheck.end())
								{
									domVarsToCheck.push_back(variable);
								}
							}
						}
					}

					for (Variable* variable : condProbPartPair.first->getVariables())
					{
						if (find(staticVarsChecked.begin(), staticVarsChecked.end(), variable) == staticVarsChecked.end()
								&& find(staticVarsToCheck.begin(), staticVarsToCheck.end(), variable) == staticVarsToCheck.end())
						{
							staticVarsToCheck.push_back(variable);
						}
					}
				}
			}
		}
		if (!staticVarsChecked.empty())
		{
			staticVarsChecked.insert(staticVarsChecked.end(), staticVarsToCheck.begin(), staticVarsToCheck.end());
		}

		if (!domVarsToCheck.empty())
		{
			domVarsChecked.insert(domVarsChecked.end(), domVarsToCheck.begin(), domVarsToCheck.end());
		}

		//write back relevant variables, this contains variables obtained earlier
		query->setRelevantStaticVariables(staticVarsChecked);
		query->setRelevantDomainVariables(domVarsChecked);

		// write back problem parts
		vector<shared_ptr<ProblemPart>> problemParts = vector<shared_ptr<ProblemPart>>();
		for (auto& pair : newCondProbPartMap)
		{
			auto problemPart = pair.second;
			if (find(problemParts.begin(), problemParts.end(), problemPart) == problemParts.end())
			{
				problemParts.push_back(problemPart);
			}
		}
		query->addProblemParts(problemParts);
	}

} /* namespace supplementary */
