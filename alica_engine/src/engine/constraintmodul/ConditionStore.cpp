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

		map<Condition*, shared_ptr<ProblemPart>> newConditions = map<Condition*, shared_ptr<ProblemPart>>();
		map<Condition*, shared_ptr<ProblemPart>> allConditions = map<Condition*, shared_ptr<ProblemPart>>();
		mtx.lock();
		for (Condition* cond : activeConditions)
		{
			allConditions.insert(pair<Condition*, shared_ptr<ProblemPart>>(cond, make_shared<ProblemPart>(cond, rp)));
		}
		mtx.unlock();

#ifdef CS_DEBUG
		for (Variable* v : relStaticVars)
		{
			cout << "CS: related vars : " << v->getName() << endl;
		}
#endif

		vector<Variable*> staticVarsToCheck = relStaticVars;
		vector<Variable*> domVarsToCheck = relDomainVars;
		vector<Variable*> staticVarsChecked = vector<Variable*>();
		vector<Variable*> domVarsChecked = vector<Variable*>();
		while (newConditions.size() < allConditions.size() && (domVarsToCheck.size() > 0 || staticVarsToCheck.size() > 0))
		{
			if (staticVarsToCheck.size() > 0)
			{
				Variable* curVariable = staticVarsToCheck[staticVarsToCheck.size() - 1];
				//staticVarsToCheck.erase(staticVarsToCheck.begin() + (staticVarsToCheck.size() - 1));
				staticVarsToCheck.pop_back();
				staticVarsChecked.push_back(curVariable);

				auto it = activeVar2CondMap.find(curVariable);
				if (it == activeVar2CondMap.end())
				{
					// the current variable wasn't active
					continue;
				}

				for (Condition* c : *it->second)
				{
					if (newConditions.find(c) != newConditions.end())
					{
						// condition was already inserted into the newConditions list
						continue;
					}

					shared_ptr<ProblemPart> problemPart = allConditions[c];
					newConditions.insert(pair<Condition*, shared_ptr<ProblemPart>>(c, problemPart));
					auto domainVariables = problemPart->getDomainVariables();
					//for (auto iter = domainVariables->begin(); iter != domainVariables->end(); iter++)
					for (auto& lvarr : (*domainVariables))
					{
						//list<vector<Variable*>> lvarr = *iter;
						for (vector<Variable*> varr : lvarr)
						{
							for (int i = 0; i < varr.size(); ++i)
							{
								if (find(domVarsChecked.begin(), domVarsChecked.end(), varr[i])
										== domVarsChecked.end()
										&& find(domVarsToCheck.begin(), domVarsToCheck.end(), varr[i])
												== domVarsToCheck.end())
								{
									domVarsToCheck.push_back(varr[i]);
								}
							}
						}
					}
					for (Variable* vv : c->getVariables())
					{
						if (find(staticVarsChecked.begin(), staticVarsChecked.end(), vv) == staticVarsChecked.end()
								&& find(staticVarsToCheck.begin(), staticVarsToCheck.end(), vv) == staticVarsToCheck.end())
						{
							staticVarsToCheck.push_back(vv);
						}
					}
				}
			}
			else if (domVarsToCheck.size() > 0)
			{
				Variable* v = domVarsToCheck[domVarsToCheck.size() - 1];
				domVarsToCheck.erase(domVarsToCheck.begin() + (domVarsToCheck.size() - 1));
				domVarsChecked.push_back(v);
				for (auto iter = allConditions.begin(); iter != allConditions.end(); ++iter)
				{
					if (newConditions.find(iter->first) == newConditions.end())
					{
						if (iter->second->hasVariable(v))
						{
							newConditions.insert(
									pair<Condition*, shared_ptr<ProblemPart>>(iter->first, iter->second));
							for (Variable* vv : iter->first->getVariables())
							{
								if (find(staticVarsChecked.begin(), staticVarsChecked.end(), vv) == staticVarsChecked.end()
										&& find(staticVarsToCheck.begin(), staticVarsToCheck.end(), vv) == staticVarsToCheck.end())
								{
									staticVarsToCheck.push_back(vv);
								}
							}
							auto sortedVariables = iter->second->getDomainVariables();
							for (auto iter = sortedVariables->begin(); iter != sortedVariables->end(); iter++)
							{
								list<vector<Variable*>> lvarr = *iter;
								for (vector<Variable*> varr : lvarr)
								{
									for (int i = 0; i < varr.size(); ++i)
									{
										if (find(domVarsChecked.begin(), domVarsChecked.end(), varr[i])
												== domVarsChecked.end()
												&& find(domVarsToCheck.begin(), domVarsToCheck.end(), varr[i])
														== domVarsToCheck.end())
										{
											domVarsToCheck.push_back(varr[i]);
										}
									}
								}
							}
						}
					}
				}
			}
		}
		staticVarsChecked.insert(staticVarsChecked.end(), staticVarsToCheck.begin(), staticVarsToCheck.end());

		domVarsChecked.insert(domVarsChecked.end(), domVarsToCheck.begin(), domVarsToCheck.end());

		//writeback relevant variables, this contains variables obtained earlier
		query->setRelevantStaticVariables(staticVarsChecked);
		query->setRelevantDomainVariables(domVarsChecked);

		vector<shared_ptr<ProblemPart>> problemParts = vector<shared_ptr<ProblemPart>>();
		//for (map<Condition*, shared_ptr<ProblemPart>>::iterator iter = newConditions.begin();
				//iter != newConditions.end(); ++iter)
		for (auto& pair : newConditions)
		{
			shared_ptr<ProblemPart> problemPart = pair.second;
			if (find(problemParts.begin(), problemParts.end(), problemPart) == problemParts.end())
			{
				problemParts.push_back(pair.second);
			}
		}
		query->addProblemParts(problemParts);
	}

} /* namespace supplementary */
