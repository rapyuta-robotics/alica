/*
 * ConstraintStore.cpp
 *
 *  Created on: Jul 15, 2014
 *      Author: Paul Panin
 */

#include "engine/constraintmodul/ConstraintStore.h"
//#define CS_DEBUG

#include "engine/RunningPlan.h"
#include "engine/constraintmodul/ConstraintCall.h"
#include "engine/constraintmodul/ConstraintQuery.h"
#include "engine/model/Condition.h"
#include "engine/model/Variable.h"
#include "engine/model/AbstractPlan.h"

#include <iostream>

namespace alica
{

	/**
	 * Default constructor
	 */
	ConstraintStore::ConstraintStore()
	{

	}

	ConstraintStore::~ConstraintStore()
	{

	}

	/**
	 * Clear store, revoking all constraints
	 */
	void ConstraintStore::clear()
	{
		activeVariables.clear();
		mtx.lock();
		activeConditions.clear();
		mtx.unlock();
	}

	/**
	 * Add a condition to the store
	 * @param con A Condition
	 */
	void ConstraintStore::addCondition(Condition* con)
	{
		if (con == nullptr)
		{
			return;
		}
		if (con->getVariables().size() == 0 && con->getQuantifiers().size() == 0)
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
			for (Variable* v : con->getVariables())
			{
				map<Variable*, shared_ptr<vector<Condition*>> >::iterator it = activeVariables.find(v);
				if (it != activeVariables.end())
				{
					it->second->push_back(con);
				}
				else
				{
					shared_ptr<vector<Condition*>> l = make_shared<vector<Condition*>>();
					l->push_back(con);
					activeVariables.insert(pair<Variable*, shared_ptr<vector<Condition*>> >(v, l));
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
	void ConstraintStore::removeCondition(Condition* con)
	{
		if (con == nullptr)
		{
			return;
		}
		bool modified = false;
		mtx.lock();
		if (find(activeConditions.begin(), activeConditions.end(), con) == activeConditions.end())
		{
			modified = true;
			activeConditions.remove(con);
		}
		mtx.unlock();
		if (modified)
		{
			for (Variable* v : con->getVariables())
			{
				remove(activeVariables[v]->begin(), activeVariables[v]->end(), con);
			}
		}

#ifdef CS_DEBUG
		cout << "CS: Removed condition in " << con->getAbstractPlan()->getName() << " with " << con->getVariables().size() << " vars" << endl;
#endif
	}

	void ConstraintStore::acceptQuery(shared_ptr<ConstraintQuery> query, shared_ptr<RunningPlan> rp)
	{
#ifdef CS_DEBUG
		cout << "CS: Accepting Query - Store size is " << activeConditions.size() << endl;
#endif
		if (activeConditions.size() == 0)
		{
			return;
		}

		vector<Variable*> relVars = query->getRelevantStaticVariables(); //get the set of variables that are relevant in the current
		vector<Variable*> relDomainVars = query->getRelevantDomainVariables(); //get all the domain variables
		if (relVars.size() == 0 && relDomainVars.size() == 0)
		{
			return; //nothing to do
		}
		map<Condition*, shared_ptr<ConstraintCall>> newConditions = map<Condition*, shared_ptr<ConstraintCall>>();
		map<Condition*, shared_ptr<ConstraintCall>> allConditions = map<Condition*, shared_ptr<ConstraintCall>>();
		mtx.lock();
		for (Condition* c : activeConditions)
		{
			allConditions.insert(pair<Condition*, shared_ptr<ConstraintCall>>(c, make_shared<ConstraintCall>(c, rp)));
		}
		mtx.unlock();
#ifdef CS_DEBUG
		for (Variable* v : relVars)
		{
			cout << "CS: related vars : " << v->getName() << endl;
		}
#endif

		vector<Variable*> varsToCheck = relVars;
		vector<Variable*> domVarsToCheck = relDomainVars;
		vector<Variable*> varsChecked = vector<Variable*>();
		vector<Variable*> domVarsChecked = vector<Variable*>();
		while (newConditions.size() < allConditions.size() && (domVarsToCheck.size() > 0 || varsToCheck.size() > 0))
		{
			if (varsToCheck.size() > 0)
			{
				Variable* v = varsToCheck[varsToCheck.size() - 1];
				varsToCheck.erase(varsToCheck.begin() + (varsToCheck.size() - 1));
				varsChecked.push_back(v);

				map<Variable*, shared_ptr<vector<Condition*>> >::iterator it = activeVariables.find(v);
				if (it != activeVariables.end())
				{
					for (Condition* c : *it->second)
					{
						if (newConditions.find(c) == newConditions.end())
						{
							shared_ptr<ConstraintCall> cc = allConditions[c];
							newConditions.insert(pair<Condition*, shared_ptr<ConstraintCall>>(c, cc));
							auto sortedVariables = cc->getSortedVariables();
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
							for (Variable* vv : c->getVariables())
							{
								if (find(varsChecked.begin(), varsChecked.end(), vv) == varsChecked.end()
										&& find(varsToCheck.begin(), varsToCheck.end(), vv) == varsToCheck.end())
								{
									varsToCheck.push_back(vv);
								}
							}
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
									pair<Condition*, shared_ptr<ConstraintCall>>(iter->first, iter->second));
							for (Variable* vv : iter->first->getVariables())
							{
								if (find(varsChecked.begin(), varsChecked.end(), vv) == varsChecked.end()
										&& find(varsToCheck.begin(), varsToCheck.end(), vv) == varsToCheck.end())
								{
									varsToCheck.push_back(vv);
								}
							}
							auto sortedVariables = iter->second->getSortedVariables();
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
		varsChecked.insert(varsChecked.end(), varsToCheck.begin(), varsToCheck.end());

		domVarsChecked.insert(domVarsChecked.end(), domVarsToCheck.begin(), domVarsToCheck.end());

		query->setRelevantStaticVariables(varsChecked); //writeback relevant variables, this contains variables obtained earlier
		query->setRelevantDomainVariables(domVarsChecked);
		vector<shared_ptr<ConstraintCall>> constraints = vector<shared_ptr<ConstraintCall>>();
		for (map<Condition*, shared_ptr<ConstraintCall>>::iterator iter = newConditions.begin();
				iter != newConditions.end(); ++iter)
		{
			shared_ptr<ConstraintCall> cc = iter->second;
			if (find(constraints.begin(), constraints.end(), cc) == constraints.end())
			{
				constraints.push_back(iter->second);
			}
		}
		query->addConstraintCalls(constraints);
	}

} /* namespace supplementary */
