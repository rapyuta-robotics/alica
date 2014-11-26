/*
 * ConstraintCall.cpp
 *
 *  Created on: Oct 17, 2014
 *      Author: Philipp
 */

#include "engine/constraintmodul/ConstraintCall.h"

#include "engine/RunningPlan.h"
#include "engine/model/Condition.h"
#include "engine/model/Quantifier.h"

namespace alica
{

	ConstraintCall::ConstraintCall(Condition* con, shared_ptr<RunningPlan> rp)
	{
		condition = con;
		sortedVariables = make_shared<vector<list<vector<Variable*>> >>();
		agentsinscope = make_shared<vector<vector<int>>>();
		for (Quantifier* q : condition->getQuantifiers())
		{
			shared_ptr<vector<int>> robots;
			// TODO: aus rp.get() muss rp
			sortedVariables->push_back(*q->getSortedVariables(rp.get(), robots));
			if (robots != nullptr) {
				agentsinscope->push_back(*robots);
			} else {
				agentsinscope->push_back(vector<int>());
			}
		}
		runningplan = rp;
	}

	bool ConstraintCall::hasVariable(Variable* v)
	{
		for (auto iter = sortedVariables->begin(); iter != sortedVariables->end(); iter++) {
			list<vector<Variable*>> lvarr = *iter;
			for (vector<Variable* > varr : lvarr)
			{
				for (int i = 0; i < varr.size(); ++i) {
					if (varr[i] == v) {
						return true;
					}
				}
			}
		}
		return false;
	}

	Condition* ConstraintCall::getCondition()
	{
		return condition;
	}

	shared_ptr<vector<list<vector<Variable* > > >> ConstraintCall::getSortedVariables()
	{
		return sortedVariables;
	}

	shared_ptr<RunningPlan> ConstraintCall::getRunningPlan()
	{
		return runningplan;
	}

	shared_ptr<vector<vector<int>>> ConstraintCall::getAgentsInScope()
	{
		return agentsinscope;
	}

} /* namespace alica */
