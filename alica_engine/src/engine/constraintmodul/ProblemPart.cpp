/*
 * ConstraintCall.cpp
 *
 *  Created on: Oct 17, 2014
 *      Author: Philipp
 */

#include <engine/constraintmodul/ProblemPart.h>
#include "engine/RunningPlan.h"
#include "engine/model/Condition.h"
#include "engine/model/Quantifier.h"

namespace alica
{

	ProblemPart::ProblemPart(Condition* con, shared_ptr<RunningPlan> rp)
	{
		condition = con;
		domainVariables = make_shared<vector<list<vector<Variable*>> >>();
		agentsInScope = make_shared<vector<shared_ptr<vector<int>>>>();
		for (Quantifier* quantifier : condition->getQuantifiers())
		{
			shared_ptr<vector<int>> robots;
			domainVariables->push_back(*quantifier->getDomainVariables(rp, robots));
			if (robots) {
				agentsInScope->push_back(robots);
			} else {
				agentsInScope->push_back(make_shared<vector<int>>());
			}
		}
		runningplan = rp;
	}

	/**
	 * Checks whether the given variable is one of the domain variables.
	 */
	bool ProblemPart::hasVariable(Variable* v)
	{
		for (auto& listOfRobots : (*domainVariables))
		{
			for (vector<Variable*> variables : listOfRobots)
			{
				for(auto variable : variables)
				{
					if (variable == v)
					{
						return true;
					}
				}
			}
		}
		return false;
	}

	Condition* ProblemPart::getCondition()
	{
		return condition;
	}

	shared_ptr<vector<list<vector<Variable* > > >> ProblemPart::getDomainVariables()
	{
		return domainVariables;
	}

	shared_ptr<RunningPlan> ProblemPart::getRunningPlan()
	{
		return runningplan;
	}

	shared_ptr<vector<shared_ptr<vector<int>>>> ProblemPart::getAgentsInScope()
	{
		return agentsInScope;
	}

} /* namespace alica */
