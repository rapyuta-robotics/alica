/*
 * ProblemDescriptor.cpp
 *
 *  Created on: Sep 30, 2014
 *      Author: Philipp Sperber
 */

#include <engine/constraintmodul/ProblemDescriptor.h>
#include "engine/constraintmodul/SolverTerm.h"
#include "engine/constraintmodul/SolverVariable.h"
#include <iostream>
#include <limits>

namespace alica
{

	ProblemDescriptor::ProblemDescriptor(shared_ptr<vector<shared_ptr<SolverVariable>>> vars,
	                                           shared_ptr<vector<shared_ptr<vector<shared_ptr<vector<shared_ptr<SolverVariable>>>>>>> domVars)
	{
		dim = vars->size();
		utilitySufficiencyThreshold = numeric_limits<double>::max();
		staticRanges = make_shared<vector<vector<double>>>();
		for (int i = 0; i < dim; ++i)
		{
			staticRanges->push_back(vector<double>(2));
		}
		allVars = make_shared<vector<shared_ptr<SolverVariable>>>();
		for (int i = 0; i < dim; ++i)
		{
			staticRanges->at(i)[0] = min;
			staticRanges->at(i)[1] = max;
			allVars->push_back(vars->at(i));
		}
		staticVars = make_shared<vector<shared_ptr<SolverVariable>>>(vars->size());
		for (int i = 0; i < vars->size(); ++i)
		{
			staticVars->at(i) = vars->at(i);
		}
		domainVars = domVars;
		domainRanges = make_shared<vector<vector<vector<vector<double>>>>>();
		for (auto iter = domVars->begin(); iter != domVars->end(); iter++) {
			auto lat = *iter;

			vector<vector<vector<double>>> l = vector<vector<vector<double>>>();
			for (auto tarr : *lat)
			{
				vector<vector<double>> r = vector<vector<double>>();
				for (int i = 0; i < tarr->size(); ++i)
				{
					r.push_back(vector<double>(2));
				}
				for (int i = 0; i < tarr->size(); ++i)
				{
					dim++;
					r[i][0] = min;
					r[i][1] = max;
					allVars->push_back(tarr->at(i));
				}
				l.push_back(r);
			}
			domainRanges->push_back(l);
		}
		setSetsUtilitySignificanceThreshold(false);
	}

	bool ProblemDescriptor::getSetsUtilitySignificanceThreshold()
	{
		return setsUtilitySignificanceThreshold;
	}

	void ProblemDescriptor::setSetsUtilitySignificanceThreshold(bool value)
	{
		setsUtilitySignificanceThreshold = value;
	}

	double ProblemDescriptor::getUtilitySignificanceThreshold()
	{
		return utilitySignificanceThreshold;
	}

	void ProblemDescriptor::setUtilitySignificanceThreshold(double value)
	{
		utilitySignificanceThreshold = value;
		setSetsUtilitySignificanceThreshold(true);
	}

	shared_ptr<SolverTerm> ProblemDescriptor::getConstraint()
	{
		return constraint;
	}

	void ProblemDescriptor::setConstraint(shared_ptr<SolverTerm> value)
	{
		constraint = value;
	}

	shared_ptr<SolverTerm> ProblemDescriptor::getUtility()
	{
		return utility;
	}

	void ProblemDescriptor::setUtility(shared_ptr<SolverTerm> value)
	{
		utility = value;
	}

	double ProblemDescriptor::getUtilitySufficiencyThreshold()
	{
		return utilitySufficiencyThreshold;
	}

	void ProblemDescriptor::setUtilitySufficiencyThreshold(double value)
	{
		utilitySufficiencyThreshold = value;
	}

	shared_ptr<vector<shared_ptr<SolverVariable>>> ProblemDescriptor::getStaticVars()
	{
		return staticVars;
	}

	void ProblemDescriptor::setStaticVars(shared_ptr<vector<shared_ptr<SolverVariable>>> value)
	{
		staticVars = value;
	}

	shared_ptr<vector<shared_ptr<vector<shared_ptr<vector<shared_ptr<SolverVariable>>>>> >> ProblemDescriptor::getDomainVars()
	{
		return domainVars;
	}

	void ProblemDescriptor::setDomainVars(shared_ptr<vector<shared_ptr<vector<shared_ptr<vector<shared_ptr<SolverVariable>>>>> >> value)
	{
		domainVars = value;
	}

	shared_ptr<vector<shared_ptr<vector<int>>>> ProblemDescriptor::getAgentsInScope()
	{
		return agentsInScope;
	}

	void ProblemDescriptor::setAgentsInScope(shared_ptr<vector<shared_ptr<vector<int>>>> value)
	{
		agentsInScope = value;
	}

	shared_ptr<vector<shared_ptr<SolverVariable>>> ProblemDescriptor::getAllVars()
	{
		return allVars;
	}

	void ProblemDescriptor::setAllVars(shared_ptr<vector<shared_ptr<SolverVariable>>> value)
	{
		allVars = value;
	}

	shared_ptr<vector<vector<double>>> ProblemDescriptor::allRanges()
	{
		auto allRanges = make_shared<vector<vector<double>>>(*staticRanges);
		for (auto iter = domainRanges->begin(); iter != domainRanges->end(); iter++) {
			vector<vector<vector<double>>> ld = *iter;
			for (vector<vector<double>> darr : ld)
			{
				allRanges->insert(allRanges->end(), darr.begin(), darr.end());
			}
		}
		return allRanges;
	}

	shared_ptr<vector<vector<vector<vector<double>>>>> ProblemDescriptor::getDomainRanges()
	{
		return domainRanges;
	}

	void ProblemDescriptor::setDomainRanges(shared_ptr<vector<vector<vector<vector<double>>>>> value)
	{
		domainRanges = value;
	}

	shared_ptr<vector<vector<double>>> ProblemDescriptor::getStaticRanges()
	{
		return staticRanges;
	}

	void ProblemDescriptor::setStaticRanges(shared_ptr<vector<vector<double>>> value)
	{
		staticRanges = value;
	}

} /* namespace Alica */
