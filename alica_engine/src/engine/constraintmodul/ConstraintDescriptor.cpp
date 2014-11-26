/*
 * ConstraintDescriptor.cpp
 *
 *  Created on: Sep 30, 2014
 *      Author: psp
 */

#include "engine/constraintmodul/ConstraintDescriptor.h"

#include "engine/constraintmodul/SolverTerm.h"
#include "engine/constraintmodul/SolverVariable.h"

#include <limits>

namespace alica
{

	ConstraintDescriptor::ConstraintDescriptor(shared_ptr<vector<shared_ptr<SolverVariable>>> vars,
	                                           shared_ptr<vector<vector<vector<shared_ptr<SolverTerm>>>>> domVars)
	{
		dim = vars->size();
//		constraint = ConstraintBuilder::TRUE; TODO: KENNT DEN CONSTRAINTBUILDER NICH
//		utility = TermBuilder::constant(1);	TODO: KENNT DEN TERMBUILDER NICH
		utilitySufficiencyThreshold = numeric_limits<double>::min();
//		fixedValues = ;
		staticRanges = make_shared<vector<vector<double>>>();
		for (int i = 0; i < dim; ++i)
		{
			staticRanges->push_back(vector<double>(2));
		}
		allVars = make_shared<vector<shared_ptr<SolverTerm>>>();
		for (int i = 0; i < dim; ++i)
		{
			staticRanges->at(i)[0] = min;
			staticRanges->at(i)[1] = max;
			allVars->push_back(vars->at(i));
		}
//		staticVars = vars;
		staticVars = make_shared<vector<shared_ptr<SolverTerm>>>(vars->size());
		for (int i = 0; i < vars->size(); ++i)
		{
			staticVars->at(i) = vars->at(i);
		}
		domainVars = domVars;
		domainRanges = make_shared<vector<vector<vector<vector<double>>>>>();
		for (auto iter = domVars->begin(); iter != domVars->end(); iter++) {
			vector<vector<shared_ptr<SolverTerm>>> lat = *iter;

			vector<vector<vector<double>>> l = vector<vector<vector<double>>>();
			domainRanges->push_back(l);
			for (vector<shared_ptr<SolverTerm>> tarr : lat)
			{
				vector<vector<double>> r = vector<vector<double>>();
				for (int i = 0; i < tarr.size(); ++i)
				{
					r.push_back(vector<double>(2));
				}
				l.push_back(r);
				for (int i = 0; i < tarr.size(); ++i)
				{
					dim++;
					r[i][0] = min;
					r[i][1] = max;
					allVars->push_back(tarr[i]);
				}
			}
		}
		setSetsUtilitySignificanceThreshold(false);
	}

	bool ConstraintDescriptor::getSetsUtilitySignificanceThreshold()
	{
		return setsUtilitySignificanceThreshold;
	}

	void ConstraintDescriptor::setSetsUtilitySignificanceThreshold(bool value)
	{
		setsUtilitySignificanceThreshold = value;
	}

	double ConstraintDescriptor::getUtilitySignificanceThreshold()
	{
		return utilitySignificanceThreshold;
	}

	void ConstraintDescriptor::setUtilitySignificanceThreshold(double value)
	{
		utilitySignificanceThreshold = value;
		setSetsUtilitySignificanceThreshold(true);
	}

//		void ConstraintDescriptor::setFixedValue(shared_ptr<SolverTerm> SolverVariable, object value)
//		{
//		}

//		object ConstraintDescriptor::getFixedValue(shared_ptr<SolverTerm> SolverVariable)
//		{
//		}

	shared_ptr<SolverTerm> ConstraintDescriptor::getConstraint()
	{
		return constraint;
	}

	void ConstraintDescriptor::setConstraint(shared_ptr<SolverTerm> value)
	{
		constraint = value;
	}

	shared_ptr<SolverTerm> ConstraintDescriptor::getUtility()
	{
		return utility;
	}

	void ConstraintDescriptor::setUtility(shared_ptr<SolverTerm> value)
	{
		utility = value;
	}

	double ConstraintDescriptor::getUtilitySufficiencyThreshold()
	{
		return utilitySufficiencyThreshold;
	}

	void ConstraintDescriptor::setUtilitySufficiencyThreshold(double value)
	{
		utilitySufficiencyThreshold = value;
	}

	shared_ptr<vector<shared_ptr<SolverTerm>>> ConstraintDescriptor::getStaticVars()
	{
		return staticVars;
	}

	void ConstraintDescriptor::setStaticVars(shared_ptr<vector<shared_ptr<SolverTerm>>> value)
	{
		staticVars = value;
	}

	shared_ptr<vector<vector<vector<shared_ptr<SolverTerm>>>>> ConstraintDescriptor::getDomainVars()
	{
		return domainVars;
	}

	void ConstraintDescriptor::setDomainVars(shared_ptr<vector<vector<vector<shared_ptr<SolverTerm>>>>> value)
	{
		domainVars = value;
	}

	shared_ptr<vector<vector<int>>> ConstraintDescriptor::getAgentsInScope()
	{
		return agentsInScope;
	}

	void ConstraintDescriptor::setAgentsInScope(shared_ptr<vector<vector<int>>> value)
	{
		agentsInScope = value;
	}

	shared_ptr<vector<shared_ptr<SolverTerm>>> ConstraintDescriptor::getAllVars()
	{
		return allVars;
	}

	void ConstraintDescriptor::setAllVars(shared_ptr<vector<shared_ptr<SolverTerm>>> value)
	{
		allVars = value;
	}

	shared_ptr<vector<vector<double>>> ConstraintDescriptor::allRanges()
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

	shared_ptr<vector<vector<vector<vector<double>>>>> ConstraintDescriptor::getDomainRanges()
	{
		return domainRanges;
	}

	void ConstraintDescriptor::setDomainRanges(shared_ptr<vector<vector<vector<vector<double>>>>> value)
	{
		domainRanges = value;
	}

	shared_ptr<vector<vector<double>>> ConstraintDescriptor::getStaticRanges()
	{
		return staticRanges;
	}

	void ConstraintDescriptor::setStaticRanges(shared_ptr<vector<vector<double>>> value)
	{
		staticRanges = value;
	}

} /* namespace Alica */
