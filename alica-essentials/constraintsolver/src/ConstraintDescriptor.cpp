/*
 * ConstraintDescriptor.cpp
 *
 *  Created on: Sep 30, 2014
 *      Author: psp
 */

#include "ConstraintDescriptor.h"

#include "ConstraintBuilder.h"

#include <limits>

namespace Alica
{

	ConstraintDescriptor::ConstraintDescriptor(vector<shared_ptr<Variable>> vars,
												vector<vector<vector<shared_ptr<Term>>> > domVars)
	{
		dim = vars.size();
		constraint = ConstraintBuilder::TRUE;
		utility = TermBuilder::constant(1);
		utilitySufficiencyThreshold = numeric_limits<double>::min();
//		fixedValues = ;
		staticRanges = vector<vector<double>>();
		for (int i = 0; i < dim; ++i) {
			staticRanges.push_back(vector<double>(2));
		}
		allVars = vector<shared_ptr<Term>>();
		for (int i = 0; i < dim; ++i) {
			staticRanges[i][0] = min;
			staticRanges[i][1] = max;
			allVars.push_back(vars[i]);
		}
		staticVars = vars;
		domainVars = domVars;
		domainRanges = vector<vector<vector<vector<double>>>>();
		for (vector<vector<shared_ptr<Term>>> lat : domVars) {
			vector<vector<vector<double>>> l = vector<vector<vector<double>>>();
			domainRanges.push_back(l);
			for (vector<shared_ptr<Term>> tarr : lat) {
				vector<vector<double>> r = vector<vector<double>>();
				for (int i = 0; i < tarr.size(); ++i) {
					r.push_back(vector<double>(2));
				}
				l.push_back(r);
				for (int i = 0; i < tarr.size(); ++i) {
					dim++;
					r[i][0] = min;
					r[i][1] = max;
					allVars.push_back(tarr[i]);
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
		setsUtilitySignificanceThreshold(true);
	}

//		void ConstraintDescriptor::setFixedValue(shared_ptr<Term> variable, object value)
//		{
//		}

//		object ConstraintDescriptor::getFixedValue(shared_ptr<Term> variable)
//		{
//		}

	shared_ptr<Term> ConstraintDescriptor::getConstraint()
	{
		return constraint;
	}

	void ConstraintDescriptor::setConstraint(shared_ptr<Term> value)
	{
		constraint = value;
	}

	shared_ptr<Term> ConstraintDescriptor::getUtility()
	{
		return utility;
	}

	void ConstraintDescriptor::setUtility(shared_ptr<Term> value)
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

	vector<shared_ptr<Term>> ConstraintDescriptor::getStaticVars()
	{
		return staticVars;
	}

	void ConstraintDescriptor::setStaticVars(vector<shared_ptr<Term>> value)
	{
		staticVars = value;
	}

	vector<vector<vector<shared_ptr<Term>>> > ConstraintDescriptor::getDomainVars()
	{
		return domainVars;
	}

	void ConstraintDescriptor::setDomainVars(vector<vector<vector<shared_ptr<Term>>>> value)
	{
		domainVars = value;
	}

//		vector<ICollection<int>> ConstraintDescriptor::getAgentsInScope()
//		{
//		}

//		void ConstraintDescriptor::setAgentsInScope(vector<ICollection<int>> value)
//		{
//		}

	vector<shared_ptr<Term>> ConstraintDescriptor::getAllVars()
	{
		return allVars;
	}

	void ConstraintDescriptor::setAllVars(vector<shared_ptr<Term>> value)
	{
		allVars = value;
	}

	vector<vector<double>> ConstraintDescriptor::allRanges()
	{
		vector<vector<double>> allRanges = vector<vector<double>>(staticRanges);
		for (vector<vector<vector<double>>> ld : domainRanges) {
			for (vector<vector<double>> darr : ld) {
				allRanges.insert(allRanges.end(), darr.begin(), darr.end());
			}
		}
		return allRanges;
	}

	vector<vector<vector<vector<double>>>> ConstraintDescriptor::getDomainRanges()
	{
		return domainRanges;
	}

	void ConstraintDescriptor::setDomainRanges(vector<vector<vector<vector<double>>>> value)
	{
		domainRanges = value;
	}

	vector<vector<double>> ConstraintDescriptor::getStaticRanges()
	{
		return staticRanges;
	}

	void ConstraintDescriptor::setStaticRanges(vector<vector<double>> value)
	{
		staticRanges = value;
	}

} /* namespace Alica */
