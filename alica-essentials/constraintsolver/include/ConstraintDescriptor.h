/*
 * ConstraintDescriptor.h
 *
 *  Created on: Sep 30, 2014
 *      Author: psp
 */

#ifndef CONSTRAINTDESCRIPTOR_H_
#define CONSTRAINTDESCRIPTOR_H_

#include <AutoDiff.h>

#include <memory>

using namespace std;
using namespace AutoDiff;

namespace Alica
{

	class ConstraintDescriptor : public enable_shared_from_this<ConstraintDescriptor>
	{
	private:
		int dim;
		const double min = -10E29;
		const double max = 10E29;

		double utilitySignificanceThreshold = 1E-22;
		bool setsUtilitySignificanceThreshold;
		double utilitySignificanceThreshold;

// TODO:		map<int, object> fixedValues;

		shared_ptr<Term> constraint;
		shared_ptr<Term> utility;
		double utilitySufficiencyThreshold;
		vector<shared_ptr<Term>> staticVars;
		vector<vector<vector<shared_ptr<Term>>>> domainVars;
//		vector<ICollection<int>> agentsInScope;
		vector<shared_ptr<Term>> allVars;

		vector<vector<vector<vector<double>>>> domainRanges;
		vector<vector<double>> staticRanges;

	public:
		ConstraintDescriptor(vector<shared_ptr<Variable>> vars, vector<vector<vector<shared_ptr<Term>>>> domVars);

		bool getSetsUtilitySignificanceThreshold();
		void setSetsUtilitySignificanceThreshold(bool value);
		double getUtilitySignificanceThreshold();
		void setUtilitySignificanceThreshold(double value);
//		void setFixedValue(shared_ptr<Term> variable, object value);
//		object getFixedValue(shared_ptr<Term> variable);
		shared_ptr<Term> getConstraint();
		void setConstraint(shared_ptr<Term> value);
		shared_ptr<Term> getUtility();
		void setUtility(shared_ptr<Term> value);
		double getUtilitySufficiencyThreshold();
		void setUtilitySufficiencyThreshold(double value);
		vector<shared_ptr<Term>> getStaticVars();
		void setStaticVars(vector<shared_ptr<Term>> value);
		vector<vector<vector<shared_ptr<Term>>>> getDomainVars();
		void setDomainVars(vector<vector<vector<shared_ptr<Term>>>> value);
//		vector<ICollection<int>> getAgentsInScope();
//		void setAgentsInScope(vector<ICollection<int>> value);
		vector<shared_ptr<Term>> getAllVars();
		void setAllVars(vector<shared_ptr<Term>> value);

		vector<vector<double>> allRanges();

		vector<vector<vector<vector<double>>>> getDomainRanges();
		void setDomainRanges(vector<vector<vector<vector<double>>>> value);
		vector<vector<double>> getStaticRanges();
		void setStaticRanges(vector<vector<double>> value);
	};

} /* namespace Alica */

#endif /* CONSTRAINTDESCRIPTOR_H_ */
