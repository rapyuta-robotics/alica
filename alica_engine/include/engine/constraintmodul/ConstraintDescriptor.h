/*
 * ConstraintDescriptor.h
 *
 *  Created on: Sep 30, 2014
 *      Author: psp
 */

#ifndef CONSTRAINTDESCRIPTOR_H_
#define CONSTRAINTDESCRIPTOR_H_

#include <memory>
#include <vector>

using namespace std;

namespace alica
{
	class SolverTerm;
	class SolverVariable;

	class ConstraintDescriptor : public enable_shared_from_this<ConstraintDescriptor>
	{
	private:
		int dim;
		const double min = -10E29;
		const double max = 10E29;

		double utilitySignificanceThreshold = 1E-22;
		bool setsUtilitySignificanceThreshold;

// TODO:		map<int, object> fixedValues;

		shared_ptr<SolverTerm> constraint;
		shared_ptr<SolverTerm> utility;
		double utilitySufficiencyThreshold;
		vector<shared_ptr<SolverTerm>> staticVars;
		vector<vector<vector<shared_ptr<SolverTerm>>>> domainVars;
		vector<vector<int>> agentsInScope;
		vector<shared_ptr<SolverTerm>> allVars;

		vector<vector<vector<vector<double>>>> domainRanges;
		vector<vector<double>> staticRanges;

	public:
		ConstraintDescriptor(vector<shared_ptr<SolverVariable>> vars, vector<vector<vector<shared_ptr<SolverTerm>>>> domVars);

		bool getSetsUtilitySignificanceThreshold();
		void setSetsUtilitySignificanceThreshold(bool value);
		double getUtilitySignificanceThreshold();
		void setUtilitySignificanceThreshold(double value);
//		void setFixedValue(shared_ptr<SolverTerm> SolverVariable, object value);
//		object getFixedValue(shared_ptr<SolverTerm> SolverVariable);
		shared_ptr<SolverTerm> getConstraint();
		void setConstraint(shared_ptr<SolverTerm> value);
		shared_ptr<SolverTerm> getUtility();
		void setUtility(shared_ptr<SolverTerm> value);
		double getUtilitySufficiencyThreshold();
		void setUtilitySufficiencyThreshold(double value);
		vector<shared_ptr<SolverTerm>> getStaticVars();
		void setStaticVars(vector<shared_ptr<SolverTerm>> value);
		vector<vector<vector<shared_ptr<SolverTerm>>>> getDomainVars();
		void setDomainVars(vector<vector<vector<shared_ptr<SolverTerm>>>> value);
		vector<vector<int>> getAgentsInScope();
		void setAgentsInScope(vector<vector<int>> value);
		vector<shared_ptr<SolverTerm>> getAllVars();
		void setAllVars(vector<shared_ptr<SolverTerm>> value);

		vector<vector<double>> allRanges();

		vector<vector<vector<vector<double>>>> getDomainRanges();
		void setDomainRanges(vector<vector<vector<vector<double>>>> value);
		vector<vector<double>> getStaticRanges();
		void setStaticRanges(vector<vector<double>> value);
	};

} /* namespace Alica */

#endif /* CONSTRAINTDESCRIPTOR_H_ */
