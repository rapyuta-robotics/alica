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
		shared_ptr<vector<shared_ptr<SolverTerm>>> staticVars;
		shared_ptr<vector<shared_ptr<vector<shared_ptr<vector<shared_ptr<SolverTerm>>>>> >> domainVars;
		shared_ptr<vector<shared_ptr<vector<int>>>> agentsInScope;
		shared_ptr<vector<shared_ptr<SolverTerm>>> allVars;

		shared_ptr<vector<vector<vector<vector<double>>>>> domainRanges;
		shared_ptr<vector<vector<double>>> staticRanges;

	public:
		ConstraintDescriptor(shared_ptr<vector<shared_ptr<SolverVariable>>> vars, shared_ptr<vector<shared_ptr<vector<shared_ptr<vector<shared_ptr<SolverTerm>>>>> >> domVars);

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
		shared_ptr<vector<shared_ptr<SolverTerm>>> getStaticVars();
		void setStaticVars(shared_ptr<vector<shared_ptr<SolverTerm>>> value);
		shared_ptr<vector<shared_ptr<vector<shared_ptr<vector<shared_ptr<SolverTerm>>>>> >> getDomainVars();
		void setDomainVars(shared_ptr<vector<shared_ptr<vector<shared_ptr<vector<shared_ptr<SolverTerm>>>>> >> value);
		shared_ptr<vector<shared_ptr<vector<int>>>> getAgentsInScope();
		void setAgentsInScope(shared_ptr<vector<shared_ptr<vector<int>>>> value);
		shared_ptr<vector<shared_ptr<SolverTerm>>> getAllVars();
		void setAllVars(shared_ptr<vector<shared_ptr<SolverTerm>>> value);

		shared_ptr<vector<vector<double>>> allRanges();

		shared_ptr<vector<vector<vector<vector<double>>>>> getDomainRanges();
		void setDomainRanges(shared_ptr<vector<vector<vector<vector<double>>>>> value);
		shared_ptr<vector<vector<double>>> getStaticRanges();
		void setStaticRanges(shared_ptr<vector<vector<double>>> value);
	};

} /* namespace Alica */

#endif /* CONSTRAINTDESCRIPTOR_H_ */
