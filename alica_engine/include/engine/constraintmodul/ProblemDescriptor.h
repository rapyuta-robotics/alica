/*
 * ProblemDescriptor.h
 *
 *  Created on: Sep 30, 2014
 *      Author: Philipp Sperber
 */

#ifndef PROBLEMDESCRIPTOR_H_
#define PROBLEMDESCRIPTOR_H_

#include <memory>
#include <vector>

using namespace std;

namespace alica
{
	class SolverTerm;
	class SolverVariable;

	class ProblemDescriptor : public enable_shared_from_this<ProblemDescriptor>
	{
	public:
		ProblemDescriptor(shared_ptr<vector<shared_ptr<SolverVariable>>> vars, shared_ptr<vector<shared_ptr<vector<shared_ptr<vector<shared_ptr<SolverVariable>>>>> >> domVars);

		bool getSetsUtilitySignificanceThreshold();
		void setSetsUtilitySignificanceThreshold(bool value);
		double getUtilitySignificanceThreshold();
		void setUtilitySignificanceThreshold(double value);
		shared_ptr<SolverTerm> getConstraint();
		void setConstraint(shared_ptr<SolverTerm> value);
		shared_ptr<SolverTerm> getUtility();
		void setUtility(shared_ptr<SolverTerm> value);
		double getUtilitySufficiencyThreshold();
		void setUtilitySufficiencyThreshold(double value);
		shared_ptr<vector<shared_ptr<SolverVariable>>> getStaticVars();
		void setStaticVars(shared_ptr<vector<shared_ptr<SolverVariable>>> value);
		shared_ptr<vector<shared_ptr<vector<shared_ptr<vector<shared_ptr<SolverVariable>>>>> >> getDomainVars();
		void setDomainVars(shared_ptr<vector<shared_ptr<vector<shared_ptr<vector<shared_ptr<SolverVariable>>>>> >> value);
		shared_ptr<vector<shared_ptr<vector<int>>>> getAgentsInScope();
		void setAgentsInScope(shared_ptr<vector<shared_ptr<vector<int>>>> value);
		shared_ptr<vector<shared_ptr<SolverVariable>>> getAllVars();
		void setAllVars(shared_ptr<vector<shared_ptr<SolverVariable>>> value);

		shared_ptr<vector<vector<double>>> allRanges();

		shared_ptr<vector<vector<vector<vector<double>>>>> getDomainRanges();
		void setDomainRanges(shared_ptr<vector<vector<vector<vector<double>>>>> value);
		shared_ptr<vector<vector<double>>> getStaticRanges();
		void setStaticRanges(shared_ptr<vector<vector<double>>> value);

	private:
		int dim;
		const double min = -10E29;
		const double max = 10E29;

		double utilitySignificanceThreshold = 1E-22; /*<< minimum delta for adapting a better utility */
		bool setsUtilitySignificanceThreshold;

		shared_ptr<SolverTerm> constraint;
		shared_ptr<SolverTerm> utility;
		double utilitySufficiencyThreshold;
		shared_ptr<vector<shared_ptr<SolverVariable>>> staticVars;
		shared_ptr<vector<shared_ptr<vector<shared_ptr<vector<shared_ptr<SolverVariable>>>>> >> domainVars;
		shared_ptr<vector<shared_ptr<vector<int>>>> agentsInScope;
		shared_ptr<vector<shared_ptr<SolverVariable>>> allVars;

		shared_ptr<vector<vector<vector<vector<double>>>>> domainRanges;
		shared_ptr<vector<vector<double>>> staticRanges;
	};

} /* namespace Alica */

#endif /* PROBLEMDESCRIPTOR_H_ */
