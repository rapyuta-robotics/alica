/*
 * ConstraintTestPlanDummySolver.cpp
 *
 *  Created on: Nov 24, 2014
 *      Author: Philipp
 */

#include "ConstraintTestPlanDummySolver.h"

#include <iostream>

namespace alica
{
	namespace reasoner
	{
		int ConstraintTestPlanDummySolver::existsSolutionCallCounter = 0;
		int ConstraintTestPlanDummySolver::getSolutionCallCounter = 0;

		ConstraintTestPlanDummySolver::ConstraintTestPlanDummySolver()
		{
			// TODO Auto-generated constructor stub

		}

		ConstraintTestPlanDummySolver::~ConstraintTestPlanDummySolver()
		{
			// TODO Auto-generated destructor stub
		}

		bool ConstraintTestPlanDummySolver::existsSolution(vector<Variable*> vars,
															vector<shared_ptr<ConstraintDescriptor>> calls)
		{
			existsSolutionCallCounter++;
			std::cout << "ConstraintTestPlanDummySolver::existsSolution was called " << existsSolutionCallCounter
					<< " times!" << std::endl;
			return false;
		}

		bool ConstraintTestPlanDummySolver::getSolution(vector<Variable*> vars,
														vector<shared_ptr<ConstraintDescriptor>> calls,
														vector<double>* results)
		{
			getSolutionCallCounter++;
			std::cout << "ConstraintTestPlanDummySolver::getSolution was called " << getSolutionCallCounter << " times!"
					<< std::endl;
			return false;
		}

		int ConstraintTestPlanDummySolver::getExistsSolutionCallCounter()
		{
			return existsSolutionCallCounter;
		}

		int ConstraintTestPlanDummySolver::getGetSolutionCallCounter()
		{
			return getSolutionCallCounter;
		}
	}

} /* namespace alica */
