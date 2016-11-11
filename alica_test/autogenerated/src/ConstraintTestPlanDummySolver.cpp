/*
 * ConstraintTestPlanDummySolver.cpp
 *
 *  Created on: Nov 24, 2014
 *      Author: Philipp
 */

#include "ConstraintTestPlanDummySolver.h"
#include <engine/model/Variable.h>
#include <engine/constraintmodul/SolverVariable.h>

#include <iostream>

namespace alica
{
	namespace reasoner
	{
		int ConstraintTestPlanDummySolver::existsSolutionCallCounter = 0;
		int ConstraintTestPlanDummySolver::getSolutionCallCounter = 0;

		ConstraintTestPlanDummySolver::ConstraintTestPlanDummySolver(AlicaEngine *ae) :
				ISolver(ae)
		{
			// TODO Auto-generated constructor stub

		}

		ConstraintTestPlanDummySolver::~ConstraintTestPlanDummySolver()
		{
			// TODO Auto-generated destructor stub
		}

		bool ConstraintTestPlanDummySolver::existsSolution(vector<Variable*>& vars,
															vector<shared_ptr<ProblemDescriptor>>& calls)
		{
			existsSolutionCallCounter++;
			//std::cout << "ConstraintTestPlanDummySolver::existsSolution was called " << existsSolutionCallCounter
			//		<< " times!" << std::endl;
			return false;
		}

		bool ConstraintTestPlanDummySolver::getSolution(vector<Variable*>& vars,
														vector<shared_ptr<ProblemDescriptor>>& calls,
														vector<void*>& results)
		{
			for (int i = 0; i < vars.size(); i++)
			{
				string* s = new string(vars.at(i)->getName());
				results.push_back(s);
			}
			getSolutionCallCounter++;
			//std::cout << "ConstraintTestPlanDummySolver::getSolution was called " << getSolutionCallCounter << " times!"
			//		<< std::endl;
			return true;
		}

		int ConstraintTestPlanDummySolver::getExistsSolutionCallCounter()
		{
			return existsSolutionCallCounter;
		}

		int ConstraintTestPlanDummySolver::getGetSolutionCallCounter()
		{
			return getSolutionCallCounter;
		}

		shared_ptr<SolverVariable> ConstraintTestPlanDummySolver::createVariable(long id) {
			return make_shared<SolverVariable>();

		}
	}

} /* namespace alica */
