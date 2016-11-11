/*
 * ConstraintTestPlanDummySolver.h
 *
 *  Created on: Nov 24, 2014
 *      Author: Philipp
 */

#ifndef CONSTRAINTTESTPLANDUMMYSOLVER_H_
#define CONSTRAINTTESTPLANDUMMYSOLVER_H_

#include <engine/constraintmodul/ISolver.h>

namespace alica
{
	namespace reasoner
	{

		class ConstraintTestPlanDummySolver : public ISolver
		{
		public:
			ConstraintTestPlanDummySolver(AlicaEngine *ae);
			virtual ~ConstraintTestPlanDummySolver();

			bool existsSolution(vector<Variable*>& vars, vector<shared_ptr<ProblemDescriptor>>& calls);
			bool getSolution(vector<Variable*>& vars, vector<shared_ptr<ProblemDescriptor>>& calls,
								vector<void*>& results);
			shared_ptr<SolverVariable> createVariable(long id);

			static int getExistsSolutionCallCounter();
			static int getGetSolutionCallCounter();
		private:
			static int existsSolutionCallCounter;
			static int getSolutionCallCounter;
		};
	} /* namespace reasoner */

} /* namespace alica */

#endif /* CONSTRAINTTESTPLANDUMMYSOLVER_H_ */
