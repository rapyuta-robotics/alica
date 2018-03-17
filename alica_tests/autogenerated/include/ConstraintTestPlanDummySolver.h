#pragma once

#include <engine/constraintmodul/ISolver.h>
#include <vector>
#include <memory>

namespace alica
{
	namespace reasoner
	{

		class ConstraintTestPlanDummySolver : public ISolver
		{
		public:
			ConstraintTestPlanDummySolver(AlicaEngine *ae);
			virtual ~ConstraintTestPlanDummySolver();

			bool existsSolution(std::vector<Variable*>& vars, std::vector<std::shared_ptr<ProblemDescriptor>>& calls);
			bool getSolution(std::vector<Variable*>& vars, std::vector<std::shared_ptr<ProblemDescriptor>>& calls,
								std::vector<void*>& results);
			std::shared_ptr<SolverVariable> createVariable(long id);

			static int getExistsSolutionCallCounter();
			static int getGetSolutionCallCounter();
		private:
			static int existsSolutionCallCounter;
			static int getSolutionCallCounter;
		};
	} /* namespace reasoner */

} /* namespace alica */
