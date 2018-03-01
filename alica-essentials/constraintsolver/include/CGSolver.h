#pragma once

#include <AutoDiff.h>
#include <engine/constraintmodul/ISolver.h>
#include <vector>
#include <memory>
#include <mutex>

//#define CGSolver_DEBUG

namespace alica
{
	class AlicaEngine;
	class IVariableSyncModule;

	namespace reasoner
	{
		class GSolver;

		class CGSolver : public ISolver
		{
		public:
			CGSolver(AlicaEngine* ae);
			virtual ~CGSolver();

			bool existsSolution(vector<Variable*>& vars, vector<shared_ptr<ProblemDescriptor>>& calls);
			bool getSolution(vector<Variable*>& vars, vector<shared_ptr<ProblemDescriptor>>& calls, vector<void*>& results);
			shared_ptr<SolverVariable> createVariable(long id);

		protected:
			shared_ptr<GSolver> gs;
			shared_ptr<GSolver> sgs;

			mutex mtx;

			double lastUtil;
			double lastRuns;
			double lastFEvals;
		};

	} /* namespace Reasoner */
} /* namespace Alica */
