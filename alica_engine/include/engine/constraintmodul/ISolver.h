/*
 * IConstraintSolver.h
 *
 *  Created on: Sep 30, 2014
 *      Author: Philipp Sperber
 */

#ifndef ICONSTRAINTSOLVER_H_
#define ICONSTRAINTSOLVER_H_

#include <memory>
#include <vector>

using namespace std;

namespace alica
{
	class AlicaEngine;
	class ProblemDescriptor;
	class Variable;
	class SolverVariable;

	class ISolver : public enable_shared_from_this<ISolver>
	{
	public:
		ISolver(AlicaEngine* ae) {
			this->ae = ae;
		}
		virtual ~ISolver()
		{
		}

		virtual bool existsSolution(vector<Variable*>& vars, vector<shared_ptr<ProblemDescriptor>>& calls) = 0;
		virtual bool getSolution(vector<Variable*>& vars, vector<shared_ptr<ProblemDescriptor>>& calls, vector<void*>& results) = 0;
		virtual shared_ptr<SolverVariable> createVariable(long id) = 0;

		virtual double utilityEstimate(vector<Variable*>& vars, vector<shared_ptr<ProblemDescriptor>>& calls) {
			return 0;
		}

		AlicaEngine* getAlicaEngine() {
			return ae;
		}

	protected:
		AlicaEngine* ae;
	};

} /* namespace alica */

#endif /* ICONSTRAINTSOLVER_H_ */
