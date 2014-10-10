/*
 * IConstraintSolver.h
 *
 *  Created on: Sep 30, 2014
 *      Author: psp
 */

#ifndef ICONSTRAINTSOLVER_H_
#define ICONSTRAINTSOLVER_H_

#include <AutoDiff.h>

#include <memory>

using namespace autodiff;

namespace alica
{
	class ConstraintDescriptor;

	class IConstraintSolver : public enable_shared_from_this<IConstraintSolver>
	{
	public:
		virtual ~IConstraintSolver();

		virtual bool existsSolution(vector<shared_ptr<Variable>> vars, vector<shared_ptr<ConstraintDescriptor>> calls) = 0;
		virtual bool getSolution(vector<shared_ptr<Variable>> vars, vector<shared_ptr<ConstraintDescriptor>> calls, vector<double>& results) = 0;
//		virtual bool getSolution(vector<shared_ptr<Variable>> vars, vector<shared_ptr<ConstraintDescriptor>> calls, vector<object>& results) = 0;
	};

} /* namespace alica */

#endif /* ICONSTRAINTSOLVER_H_ */
