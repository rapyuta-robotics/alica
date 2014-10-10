/*
 * IConstraintSolver.h
 *
 *  Created on: Sep 30, 2014
 *      Author: psp
 */

#ifndef ICONSTRAINTSOLVER_H_
#define ICONSTRAINTSOLVER_H_

#include <AutoDiff.h>
#include "ConstraintDescriptor.h"

#include <memory>

using namespace AutoDiff;


namespace Alica
{
	class IConstraintSolver
	{
	public:
		inline bool existsSolution(vector<shared_ptr<Variable>> vars, vector<shared_ptr<ConstraintDescriptor>> calls);
		inline bool getSolution(vector<shared_ptr<Variable>> vars, vector<shared_ptr<ConstraintDescriptor>> calls, vector<double>& results);
	};

} /* namespace Alica */

#endif /* ICONSTRAINTSOLVER_H_ */
