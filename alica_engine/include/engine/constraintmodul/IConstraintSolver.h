/*
 * IConstraintSolver.h
 *
 *  Created on: Sep 30, 2014
 *      Author: psp
 */

#ifndef ICONSTRAINTSOLVER_H_
#define ICONSTRAINTSOLVER_H_

#include <memory>
#include <vector>

using namespace std;

namespace alica
{
	class ConstraintDescriptor;
	class Variable;

	class IConstraintSolver : public enable_shared_from_this<IConstraintSolver>
	{
	public:
		virtual ~IConstraintSolver()
		{
		}

		virtual bool existsSolution(vector<Variable*> vars, vector<shared_ptr<ConstraintDescriptor>> calls) = 0;
		virtual bool getSolution(vector<Variable*> vars, vector<shared_ptr<ConstraintDescriptor>> calls, vector<double>* results) = 0;
//		virtual bool getSolution(vector<Variable*> vars, vector<shared_ptr<ConstraintDescriptor>> calls, vector<object>& results) = 0;
	};

} /* namespace alica */

#endif /* ICONSTRAINTSOLVER_H_ */
