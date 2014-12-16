/*
 * Variable.h
 *
 *  Created on: Jun 5, 2014
 *      Author: psp
 */

#ifndef AutoDiffVARIABLE_H_
#define AutoDiffVARIABLE_H_

#include <engine/constraintmodul/SolverVariable.h>

#include "Term.h"

namespace autodiff
{
	class Variable : public Term, public alica::SolverVariable
	{
	public:
		double globalMin;
		double globalMax;

		Variable();

		int accept(shared_ptr<ITermVisitor> visitor);

		shared_ptr<Term> aggregateConstants();
		shared_ptr<Term> derivative(shared_ptr<Variable> v);
	};
} /* namespace autodiff */

#endif /* AutoDiffVARIABLE_H_ */
