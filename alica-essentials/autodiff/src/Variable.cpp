/*
 * Variable.cpp
 *
 *  Created on: Jun 5, 2014
 *      Author: psp
 */

#include "Variable.h"

#include "TermBuilder.h"

namespace autodiff
{
	int Variable::accept(shared_ptr<ITermVisitor> visitor) {
		shared_ptr<Variable> thisCasted = dynamic_pointer_cast<Variable>(shared_from_this());
		return visitor->visit(thisCasted);
	}

	shared_ptr<Term> Variable::aggregateConstants()
	{
		return shared_from_this();
	}

	shared_ptr<Term> Variable::derivative(shared_ptr<Variable> v)
	{
		if (shared_from_this() == v) {
			return TermBuilder::constant(1);
		} else {
			return TermBuilder::constant(0);
		}
	}
} /* namespace autodiff */
