/*
 * LinSigmoid.cpp
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#include "LinSigmoid.h"

#include "TermBuilder.h"
#include "Constant.h"

#include <cmath>
#include <limits>

namespace autodiff
{
	LinSigmoid::LinSigmoid(shared_ptr<Term> arg) :
			Term()
	{
		this->arg = arg;
	}

	int LinSigmoid::accept(shared_ptr<ITermVisitor> visitor)
	{
		shared_ptr<LinSigmoid> thisCasted = dynamic_pointer_cast<LinSigmoid>(shared_from_this());
		return visitor->visit(thisCasted);
	}

	shared_ptr<Term> LinSigmoid::aggregateConstants()
	{
		arg = arg->aggregateConstants();
		if (dynamic_pointer_cast<Constant>(arg) != 0) {
			shared_ptr<Constant> arg = dynamic_pointer_cast<Constant>(arg);
			double e = exp(-arg->value);
			if (e == numeric_limits<double>::infinity()) {
				return TermBuilder::constant(Term::EPSILON);
			} else {
				e = 1.0 / (1.0 + e);
			}
			if (e < Term::EPSILON) {
				return TermBuilder::constant(Term::EPSILON);
			} else {
				return TermBuilder::constant(e);
			}
		} else {
			return shared_from_this();
		}
	}

	shared_ptr<Term> LinSigmoid::derivative(shared_ptr<Variable> v)
	{
		return arg->derivative(v);
	}
} /* namespace autodiff */
