/*
 * Abs.cpp
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#include "Abs.h"

#include "TermBuilder.h"
#include "Constant.h"

#include <cmath>

namespace autodiff
{
	Abs::Abs(shared_ptr<Term> arg) :
			Term()
	{
		_arg = arg;
	}

	int Abs::accept(shared_ptr<ITermVisitor> visitor)
	{
		shared_ptr<Abs> thisCasted = dynamic_pointer_cast<Abs>(shared_from_this());
		return visitor->visit(thisCasted);
	}

	const shared_ptr<Term> Abs::getArg()
	{
		return _arg;
	}

	shared_ptr<Term> Abs::aggregateConstants()
	{
		_arg = _arg->aggregateConstants();
		if (dynamic_pointer_cast<Constant>(_arg) != 0) {
			shared_ptr<Constant> arg = dynamic_pointer_cast<Constant>(_arg);
			return TermBuilder::constant(fabs(arg->getValue()));
		} else {
			return shared_from_this();
		}
	}
	shared_ptr<Term> Abs::derivative(shared_ptr<Variable> v)
	{
		return _arg->derivative(v) * _arg / shared_from_this();
	}
} /* namespace autodiff */
