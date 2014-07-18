/*
 * Sin.cpp
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#include "Sin.h"

#include "TermBuilder.h"
#include "Constant.h"
#include "Cos.h"
#include "Zero.h"

#include <cmath>

namespace AutoDiff
{
	Sin::Sin(shared_ptr<Term> arg) :
			Term()
	{
		_arg = arg;
	}

	int Sin::accept(shared_ptr<ITermVisitor> visitor)
	{
		shared_ptr<Sin> thisCasted = dynamic_pointer_cast<Sin>(shared_from_this());
		return visitor->visit(thisCasted);
	}

	shared_ptr<Term> Sin::aggregateConstants()
	{
		_arg = _arg->aggregateConstants();
		if (dynamic_pointer_cast<Constant>(_arg) != 0)
		{
			shared_ptr<Constant> arg = dynamic_pointer_cast<Constant>(_arg);
			return TermBuilder::constant(sin(arg->getValue()));
		} else {
			if (dynamic_pointer_cast<Zero>(_arg) != 0) {
				return TermBuilder::constant(0);
			}
			return shared_from_this();
		}
	}

	shared_ptr<Term> Sin::derivative(shared_ptr<Variable> v)
	{
		return make_shared<Cos>(_arg) * _arg->derivative(v);
	}

	const shared_ptr<Term> Sin::getArg()
	{
		return _arg;
	}
} /* namespace AutoDiff */
