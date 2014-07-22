/*
 * Cos.cpp
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#include "Cos.h"

#include "TermBuilder.h"
#include "Constant.h"
#include "Sin.h"

#include <cmath>

namespace AutoDiff
{
	Cos::Cos(shared_ptr<Term> arg) :
			Term()
	{
		_arg = arg;
	}

	int Cos::accept(shared_ptr<ITermVisitor> visitor)
	{
		shared_ptr<Cos> thisCasted = dynamic_pointer_cast<Cos>(shared_from_this());
		return visitor->visit(thisCasted);
	}

	shared_ptr<Term> Cos::aggregateConstants()
	{
		_arg = _arg->aggregateConstants();
		if (dynamic_pointer_cast<Constant>(_arg) != 0)
		{
			shared_ptr<Constant> arg = dynamic_pointer_cast<Constant>(_arg);
			return TermBuilder::constant(cos(arg->getValue()));
		}
		else
		{
			if (dynamic_pointer_cast<Constant>(_arg) != 0)
			{
				return TermBuilder::constant(1);
			}
			return shared_from_this();
		}
	}

	shared_ptr<Term> Cos::derivative(shared_ptr<Variable> v)
	{
		return make_shared<Sin>(_arg) * -_arg->derivative(v);
	}

	const shared_ptr<Term> Cos::getArg()
	{
		return _arg;
	}
} /* namespace AutoDiff */
