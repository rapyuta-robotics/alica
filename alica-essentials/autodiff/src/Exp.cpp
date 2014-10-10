/*
 * Exp.cpp
 *
 *  Created on: Jun 12, 2014
 *      Author: psp
 */

#include "Exp.h"

#include "TermBuilder.h"
#include "Constant.h"

#include <cmath>

namespace autodiff
{

	Exp::Exp(shared_ptr<Term> arg) :
			Term()
	{
		_arg = arg;
	}

	int Exp::accept(shared_ptr<ITermVisitor> visitor)
	{
		shared_ptr<Exp> thisCasted = dynamic_pointer_cast<Exp>(shared_from_this());
		return visitor->visit(thisCasted);
	}

	shared_ptr<Term> Exp::aggregateConstants()
	{
		_arg = _arg->aggregateConstants();
		if (dynamic_pointer_cast<Constant>(_arg) != 0)
		{
			shared_ptr<Constant> arg = dynamic_pointer_cast<Constant>(_arg);
			return TermBuilder::constant(exp(arg->getValue()));
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

	shared_ptr<Term> Exp::derivative(shared_ptr<Variable> v)
	{
		return shared_from_this() * _arg->derivative(v);
	}

	shared_ptr<Term> Exp::getArg()
	{
		return _arg;
	}

} /* namespace autodiff */
