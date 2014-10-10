/*
 * Log.cpp
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#include "Log.h"

#include "TermBuilder.h"
#include "Constant.h"

#include <cmath>
#include <limits>

namespace autodiff
{
	Log::Log(shared_ptr<Term> arg) :
			Term()
	{
		_arg = arg;
	}

	int Log::accept(shared_ptr<ITermVisitor> visitor)
	{
		shared_ptr<Log> thisCasted = dynamic_pointer_cast<Log>(shared_from_this());
		return visitor->visit(thisCasted);
	}

	shared_ptr<Term> Log::aggregateConstants()
	{
		_arg = _arg->aggregateConstants();
		if (dynamic_pointer_cast<Constant>(_arg) != 0) {
			shared_ptr<Constant> arg = dynamic_pointer_cast<Constant>(_arg);
			return TermBuilder::constant(log(arg->getValue()));
		} else {
			return shared_from_this();
		}
	}

	shared_ptr<Term> Log::derivative(shared_ptr<Variable> v)
	{
		return _arg->derivative(v) / _arg;
	}

	const shared_ptr<Term> Log::getArg()
	{
		return _arg;
	}
} /* namespace autodiff */
