/*
 * Constant.cpp
 *
 *  Created on: Jun 12, 2014
 *      Author: psp
 */

#include "Constant.h"

#include "TermBuilder.h"
#include "Constant.h"

#include <cmath>

namespace AutoDiff
{

	Constant::Constant(double value) :
			Term()
	{
		_value = value;
	}

	int Constant::accept(shared_ptr<ITermVisitor> visitor)
	{
		shared_ptr<Constant> thisCasted = dynamic_pointer_cast<Constant>(shared_from_this());
		return visitor->visit(thisCasted);
	}

	shared_ptr<Term> Constant::aggregateConstants()
	{
		return shared_from_this();
	}

	shared_ptr<Term> Constant::derivative(shared_ptr<Variable> v)
	{
		return 0;
	}

	double Constant::getValue()
	{
		return _value;
	}

} /* namespace AutoDiff */
