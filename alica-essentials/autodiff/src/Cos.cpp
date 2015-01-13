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

namespace autodiff
{
	Cos::Cos(shared_ptr<Term> arg) :
			Term()
	{
		this->arg = arg;
	}

	int Cos::accept(shared_ptr<ITermVisitor> visitor)
	{
		shared_ptr<Cos> thisCasted = dynamic_pointer_cast<Cos>(shared_from_this());
		return visitor->visit(thisCasted);
	}

	shared_ptr<Term> Cos::aggregateConstants()
	{
		arg = arg->aggregateConstants();
		if (dynamic_pointer_cast<Constant>(arg) != 0)
		{
			shared_ptr<Constant> arg = dynamic_pointer_cast<Constant>(arg);
			return TermBuilder::constant(cos(arg->value));
		}
		else
		{
			if (dynamic_pointer_cast<Constant>(arg) != 0)
			{
				return TermBuilder::constant(1);
			}
			return shared_from_this();
		}
	}

	shared_ptr<Term> Cos::derivative(shared_ptr<Variable> v)
	{
		return make_shared<Sin>(arg) * -arg->derivative(v);
	}
} /* namespace autodiff */
