/*
 * ConstPower.cpp
 *
 *  Created on: Jun 16, 2014
 *      Author: psp
 */

#include "ConstPower.h"

#include "TermBuilder.h"
#include "Constant.h"
#include "Zero.h"

#include <cmath>

namespace autodiff
{
	ConstPower::ConstPower(shared_ptr<Term> baseTerm, double exponent) :
			Term()
	{
		this->base = baseTerm;
		this->exponent = exponent;
	}

	int ConstPower::accept(shared_ptr<ITermVisitor> visitor)
	{
		shared_ptr<ConstPower> thisCasted = dynamic_pointer_cast<ConstPower>(shared_from_this());
		return visitor->visit(thisCasted);
	}

	shared_ptr<Term> ConstPower::aggregateConstants()
	{
		base = base->aggregateConstants();
		if (dynamic_pointer_cast<Constant>(base) != 0)
		{
			shared_ptr<Constant> base = dynamic_pointer_cast<Constant>(base);
			return TermBuilder::constant(pow(base->value, exponent));
		}
		else if (dynamic_pointer_cast<Zero>(base) != 0)
		{
			if (exponent >= 0)
			{
				return base;
			}
			else
			{
				throw "Divide By Zero";
			}
		}
		else if (dynamic_pointer_cast<ConstPower>(base) != 0)
		{
			shared_ptr<ConstPower> base = dynamic_pointer_cast<ConstPower>(base);
			this->exponent *= base->exponent;
			this->base = base->base;
			return shared_from_this();
		}
		else
		{
			return shared_from_this();
		}
	}

	shared_ptr<Term> ConstPower::derivative(shared_ptr<Variable> v)
	{
		return TermBuilder::constant(exponent) * make_shared<ConstPower>(base, exponent - 1) * base->derivative(v);
	}
} /* namespace autodiff */
