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

namespace AutoDiff
{
	ConstPower::ConstPower(shared_ptr<Term> baseTerm, double exponent) :
			Term()
	{
		_base = baseTerm;
		_exponent = exponent;
	}

	int ConstPower::accept(shared_ptr<ITermVisitor> visitor)
	{
		shared_ptr<ConstPower> thisCasted = dynamic_pointer_cast<ConstPower>(shared_from_this());
		return visitor->visit(thisCasted);
	}

	shared_ptr<Term> ConstPower::aggregateConstants()
	{
		_base = _base->aggregateConstants();
		if (dynamic_pointer_cast<Constant>(_base) != 0)
		{
			shared_ptr<Constant> base = dynamic_pointer_cast<Constant>(_base);
			return TermBuilder::constant(pow(base->getValue(), _exponent));
		}
		else if (dynamic_pointer_cast<Zero>(_base) != 0)
		{
			if (_exponent >= 0)
			{
				return _base;
			}
			else
			{
				throw "Divide By Zero";
			}
		}
		else if (dynamic_pointer_cast<ConstPower>(_base) != 0)
		{
			shared_ptr<ConstPower> base = dynamic_pointer_cast<ConstPower>(_base);
			_exponent *= base->getExponent();
			_base = base->getBase();
			return shared_from_this();
		}
		else
		{
			return shared_from_this();
		}
	}

	shared_ptr<Term> ConstPower::derivative(shared_ptr<Variable> v)
	{
		return TermBuilder::constant(_exponent) * make_shared<ConstPower>(_base, _exponent - 1) * _base->derivative(v);
	}

	const shared_ptr<Term> ConstPower::getBase()
	{
		return _base;
	}

	const double ConstPower::getExponent()
	{
		return _exponent;
	}
} /* namespace AutoDiff */
