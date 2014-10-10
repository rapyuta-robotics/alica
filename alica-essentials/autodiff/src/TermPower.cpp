/*
 * TermPower.cpp
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#include "TermPower.h"

#include "TermBuilder.h"
#include "Constant.h"
#include "Log.h"
#include "Zero.h"

#include <cmath>

namespace autodiff
{
	TermPower::TermPower(shared_ptr<Term> baseTerm, shared_ptr<Term> exponent) :
			Term()
	{
		_base = baseTerm;
		_exponent = exponent;
	}

	int TermPower::accept(shared_ptr<ITermVisitor> visitor)
	{
		shared_ptr<TermPower> thisCasted = dynamic_pointer_cast<TermPower>(shared_from_this());
		return visitor->visit(thisCasted);
	}

	shared_ptr<Term> TermPower::aggregateConstants()
	{
		_base = _base->aggregateConstants();
		_exponent = _exponent->aggregateConstants();
		if (dynamic_pointer_cast<Zero>(_exponent) != 0)
		{
			return TermBuilder::constant(1);
		}
		if (dynamic_pointer_cast<Constant>(_base) != 0 && dynamic_pointer_cast<Constant>(_exponent) != 0)
		{
			shared_ptr<Constant> base = dynamic_pointer_cast<Constant>(_base);
			shared_ptr<Constant> exponent = dynamic_pointer_cast<Constant>(_exponent);
			return TermBuilder::constant(pow(base->getValue(), exponent->getValue()));
		} else if (dynamic_pointer_cast<Zero>(_base) != 0) {
			return _base;
		} else if (dynamic_pointer_cast<TermPower>(_base) != 0) {
			shared_ptr<TermPower> base = dynamic_pointer_cast<TermPower>(_base);
			_exponent = _exponent * base->getExponent();
			_base = base->getBase();
			return shared_from_this();
		} else {
			return shared_from_this();
		}
	}

	shared_ptr<Term> TermPower::derivative(shared_ptr<Variable> v)
	{
		return make_shared<TermPower>(_base, _exponent - 1)
				* (_exponent * _base->derivative(v) + _base * make_shared<Log>(_base) * _exponent->derivative(v));
	}

	const shared_ptr<Term> TermPower::getBase()
	{
		return _base;
	}

	const shared_ptr<Term> TermPower::getExponent()
	{
		return _exponent;
	}
} /* namespace autodiff */
