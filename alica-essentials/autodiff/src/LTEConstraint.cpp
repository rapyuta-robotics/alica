/*
 * LTEConstraint.cpp
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#include "LTEConstraint.h"

#include "TermBuilder.h"
#include "Constant.h"
#include "LTConstraint.h"

#include <cmath>

namespace AutoDiff
{
	LTEConstraint::LTEConstraint(shared_ptr<Term> x, shared_ptr<Term> y, double steppness) :
			Term()
	{
		_x = x;
		_y = y;
		_steppness = steppness;
//		_negatedform = 0;
		_negatedform = make_shared<LTConstraint>(_x, _y, _steppness, shared_from_this());
	}

	LTEConstraint::LTEConstraint(shared_ptr<Term> x, shared_ptr<Term> y, double steppness, shared_ptr<Term> negatedForm) :
			Term()
	{
		_x = x;
		_y = y;
		_steppness = steppness;
		_negatedform = negatedForm;
	}

	int LTEConstraint::accept(shared_ptr<ITermVisitor> visitor)
	{
		shared_ptr<LTEConstraint> thisCasted = dynamic_pointer_cast<LTEConstraint>(shared_from_this());
		return visitor->visit(thisCasted);
	}

	shared_ptr<Term> LTEConstraint::aggregateConstants()
	{
		_x = _x->aggregateConstants();
		_y = _y->aggregateConstants();
		if (dynamic_pointer_cast<Constant>(_x) != 0 && dynamic_pointer_cast<Constant>(_y) != 0)
		{
			shared_ptr<Constant> x = dynamic_pointer_cast<Constant>(_x);
			shared_ptr<Constant> y = dynamic_pointer_cast<Constant>(_y);
			if (x->getValue() <= y->getValue())
			{
				return Term::TRUE;
			}
			else
			{
				return Term::FALSE;
			}
		}
		else
		{
			return shared_from_this();
		}
	}

	shared_ptr<Term> LTEConstraint::derivative(shared_ptr<Variable> v)
	{
		throw "Symbolic Derivation of Less-Than-Or-Equal not supported.";
	}

	shared_ptr<Term> LTEConstraint::negate()
	{
//		if (_negatedform == 0)
//		{
//			_negatedform = make_shared<LTConstraint>(_x, _y, _steppness, shared_from_this());
//		}
		return _negatedform;
	}

	const shared_ptr<Term> LTEConstraint::getX()
	{
		return _x;
	}

	const shared_ptr<Term> LTEConstraint::getY()
	{
		return _y;
	}

	const double LTEConstraint::getSteppness()
	{
		return _steppness;
	}

	const shared_ptr<Term> LTEConstraint::getNegatedForm()
	{
		return _negatedform;
	}
} /* namespace AutoDiff */
