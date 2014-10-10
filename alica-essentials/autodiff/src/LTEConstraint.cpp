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

namespace autodiff
{
	LTEConstraint::LTEConstraint(shared_ptr<Term> x, shared_ptr<Term> y, double steppness) :
			Term()
	{
		_left = x;
		_right = y;
		_steppness = steppness;
		_negatedform = nullptr;
	}

	LTEConstraint::LTEConstraint(shared_ptr<Term> x, shared_ptr<Term> y, double steppness, shared_ptr<Term> negatedForm) :
			Term()
	{
		_left = x;
		_right = y;
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
		_left = _left->aggregateConstants();
		_right = _right->aggregateConstants();
		if (dynamic_pointer_cast<Constant>(_left) != 0 && dynamic_pointer_cast<Constant>(_right) != 0)
		{
			shared_ptr<Constant> x = dynamic_pointer_cast<Constant>(_left);
			shared_ptr<Constant> y = dynamic_pointer_cast<Constant>(_right);
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
		if (!(_negatedform != nullptr))
		{
			_negatedform = make_shared<LTConstraint>(_left, _right, _steppness, shared_from_this());
		}
		return _negatedform;
	}

	const shared_ptr<Term> LTEConstraint::getLeft()
	{
		return _left;
	}

	const shared_ptr<Term> LTEConstraint::getRight()
	{
		return _right;
	}

	const double LTEConstraint::getSteppness()
	{
		return _steppness;
	}

	const shared_ptr<Term> LTEConstraint::getNegatedForm()
	{
		return _negatedform;
	}
} /* namespace autodiff */
