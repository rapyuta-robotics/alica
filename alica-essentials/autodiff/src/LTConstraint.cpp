/*
 * LTConstraint.cpp
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#include "LTConstraint.h"

#include "TermBuilder.h"
#include "Constant.h"
#include "LTEConstraint.h"

#include <cmath>

namespace AutoDiff
{
	LTConstraint::LTConstraint(shared_ptr<Term> x, shared_ptr<Term> y, double steppness) :
			Term()
	{
		_left = x;
		_right = y;
		_steppness = steppness;
		_negatedform = nullptr;
	}

	LTConstraint::LTConstraint(shared_ptr<Term> x, shared_ptr<Term> y, double steppness, shared_ptr<Term> negatedForm) :
			Term()
	{
		_left = x;
		_right = y;
		_steppness = steppness;
		_negatedform = negatedForm;
	}

	int LTConstraint::accept(shared_ptr<ITermVisitor> visitor)
	{
		shared_ptr<LTConstraint> thisCasted = dynamic_pointer_cast<LTConstraint>(shared_from_this());
		return visitor->visit(thisCasted);
	}

	shared_ptr<Term> LTConstraint::aggregateConstants()
	{
		_left = _left->aggregateConstants();
		_right = _right->aggregateConstants();
		if (dynamic_pointer_cast<Constant>(_left) != 0 && dynamic_pointer_cast<Constant>(_right) != 0)
		{
			shared_ptr<Constant> x = dynamic_pointer_cast<Constant>(_left);
			shared_ptr<Constant> y = dynamic_pointer_cast<Constant>(_right);
			if (x->getValue() < y->getValue())
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

	shared_ptr<Term> LTConstraint::derivative(shared_ptr<Variable> v)
	{
		throw "Symbolic Derivation of Less-Than not supported.";
	}

	shared_ptr<Term> LTConstraint::negate()
	{
		if (!(_negatedform != nullptr))
		{
			_negatedform = make_shared<LTEConstraint>(_left, _right, _steppness, shared_from_this());
		}
		return _negatedform;
	}

	const shared_ptr<Term> LTConstraint::getLeft()
	{
		return _left;
	}

	const shared_ptr<Term> LTConstraint::getRight()
	{
		return _right;
	}

	const double LTConstraint::getSteppness()
	{
		return _steppness;
	}

	const shared_ptr<Term> LTConstraint::getNegatedForm()
	{
		return _negatedform;
	}
} /* namespace AutoDiff */
