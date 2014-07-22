/*
 * Min.cpp
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#include "Min.h"

#include "TermBuilder.h"
#include "Constant.h"

#include <cmath>

namespace AutoDiff
{
	Min::Min(shared_ptr<Term> left, shared_ptr<Term> right) :
			Term()
	{
		_left = left;
		_right = right;
	}

	int Min::accept(shared_ptr<ITermVisitor> visitor)
	{
		shared_ptr<Min> thisCasted = dynamic_pointer_cast<Min>(shared_from_this());
		return visitor->visit(thisCasted);
	}

	shared_ptr<Term> Min::aggregateConstants()
	{
		_left = _left->aggregateConstants();
		_right = _right->aggregateConstants();
		if (dynamic_pointer_cast<Constant>(_left) != 0 && dynamic_pointer_cast<Constant>(_right) != 0)
		{
			shared_ptr<Constant> left = dynamic_pointer_cast<Constant>(_left);
			shared_ptr<Constant> right = dynamic_pointer_cast<Constant>(_right);
			return make_shared<Constant>(min(left->getValue(), right->getValue()));
		}
		else
		{
			return shared_from_this();
		}
	}

	shared_ptr<Term> Min::derivative(shared_ptr<Variable> v)
	{
		throw "Symbolic Derivation of Min not supported.";
	}

	shared_ptr<Term> Min::negate()
	{
		return _left->negate() | _right->negate();
	}

	const shared_ptr<Term> Min::getLeft()
	{
		return _left;
	}

	const shared_ptr<Term> Min::getRight()
	{
		return _right;
	}
} /* namespace AutoDiff */
