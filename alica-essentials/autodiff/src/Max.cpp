/*
 * Max.cpp
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#include "Max.h"

#include "TermBuilder.h"
#include "Constant.h"

#include <cmath>

namespace AutoDiff
{
	Max::Max(shared_ptr<Term> left, shared_ptr<Term> right) :
			Term()
	{
		_left = left;
		_right = right;
	}

	int Max::accept(shared_ptr<ITermVisitor> visitor)
	{
		shared_ptr<Max> thisCasted = dynamic_pointer_cast<Max>(shared_from_this());
		return visitor->visit(thisCasted);
	}

	shared_ptr<Term> Max::aggregateConstants()
	{
		_left = _left->aggregateConstants();
		if (_left == Term::TRUE)
		{
			return _left;
		}
		_right = _right->aggregateConstants();
		if (_left == Term::FALSE)
		{
			return _right;
		}
		if (_right == Term::TRUE)
		{
			return _right;
		}
		if (_right == Term::FALSE)
		{
			return _left;
		}
		if (dynamic_pointer_cast<Constant>(_left) != 0 && dynamic_pointer_cast<Constant>(_right) != 0)
		{
			shared_ptr<Constant> left = dynamic_pointer_cast<Constant>(_left);
			shared_ptr<Constant> right = dynamic_pointer_cast<Constant>(_right);
			return make_shared<Constant>(max(left->getValue(), right->getValue()));
		}
		else
		{
			return shared_from_this();
		}
	}

	shared_ptr<Term> Max::derivative(shared_ptr<Variable> v)
	{
		throw "Symbolic Derivation of Max not supported.";
	}

	shared_ptr<Term> Max::negate()
	{
		return _left->negate() & _right->negate();
	}

	const shared_ptr<Term> Max::getLeft()
	{
		return _left;
	}

	const shared_ptr<Term> Max::getRight()
	{
		return _right;
	}
} /* namespace AutoDiff */
