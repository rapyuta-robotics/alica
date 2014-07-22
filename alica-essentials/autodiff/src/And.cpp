/*
 * And.cpp
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#include "And.h"

#include "TermBuilder.h"
#include "Constant.h"

namespace AutoDiff
{
	And::And(shared_ptr<Term> left, shared_ptr<Term> right) :
			Term()
	{
		_left = left;
		_right = right;
	}

	int And::accept(shared_ptr<ITermVisitor> visitor)
	{
		shared_ptr<And> thisCasted = dynamic_pointer_cast<And>(shared_from_this());
		return visitor->visit(thisCasted);
	}

	shared_ptr<Term> And::aggregateConstants()
	{
		_left = _left->aggregateConstants();
		if (_left == Term::FALSE) {
			return _left;
		}
		_right = _right->aggregateConstants();
		if (_left == Term::TRUE) {
			return _right;
		}
		if (_right == Term::FALSE) {
			return _right;
		}
		if (_right == Term::TRUE) {
			return _left;
		}
		if (dynamic_pointer_cast<Constant>(_left) != 0 && dynamic_pointer_cast<Constant>(_right) != 0) {
			shared_ptr<Constant> left = dynamic_pointer_cast<Constant>(_left);
			shared_ptr<Constant> right = dynamic_pointer_cast<Constant>(_right);
			if (left->getValue() > 0.75 && right->getValue() > 0.75) {
				return Term::TRUE;
			} else {
				return Term::FALSE;
			}
		} else {
			return shared_from_this();
		}
	}

	shared_ptr<Term> And::derivative(shared_ptr<Variable> v)
	{
		throw "Symbolic Derivation of And not supported";
	}

	shared_ptr<Term> And::negate()
	{
		return _left->negate() | _right->negate();
	}

	const shared_ptr<Term> And::getLeft()
	{
		return _left;
	}

	const shared_ptr<Term> And::getRight()
	{
		return _right;
	}
} /* namespace AutoDiff */
