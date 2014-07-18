/*
 * Atan2.cpp
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#include "Atan2.h"

#include "TermBuilder.h"
#include "Constant.h"
#include "ConstPower.h"

#include <cmath>

namespace AutoDiff
{
	Atan2::Atan2(shared_ptr<Term> left, shared_ptr<Term> right) :
					Term()
	{
		_left = left;
		_right = right;
	}

	int Atan2::accept(shared_ptr<ITermVisitor> visitor)
	{
		shared_ptr<Atan2> thisCasted = dynamic_pointer_cast<Atan2>(shared_from_this());
		return visitor->visit(thisCasted);
	}

	shared_ptr<Term> Atan2::aggregateConstants()
	{
		_left = _left->aggregateConstants();
		_right = _right->aggregateConstants();
		if (dynamic_pointer_cast<Constant>(_left) != 0 && dynamic_pointer_cast<Constant>(_right) != 0) {
			shared_ptr<Constant> left = dynamic_pointer_cast<Constant>(_left);
			shared_ptr<Constant> right = dynamic_pointer_cast<Constant>(_right);
			return TermBuilder::constant(atan2(left->getValue(), right->getValue()));
		} else {
			return shared_from_this();
		}
	}

	shared_ptr<Term> Atan2::derivative(shared_ptr<Variable> v)
	{
		shared_ptr<Term> t = _left * _right->derivative(v) - _right * _left->derivative(v);
		return t / (make_shared<ConstPower>(_left, 2) + make_shared<ConstPower>(_right, 2));
	}

	const shared_ptr<Term> Atan2::getLeft()
	{
		return _left;
	}

	const shared_ptr<Term> Atan2::getRight()
	{
		return _right;
	}
} /* namespace AutoDiff */
