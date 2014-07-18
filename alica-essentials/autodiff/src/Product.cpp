/*
 * Product.cpp
 *
 *  Created on: Jun 12, 2014
 *      Author: psp
 */

#include "Product.h"

#include "TermBuilder.h"
#include "Constant.h"
#include "Zero.h"

#include <cmath>

namespace AutoDiff
{

	Product::Product(shared_ptr<Term> left, shared_ptr<Term> right) :
			Term()
	{
		_left = left;
		_right = right;
	}

	int Product::accept(shared_ptr<ITermVisitor> visitor)
	{
		shared_ptr<Product> thisCasted = dynamic_pointer_cast<Product>(shared_from_this());
		return visitor->visit(thisCasted);
	}

	shared_ptr<Term> Product::aggregateConstants()
	{
		_left = _left->aggregateConstants();
		_right = _right->aggregateConstants();
		if (dynamic_pointer_cast<Constant>(_left) != 0 && dynamic_pointer_cast<Constant>(_right) != 0)
		{
			shared_ptr<Constant> left = dynamic_pointer_cast<Constant>(_left);
			shared_ptr<Constant> right = dynamic_pointer_cast<Constant>(_right);
			return TermBuilder::constant(left->getValue() * right->getValue());
		}
		else if (dynamic_pointer_cast<Zero>(_left) != 0)
		{
			return _left;
		}
		else if (dynamic_pointer_cast<Zero>(_right) != 0)
		{
			return _right;
		}
		if (dynamic_pointer_cast<Constant>(_left) != 0)
		{
			shared_ptr<Constant> left = dynamic_pointer_cast<Constant>(_left);
			if (left->getValue() == 1)
			{
				return _right;
			}
		}
		if (dynamic_pointer_cast<Constant>(_right) != 0)
		{
			shared_ptr<Constant> right = dynamic_pointer_cast<Constant>(_right);
			if (right->getValue() == 1)
			{
				return _left;
			}
		}
		return shared_from_this();
	}

	shared_ptr<Term> Product::derivative(shared_ptr<Variable> v)
	{
		return _left * _right->derivative(v) + _right * _left->derivative(v);
	}

	shared_ptr<Term> Product::getLeft()
	{
		return _left;
	}
	shared_ptr<Term> Product::getRight()
	{
		return _right;
	}

} /* namespace AutoDiff */
