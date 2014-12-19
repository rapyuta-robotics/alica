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

namespace autodiff
{

	Product::Product(shared_ptr<Term> left, shared_ptr<Term> right) :
			Term()
	{
		this->left = left;
		this->right = right;
	}

	int Product::accept(shared_ptr<ITermVisitor> visitor)
	{
		shared_ptr<Product> thisCasted = dynamic_pointer_cast<Product>(shared_from_this());
		return visitor->visit(thisCasted);
	}

	shared_ptr<Term> Product::aggregateConstants()
	{
		left = left->aggregateConstants();
		right = right->aggregateConstants();
		if (dynamic_pointer_cast<Constant>(left) != 0 && dynamic_pointer_cast<Constant>(right) != 0)
		{
			shared_ptr<Constant> leftConstant = dynamic_pointer_cast<Constant>(left);
			shared_ptr<Constant> rightConstant = dynamic_pointer_cast<Constant>(right);
			return TermBuilder::constant(leftConstant->value * rightConstant->value);
		}
		else if (dynamic_pointer_cast<Zero>(left) != 0)
		{
			return left;
		}
		else if (dynamic_pointer_cast<Zero>(right) != 0)
		{
			return right;
		}
		if (dynamic_pointer_cast<Constant>(left) != 0)
		{
			shared_ptr<Constant> leftConstant = dynamic_pointer_cast<Constant>(left);
			if (leftConstant->value == 1)
			{
				return right;
			}
		}
		if (dynamic_pointer_cast<Constant>(right) != 0)
		{
			shared_ptr<Constant> rightConstant = dynamic_pointer_cast<Constant>(right);
			if (rightConstant->value == 1)
			{
				return left;
			}
		}
		return shared_from_this();
	}

	shared_ptr<Term> Product::derivative(shared_ptr<Variable> v)
	{
		return left * right->derivative(v) + right * left->derivative(v);
	}

} /* namespace autodiff */
