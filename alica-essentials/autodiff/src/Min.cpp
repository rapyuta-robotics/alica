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

namespace autodiff
{
	Min::Min(shared_ptr<Term> left, shared_ptr<Term> right) :
			Term()
	{
		this->left = left;
		this->right = right;
	}

	int Min::accept(shared_ptr<ITermVisitor> visitor)
	{
		shared_ptr<Min> thisCasted = dynamic_pointer_cast<Min>(shared_from_this());
		return visitor->visit(thisCasted);
	}

	shared_ptr<Term> Min::aggregateConstants()
	{
		left = left->aggregateConstants();
		right = right->aggregateConstants();
		if (dynamic_pointer_cast<Constant>(left) != 0 && dynamic_pointer_cast<Constant>(right) != 0)
		{
			shared_ptr<Constant> left = dynamic_pointer_cast<Constant>(left);
			shared_ptr<Constant> right = dynamic_pointer_cast<Constant>(right);
			return make_shared<Constant>(std::min(left->value, right->value));
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
		return left->negate() | right->negate();
	}
} /* namespace autodiff */
