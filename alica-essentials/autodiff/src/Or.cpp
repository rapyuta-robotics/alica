/*
 * Or.cpp
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#include "Or.h"

#include "TermBuilder.h"
#include "Constant.h"

#include <cmath>

namespace autodiff
{
	Or::Or(shared_ptr<Term> left, shared_ptr<Term> right) :
			Term()
	{
		this->left = left;
		this->right = right;
	}

	int Or::accept(shared_ptr<ITermVisitor> visitor)
	{
		shared_ptr<Or> thisCasted = dynamic_pointer_cast<Or>(shared_from_this());
		return visitor->visit(thisCasted);
	}

	shared_ptr<Term> Or::aggregateConstants()
	{
		left = left->aggregateConstants();
		right = right->aggregateConstants();
		if (dynamic_pointer_cast<Constant>(left) != 0 && dynamic_pointer_cast<Constant>(right) != 0)
		{
			shared_ptr<Constant> left = dynamic_pointer_cast<Constant>(left);
			shared_ptr<Constant> right = dynamic_pointer_cast<Constant>(right);
			if (left->value > 0.75 || right->value > 0.75)
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

	shared_ptr<Term> Or::derivative(shared_ptr<Variable> v)
	{
		throw "Symbolic Derivation of Or not supported.";
	}

	shared_ptr<Term> Or::negate()
	{
		return left->negate() & right->negate();
	}
} /* namespace autodiff */
