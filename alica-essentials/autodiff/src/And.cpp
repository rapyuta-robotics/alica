/*
 * And.cpp
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#include "And.h"

#include "TermBuilder.h"
#include "Constant.h"

namespace autodiff
{
	And::And(shared_ptr<Term> left, shared_ptr<Term> right) :
			Term()
	{
		this->left = left;
		this->right = right;
	}

	int And::accept(shared_ptr<ITermVisitor> visitor)
	{
		shared_ptr<And> thisCasted = dynamic_pointer_cast<And>(shared_from_this());
		return visitor->visit(thisCasted);
	}

	shared_ptr<Term> And::aggregateConstants()
	{
		left = left->aggregateConstants();
		if (left == Term::FALSE) {
			return left;
		}
		right = right->aggregateConstants();
		if (left == Term::TRUE) {
			return right;
		}
		if (right == Term::FALSE) {
			return right;
		}
		if (right == Term::TRUE) {
			return left;
		}
		if (dynamic_pointer_cast<Constant>(left) != 0 && dynamic_pointer_cast<Constant>(right) != 0) {
			shared_ptr<Constant> left = dynamic_pointer_cast<Constant>(left);
			shared_ptr<Constant> right = dynamic_pointer_cast<Constant>(right);
			if (left->value > 0.75 && right->value > 0.75) {
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
		return left->negate() | right->negate();
	}
} /* namespace autodiff */
