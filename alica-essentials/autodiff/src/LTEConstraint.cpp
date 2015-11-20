/*
 * LTEConstraint.cpp
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#include "LTEConstraint.h"

#include "TermBuilder.h"
#include "Constant.h"
#include "LTConstraint.h"

#include <cmath>

namespace autodiff
{
	LTEConstraint::LTEConstraint(shared_ptr<Term> x, shared_ptr<Term> y, double steppness) :
			Term()
	{
		this->left = x;
		this->right = y;
		this->steppness = steppness;
		this->negatedform = nullptr;
	}

	LTEConstraint::LTEConstraint(shared_ptr<Term> x, shared_ptr<Term> y, double steppness, shared_ptr<Term> negatedForm) :
			Term()
	{
		this->left = x;
		this->right = y;
		this->steppness = steppness;
		this->negatedform = negatedForm;
	}

	int LTEConstraint::accept(shared_ptr<ITermVisitor> visitor)
	{
		shared_ptr<LTEConstraint> thisCasted = dynamic_pointer_cast<LTEConstraint>(shared_from_this());
		return visitor->visit(thisCasted);
	}

	shared_ptr<Term> LTEConstraint::aggregateConstants()
	{
		left = left->aggregateConstants();
		right = right->aggregateConstants();
		if (dynamic_pointer_cast<Constant>(left) != 0 && dynamic_pointer_cast<Constant>(right) != 0)
		{
			shared_ptr<Constant> x = dynamic_pointer_cast<Constant>(left);
			shared_ptr<Constant> y = dynamic_pointer_cast<Constant>(right);
			if (x->value <= y->value)
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

	shared_ptr<Term> LTEConstraint::derivative(shared_ptr<Variable> v)
	{
		throw "Symbolic Derivation of Less-Than-Or-Equal not supported.";
	}

	shared_ptr<Term> LTEConstraint::negate()
	{
		if (!(negatedform != nullptr))
		{
			negatedform = make_shared<LTConstraint>(right, left, steppness, shared_from_this());
		}
		return negatedform;
	}

	string LTEConstraint::toString()
	{
		string str;
		str.append(left->toString());
		str.append(" <= ");
		str.append(right->toString());
		return str;
	}
} /* namespace autodiff */
