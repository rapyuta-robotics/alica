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

namespace autodiff
{
	Atan2::Atan2(shared_ptr<Term> left, shared_ptr<Term> right) :
					Term()
	{
		this->left = left;
		this->right = right;
	}

	int Atan2::accept(shared_ptr<ITermVisitor> visitor)
	{
		shared_ptr<Atan2> thisCasted = dynamic_pointer_cast<Atan2>(shared_from_this());
		return visitor->visit(thisCasted);
	}

	shared_ptr<Term> Atan2::aggregateConstants()
	{
		left = left->aggregateConstants();
		right = right->aggregateConstants();
		if (dynamic_pointer_cast<Constant>(left) != 0 && dynamic_pointer_cast<Constant>(right) != 0) {
			shared_ptr<Constant> leftConstant = dynamic_pointer_cast<Constant>(left);
			shared_ptr<Constant> rightConstant = dynamic_pointer_cast<Constant>(right);
			return TermBuilder::constant(atan2(leftConstant->value, rightConstant->value));
		} else {
			return shared_from_this();
		}
	}

	shared_ptr<Term> Atan2::derivative(shared_ptr<Variable> v)
	{
		shared_ptr<Term> t = left * right->derivative(v) - right * left->derivative(v);
		return t / (make_shared<ConstPower>(left, 2) + make_shared<ConstPower>(right, 2));
	}

	string Atan2::toString()
	{
		string str;
		str.append("atan2( ");
		str.append(left->toString());
		str.append(", ");
		str.append(right->toString());
		str.append(" )");
		return str;
	}
} /* namespace autodiff */
