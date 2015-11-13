/*
 * Exp.cpp
 *
 *  Created on: Jun 12, 2014
 *      Author: psp
 */

#include "Exp.h"

#include "TermBuilder.h"
#include "Constant.h"

#include <cmath>

namespace autodiff
{

	Exp::Exp(shared_ptr<Term> arg) :
			Term()
	{
		this->arg = arg;
	}

	int Exp::accept(shared_ptr<ITermVisitor> visitor)
	{
		shared_ptr<Exp> thisCasted = dynamic_pointer_cast<Exp>(shared_from_this());
		return visitor->visit(thisCasted);
	}

	shared_ptr<Term> Exp::aggregateConstants()
	{
		arg = arg->aggregateConstants();
		if (dynamic_pointer_cast<Constant>(arg) != 0)
		{
			shared_ptr<Constant> arg = dynamic_pointer_cast<Constant>(arg);
			return TermBuilder::constant(exp(arg->value));
		}
		else
		{
			if (dynamic_pointer_cast<Constant>(arg) != 0)
			{
				return TermBuilder::constant(1);
			}
			return shared_from_this();
		}
	}

	shared_ptr<Term> Exp::derivative(shared_ptr<Variable> v)
	{
		return shared_from_this() * arg->derivative(v);
	}

	string Exp::toString()
	{
		string str;
		str.append("exp( ");
		str.append(arg->toString());
		str.append(" )");
		return str;
	}

} /* namespace autodiff */
