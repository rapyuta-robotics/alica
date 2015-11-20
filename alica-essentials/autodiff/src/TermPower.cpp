/*
 * TermPower.cpp
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#include "TermPower.h"

#include "TermBuilder.h"
#include "Constant.h"
#include "Log.h"
#include "Zero.h"

#include <cmath>

namespace autodiff
{
	TermPower::TermPower(shared_ptr<Term> baseTerm, shared_ptr<Term> exponent) :
			Term()
	{
		this->base = baseTerm;
		this->exponent = exponent;
	}

	int TermPower::accept(shared_ptr<ITermVisitor> visitor)
	{
		shared_ptr<TermPower> thisCasted = dynamic_pointer_cast<TermPower>(shared_from_this());
		return visitor->visit(thisCasted);
	}

	shared_ptr<Term> TermPower::aggregateConstants()
	{
		base = base->aggregateConstants();
		exponent = exponent->aggregateConstants();
		if (dynamic_pointer_cast<Zero>(exponent) != 0)
		{
			return TermBuilder::constant(1);
		}
		if (dynamic_pointer_cast<Constant>(base) != 0 && dynamic_pointer_cast<Constant>(exponent) != 0)
		{
			shared_ptr<Constant> base = dynamic_pointer_cast<Constant>(base);
			shared_ptr<Constant> exponent = dynamic_pointer_cast<Constant>(exponent);
			return TermBuilder::constant(pow(base->value, exponent->value));
		} else if (dynamic_pointer_cast<Zero>(base) != 0) {
			return base;
		} else if (dynamic_pointer_cast<TermPower>(base) != 0) {
			shared_ptr<TermPower> base = dynamic_pointer_cast<TermPower>(base);
			this->exponent = exponent * base->exponent;
			this->base = base->base;
			return shared_from_this();
		} else {
			return shared_from_this();
		}
	}

	shared_ptr<Term> TermPower::derivative(shared_ptr<Variable> v)
	{
		return make_shared<TermPower>(base, exponent - 1)
				* (exponent * base->derivative(v) + base * make_shared<Log>(base) * exponent->derivative(v));
	}

	string TermPower::toString()
	{
		string str;
		str.append("termPower( ");
		str.append(base->toString());
		str.append(", ");
		str.append(exponent->toString());
		str.append(" )");
		return str;
	}
} /* namespace autodiff */
