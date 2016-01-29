/*
 * ConstPower.cpp
 *
 *  Created on: Jun 16, 2014
 *      Author: psp
 */

#include "ConstPower.h"

#include "TermBuilder.h"
#include "Constant.h"
#include "Zero.h"

#include <cmath>
#include <string>

namespace autodiff
{
	ConstPower::ConstPower(shared_ptr<Term> baseTerm, double exponent) :
			Term()
	{
		this->base = baseTerm;
		this->exponent = exponent;
	}

	int ConstPower::accept(shared_ptr<ITermVisitor> visitor)
	{
		shared_ptr<ConstPower> thisCasted = dynamic_pointer_cast<ConstPower>(shared_from_this());
		return visitor->visit(thisCasted);
	}

	shared_ptr<Term> ConstPower::aggregateConstants()
	{
		base = base->aggregateConstants(); //all your base are belong to us
		auto constBase = dynamic_pointer_cast<Constant>(this->base);

		if (constBase)
		{
			return TermBuilder::constant(pow(constBase->value, exponent));
		}

		auto zeroBase = dynamic_pointer_cast<Zero>(base);

		if (zeroBase)
		{
			if (exponent >= 0)
			{
				return zeroBase;
			}
			else
			{
				throw "ConstPower: Divide By Zero";
			}
		}

		auto constPowerBase = dynamic_pointer_cast<ConstPower>(base);

		if (constPowerBase)
		{
			this->exponent *= constPowerBase->exponent;
			this->base = constPowerBase->base;
			return shared_from_this();
		}

		return shared_from_this();
	}

	shared_ptr<Term> ConstPower::derivative(shared_ptr<Variable> v)
	{
		return TermBuilder::constant(exponent) * make_shared<ConstPower>(base, exponent - 1) * base->derivative(v);
	}

	string ConstPower::toString()
	{
		string str;
		str.append("constPower( ");
		if (base != nullptr)
			str.append(base->toString());
		else
			str.append("nullptr");
		str.append(", ");
		str.append(std::to_string(exponent));
		str.append(" )");
		return str;
	}
} /* namespace autodiff */
