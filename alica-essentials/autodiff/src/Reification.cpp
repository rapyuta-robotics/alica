/*
 * Reification.cpp
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#include "Reification.h"

namespace autodiff
{
	Reification::Reification(shared_ptr<Term> condition, double min, double max) :
			Term()
	{
		this->condition = condition;
		this->negatedCondition = condition->negate();
		this->min = min;
		this->max = max;
	}

	int Reification::accept(shared_ptr<ITermVisitor> visitor)
	{
		shared_ptr<Reification> thisCasted = dynamic_pointer_cast<Reification>(shared_from_this());
		return visitor->visit(thisCasted);
	}

	shared_ptr<Term> Reification::aggregateConstants()
	{
		return shared_from_this();
	}

	shared_ptr<Term> Reification::derivative(shared_ptr<Variable> v)
	{
		throw "Symbolic Derivation of Discretizer not supported.";
	}
} /* namespace autodiff */
