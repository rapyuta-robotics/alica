/*
 * Reification.cpp
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#include "Reification.h"

namespace AutoDiff
{
	Reification::Reification(shared_ptr<Term> condition, double min, double max) :
			Term()
	{
		_condition = condition;
		_negatedCondition = condition->negate();
		_min = min;
		_max = max;
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

	const shared_ptr<Term> Reification::getNegatedCondition()
	{
		return _negatedCondition;
	}

	const shared_ptr<Term> Reification::getCondition()
	{
		return _condition;
	}

	const double Reification::getMin()
	{
		return _min;
	}

	const double Reification::getMax()
	{
		return _max;
	}
} /* namespace AutoDiff */
