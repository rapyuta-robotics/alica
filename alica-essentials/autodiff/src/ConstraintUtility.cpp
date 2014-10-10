/*
 * ConstraintUtility.cpp
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#include "ConstraintUtility.h"

#include "TermBuilder.h"
#include "Constant.h"

#include <cmath>

namespace autodiff
{
	ConstraintUtility::ConstraintUtility(shared_ptr<Term> constraint, shared_ptr<Term> utility) :
			Term()
	{
		_constraint = constraint;
		_utility = utility;
	}

	int ConstraintUtility::accept(shared_ptr<ITermVisitor> visitor)
	{
		shared_ptr<ConstraintUtility> thisCasted = dynamic_pointer_cast<ConstraintUtility>(shared_from_this());
		return visitor->visit(thisCasted);
	}

	shared_ptr<Term> ConstraintUtility::aggregateConstants()
	{
		_constraint = _constraint->aggregateConstants();
		_utility = _utility->aggregateConstants();
		return shared_from_this();
	}

	shared_ptr<Term> ConstraintUtility::derivative(shared_ptr<Variable> v)
	{
		throw "Symbolic Derivation of ConstraintUtility not supported.";
	}

	shared_ptr<Term> ConstraintUtility::negate()
	{
		throw "Do not negate a Constraint Utility";
	}

	const shared_ptr<Term> ConstraintUtility::getConstraint()
	{
		return _constraint;
	}

	const shared_ptr<Term> ConstraintUtility::getUtility()
	{
		return _utility;
	}
} /* namespace autodiff */
