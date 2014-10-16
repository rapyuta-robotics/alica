/*
 * ConstraintBuilder.cpp
 *
 *  Created on: Sep 5, 2014
 *      Author: psp
 */

#include "engine/constraintmodul/ConstraintBuilder.h"

#include <limits>

namespace alica
{
	const shared_ptr<Term> ConstraintBuilder::TRUE = TermBuilder::constant(1);
	const shared_ptr<Term> ConstraintBuilder::FALSE = TermBuilder::constant(numeric_limits<double>::min());

	/**
	 * Returns the euclidean distance between two n-dimenasional vectors.
	 *
	 * @param t1 A TVec
	 * @param t2 A TVec
	 *
	 * @return A Term
	 */
	shared_ptr<Term> ConstraintBuilder::distance(shared_ptr<TVec> t1, shared_ptr<TVec> t2)
	{
		return TermBuilder::euclidianDistance(t1, t2);
	}
} /* namespace alica */
