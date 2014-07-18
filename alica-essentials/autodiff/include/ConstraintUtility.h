/*
 * ConstraintUtility.h
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#ifndef CONSTRAINTUTILITY_H_
#define CONSTRAINTUTILITY_H_

#include "Term.h"

namespace AutoDiff
{

	class ConstraintUtility : public Term
	{
	public:
		ConstraintUtility(shared_ptr<Term> constraint, shared_ptr<Term> utility);

		int accept(shared_ptr<ITermVisitor> visitor);

		shared_ptr<Term> aggregateConstants();
		shared_ptr<Term> derivative(shared_ptr<Variable> v);
		shared_ptr<Term> negate();

		const shared_ptr<Term> getConstraint();
		const shared_ptr<Term> getUtility();

	private:
		shared_ptr<Term> _constraint;
		shared_ptr<Term> _utility;
	};

} /* namespace AutoDiff */

#endif /* CONSTRAINTUTILITY_H_ */
