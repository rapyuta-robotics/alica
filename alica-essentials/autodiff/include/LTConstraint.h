/*
 * LTConstraint.h
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#ifndef LTCONSTRAINT_H_
#define LTCONSTRAINT_H_

#include "Term.h"

namespace autodiff
{
	class LTEConstraint;

	class LTConstraint : public Term
	{
	public:
		LTConstraint(shared_ptr<Term> x, shared_ptr<Term> y, double steppness);
		LTConstraint(shared_ptr<Term> x, shared_ptr<Term> y, double steppness, shared_ptr<Term> negatedForm);

		shared_ptr<Term> left;
		shared_ptr<Term> right;
		double steppness;
		shared_ptr<Term> negatedform;

		int accept(shared_ptr<ITermVisitor> visitor);

		shared_ptr<Term> aggregateConstants();
		shared_ptr<Term> derivative(shared_ptr<Variable> v);
		shared_ptr<Term> negate();

		string toString();
	};

} /* namespace autodiff */

#endif /* LTCONSTRAINT_H_ */
