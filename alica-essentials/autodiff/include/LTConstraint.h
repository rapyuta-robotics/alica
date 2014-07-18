/*
 * LTConstraint.h
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#ifndef LTCONSTRAINT_H_
#define LTCONSTRAINT_H_

#include "Term.h"

namespace AutoDiff
{
	class LTEConstraint;

	class LTConstraint : public Term
	{
	public:
		LTConstraint(shared_ptr<Term> x, shared_ptr<Term> y, double steppness);
		LTConstraint(shared_ptr<Term> x, shared_ptr<Term> y, double steppness, shared_ptr<Term> negatedForm);

		int accept(shared_ptr<ITermVisitor> visitor);

		shared_ptr<Term> aggregateConstants();
		shared_ptr<Term> derivative(shared_ptr<Variable> v);
		shared_ptr<Term> negate();

		const shared_ptr<Term> getX();
		const shared_ptr<Term> getY();
		const double getSteppness();
		const shared_ptr<Term> getNegatedForm();
	private:
		shared_ptr<Term> _x;
		shared_ptr<Term> _y;
		double _steppness;
		shared_ptr<Term> _negatedform;
	};

} /* namespace AutoDiff */

#endif /* LTCONSTRAINT_H_ */
