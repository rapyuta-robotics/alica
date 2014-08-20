/*
 * LTEConstraint.h
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#ifndef LTECONSTRAINT_H_
#define LTECONSTRAINT_H_

#include "Term.h"

namespace AutoDiff
{

	class LTEConstraint : public Term
	{
	public:
		LTEConstraint(shared_ptr<Term> x, shared_ptr<Term> y, double steppness);
		LTEConstraint(shared_ptr<Term> x, shared_ptr<Term> y, double steppness, shared_ptr<Term> negatedForm);

		int accept(shared_ptr<ITermVisitor> visitor);

		shared_ptr<Term> aggregateConstants();
		shared_ptr<Term> derivative(shared_ptr<Variable> v);
		shared_ptr<Term> negate();

		const shared_ptr<Term> getLeft();
		const shared_ptr<Term> getRight();
		const double getSteppness();
		const shared_ptr<Term> getNegatedForm();
	private:
		shared_ptr<Term> _left;
		shared_ptr<Term> _right;
		double _steppness;
		shared_ptr<Term> _negatedform;
	};

} /* namespace AutoDiff */

#endif /* LTECONSTRAINT_H_ */
