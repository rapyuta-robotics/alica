/*
 * And.h
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#ifndef AND_H_
#define AND_H_

#include "Term.h"

namespace autodiff
{

	class And : public Term
	{
	public:
		And(shared_ptr<Term> left, shared_ptr<Term> right);

		int accept(shared_ptr<ITermVisitor> visitor);

		shared_ptr<Term> aggregateConstants();
		shared_ptr<Term> derivative(shared_ptr<Variable> v);
		shared_ptr<Term> negate();

		const shared_ptr<Term> getLeft();
		const shared_ptr<Term> getRight();

	private:
		shared_ptr<Term> _left;
		shared_ptr<Term> _right;
	};

} /* namespace autodiff */

#endif /* AND_H_ */
