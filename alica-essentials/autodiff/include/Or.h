/*
 * Or.h
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#ifndef OR_H_
#define OR_H_

#include "Term.h"

namespace autodiff
{

	class Or : public Term
	{
	public:
		Or(shared_ptr<Term> left, shared_ptr<Term> right);

		shared_ptr<Term> left;
		shared_ptr<Term> right;

		int accept(shared_ptr<ITermVisitor> visitor);

		shared_ptr<Term> aggregateConstants();
		shared_ptr<Term> derivative(shared_ptr<Variable> v);
		shared_ptr<Term> negate();
	};

} /* namespace autodiff */

#endif /* OR_H_ */
