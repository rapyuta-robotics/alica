/*
 * Max.h
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#ifndef MAX_H_
#define MAX_H_

#include "Term.h"

namespace autodiff
{

	class Max : public Term
	{
	public:
		Max(shared_ptr<Term> left, shared_ptr<Term> right);

		shared_ptr<Term> left;
		shared_ptr<Term> right;

		int accept(shared_ptr<ITermVisitor> visitor);

		shared_ptr<Term> aggregateConstants();
		shared_ptr<Term> derivative(shared_ptr<Variable> v);
		shared_ptr<Term> negate();

		string toString();
	};

} /* namespace autodiff */

#endif /* MAX_H_ */
