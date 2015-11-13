/*
 * Product.h
 *
 *  Created on: Jun 12, 2014
 *      Author: psp
 */

#ifndef PRODUCT_H_
#define PRODUCT_H_

#include "Term.h"

namespace autodiff
{

	class Product : public Term
	{
	public:
		Product(shared_ptr<Term> left, shared_ptr<Term> right);

		shared_ptr<Term> left;
		shared_ptr<Term> right;

		int accept(shared_ptr<ITermVisitor> visitor);

		shared_ptr<Term> aggregateConstants();
		shared_ptr<Term> derivative(shared_ptr<Variable> v);

		string toString();
	};

} /* namespace autodiff */

#endif /* PRODUCT_H_ */
