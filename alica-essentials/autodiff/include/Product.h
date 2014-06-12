/*
 * Product.h
 *
 *  Created on: Jun 12, 2014
 *      Author: psp
 */

#ifndef PRODUCT_H_
#define PRODUCT_H_

#include "Term.h"

namespace AutoDiff
{

	class Product : public Term
	{
	public:
		Product(Term left, Term right);

	private:
		Term _left;
		Term _right;
	};

} /* namespace AutoDiff */

#endif /* PRODUCT_H_ */
