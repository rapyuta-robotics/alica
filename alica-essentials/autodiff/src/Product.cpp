/*
 * Product.cpp
 *
 *  Created on: Jun 12, 2014
 *      Author: psp
 */

#include "Product.h"

namespace AutoDiff
{

	Product::Product(Term left, Term right)
	{
		_left = left;
		_right = right;
	}

} /* namespace AutoDiff */
