/*
 * TermBuilder.cpp
 *
 *  Created on: Jun 6, 2014
 *      Author: psp
 */

#include "TermBuilder.h"

#include <iostream>

namespace AutoDiff
{
	/**
	 * Builds a sum of given terms.
	 *
	 * @param v1 The first term in the sum
	 * @param v2 The second term in the sum
	 * @param rest The rest of the terms in the sum.
	 *
	 * @return A term representing the sum of v1, v2 and the terms in rest.
	 */
	Sum TermBuilder::sum(Term v1, Term v2, std::vector<Term> rest) {
		std::vector<Term> allTerms = { v1, v2 };
		allTerms.insert(allTerms.end(), rest.begin(), rest.end());
		return Sum(allTerms);
	}

	/**
	 * Builds a product of given terms.
	 *
	 * @param v1 The first term in the product
	 * @param v2 The second term in the product
	 * @param rest The rest of the terms in the product
	 *
	 * @return A term representing the product of v1, v2 and the terms in rest.
	 */
	Product TermBuilder::product(Term v1, Term v2, std::vector<Term> rest) {
		Product result = Product(v1, v2);
		for (int i = 0; i < rest.size(); i++) {
			result = Product(result, rest[i]);
		}
		return result;
	}

	/**
	 * Builds a term representing the exponential function e^x.
	 *
	 * @param arg The function's exponent
	 *
	 * @return A term representing e^arg.
	 */
	Exp TermBuilder::exp(Term arg) {
		return Exp(arg);
	}
} /* namespace AutoDiff */
