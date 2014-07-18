/*
 * TermBuilder.cpp
 *
 *  Created on: Jun 6, 2014
 *      Author: psp
 */

#include "TermBuilder.h"

#include "Constant.h"
#include "ConstPower.h"
#include "Exp.h"
#include "Product.h"
#include "Sum.h"
#include "Zero.h"

namespace AutoDiff
{
	/**
	 * Builds a new constant term.
	 *
	 * @param value The constant value
	 *
	 * @return The constant term.
	 */
	shared_ptr<Term> TermBuilder::constant(double value) {
		if (value == 0) {
			return make_shared<Zero>();
		} else {
			return make_shared<Constant>(value);
		}
	}

	/**
	 * Builds a sum of given terms.
	 *
	 * @param v1 The first term in the sum
	 * @param v2 The second term in the sum
	 * @param rest The rest of the terms in the sum.
	 *
	 * @return A term representing the sum of v1, v2 and the terms in rest.
	 */
	shared_ptr<Sum> TermBuilder::sum(shared_ptr<Term> v1, shared_ptr<Term> v2, std::vector<shared_ptr<Term>> rest) {
		std::vector<shared_ptr<Term>> allTerms = { v1, v2 };
		allTerms.insert(allTerms.end(), rest.begin(), rest.end());
		return make_shared<Sum>(allTerms);
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
	shared_ptr<Product> TermBuilder::product(shared_ptr<Term> v1, shared_ptr<Term> v2, std::vector<shared_ptr<Term>> rest) {
		shared_ptr<Product> result = make_shared<Product>(v1, v2);
		for (int i = 0; i < rest.size(); i++) {
			result = make_shared<Product>(result, rest[i]);
		}
		return result;
	}

	/**
	 * Builds a power terms given a base and a constant exponent
	 *
	 * @param t The power base term
	 * @param power The exponent
	 *
	 * @return A term representing t^power.
	 */
	shared_ptr<ConstPower> TermBuilder::power(shared_ptr<Term> t, double power) {
		return make_shared<ConstPower>(t, power);
	}

	/**
	 * Builds a term representing the exponential function e^x.
	 *
	 * @param arg The function's exponent
	 *
	 * @return A term representing e^arg.
	 */
	shared_ptr<Exp> TermBuilder::exp(shared_ptr<Term> arg) {
		return make_shared<Exp>(arg);
	}
} /* namespace AutoDiff */
