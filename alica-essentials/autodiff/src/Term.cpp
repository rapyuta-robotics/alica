/*
 * Term.cpp
 *
 *  Created on: Jun 5, 2014
 *      Author: psp
 */

#include "Term.h"

#include "TermBuilder.h"
#include "Zero.h"

#include <typeinfo>

namespace AutoDiff
{
	Term::Term()
	{
	}

	Term::~Term()
	{
	}

	/**
	 * Constructs a sum of the two given terms.
	 *
	 * @param left First term in the sum
	 * @param right Second term in the sum
	 *
	 * @return A term representing the sum of left and right.
	 */
	Term operator+(const Term& left, const Term& right) {
		Zero zero;
		if (typeid(left) == typeid(zero) &&
				typeid(right) == typeid(zero)) {
			return zero;
		} else if (typeid(left) == typeid(zero)) {
			return right;
		} else if (typeid(right) == typeid(zero)) {
			return left;
		}
		return TermBuilder::sum(left, right);
	}

	/**
	 * Constructs a product term of the two given terms.
	 *
	 * @param left The first term in the product
	 * @param right The second term in the product
	 *
	 * @return A term representing the product of left and right.
	 */
	Term operator*(const Term& left, const Term& right) {
		return TermBuilder::product(left, right);
	}

	/**
	 * Constructs a fraction term of the two given terms.
	 *
	 * @param numerator The numerator of the fraction. That is, the "top" part.
	 * @param denominator The denominator of the fraction. That is, the "bottom" part.
	 *
	 * @return A term representing the fraction numerator over denominator.
	 */
//	Term operator/(const Term& numerator, const Term& denominator) {
//
//	}

	/**
	 * Constructs a difference of the two given terms.
	 *
	 * @param left The first term in the difference
	 * @param right The second term in the difference.
	 *
	 * @return A term representing left - right.
	 */
	Term operator-(const Term& left, const Term& right) {
		return Term(); // TODO: fix it
//		return left + (-1) * right;
	}
} /* namespace AutoDiff */
