/*
 * Term.cpp
 *
 *  Created on: Jun 5, 2014
 *      Author: psp
 */

#include "Term.h"

#include "TermBuilder.h"
#include "Abs.h"
#include "And.h"
#include "Atan2.h"
#include "Constant.h"
#include "ConstPower.h"
#include "ConstraintUtility.h"
#include "Cos.h"
#include "Exp.h"
#include "Gp.h"
#include "LinSigmoid.h"
#include "Log.h"
#include "LTConstraint.h"
#include "LTEConstraint.h"
#include "Max.h"
#include "Min.h"
#include "Or.h"
#include "Product.h"
#include "Reification.h"
#include "Sigmoid.h"
#include "Sin.h"
#include "Sum.h"
#include "TermPower.h"
#include "Variable.h"
#include "Zero.h"

#include <typeinfo>
#include <limits>

#include <iostream>

namespace autodiff
{
	int Term::m_nextId = 0;

	double Term::_constraintSteepness = 0.01;
	const shared_ptr<Term> Term::TRUE = TermBuilder::constant(1);
	const shared_ptr<Term> Term::FALSE = TermBuilder::constant(numeric_limits<double>::min());
	const double Term::EPSILON = 10.0e-10;
	OrType Term::_orop = OrType::MAX;
	AndType Term::_andop = AndType::MIN;

	Term::Term() :
			m_id(m_nextId++)
	{
		min = numeric_limits<double>::min();
		max = numeric_limits<double>::max();
	}

	Term::~Term()
	{
	}

	int Term::getId() const
	{
		return m_id;
	}

	shared_ptr<Term> Term::negate()
	{
		return 1 - shared_from_this();
	}

	AndType Term::getAnd()
	{
		return _andop;
	}

	void Term::setAnd(AndType a)
	{
		_andop = a;
	}

	OrType Term::getOr()
	{
		return _orop;
	}

	void Term::setOr(OrType o)
	{
		_orop = o;
	}

	double Term::getConstraintSteepness()
	{
		return _constraintSteepness;
	}

	void Term::setConstraintSteepness(double constraintSteepness)
	{
		_constraintSteepness = constraintSteepness;
	}

	/**
	 * Constructs a sum of the two given terms.
	 *
	 * @param left First term in the sum
	 * @param right Second term in the sum
	 *
	 * @return A term representing the sum of left and right.
	 */
	shared_ptr<Term> operator+(const shared_ptr<Term>& left, const shared_ptr<Term>& right)
	{
		if (dynamic_pointer_cast<Zero>(left) != 0 && dynamic_pointer_cast<Zero>(right) != 0)
		{
			return make_shared<Zero>();
		}
		else if (dynamic_pointer_cast<Zero>(left) != 0)
		{
			return right;
		}
		else if (dynamic_pointer_cast<Zero>(right) != 0)
		{
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
	shared_ptr<Term> operator*(const shared_ptr<Term>& left, const shared_ptr<Term>& right)
	{
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
	shared_ptr<Term> operator/(const shared_ptr<Term>& numerator, const shared_ptr<Term>& denominator)
	{
		return TermBuilder::product(numerator, TermBuilder::power(denominator, -1));
	}
	/**
	 * Constructs a difference of the two given terms.
	 *
	 * @param left The first term in the difference
	 * @param right The second term in the difference.
	 *
	 * @return A term representing left - right.
	 */
	shared_ptr<Term> operator-(const shared_ptr<Term>& left, const shared_ptr<Term>& right)
	{
		return left + -1 * right;
	}

	/*
	 * Support double <operator> Term
	 */
	shared_ptr<Term> operator+(const double left, const shared_ptr<Term>& right)
	{
		return TermBuilder::constant(left) + right;
	}

	shared_ptr<Term> operator*(const double left, const shared_ptr<Term>& right)
	{
		return TermBuilder::constant(left) * right;
	}

	shared_ptr<Term> operator/(const double numerator, const shared_ptr<Term>& denominator)
	{
		return TermBuilder::constant(numerator) / denominator;
	}

	shared_ptr<Term> operator-(const double left, const shared_ptr<Term>& right)
	{
		return TermBuilder::constant(left) - right;
	}

	/*
	 * Support Term <operator> double
	 */
	shared_ptr<Term> operator+(const shared_ptr<Term>& left, const double right)
	{
		return left + TermBuilder::constant(right);
	}

	shared_ptr<Term> operator*(const shared_ptr<Term>& left, const double right)
	{
		return left * TermBuilder::constant(right);
	}

	shared_ptr<Term> operator/(const shared_ptr<Term>& numerator, const double denominator)
	{
		return numerator / TermBuilder::constant(denominator);
	}

	shared_ptr<Term> operator-(const shared_ptr<Term>& left, const double right)
	{
		return left - TermBuilder::constant(right);
	}

	shared_ptr<Term> operator-(const shared_ptr<Term>& term)
	{
		return -1 * term;
	}

	shared_ptr<Term> operator!(const shared_ptr<Term>& term)
	{
		return term->negate();
	}

	shared_ptr<Term> operator&(const shared_ptr<Term>& left, const shared_ptr<Term>& right)
	{

		if (Term::getAnd() == AndType::AND)
		{
			if (left == Term::TRUE || right == Term::FALSE)
			{
				return right;
			}
			else if (left == Term::FALSE || right == Term::TRUE)
			{
				return left;
			}
			return make_shared<And>(left, right);
		}
		else
		{
			if (left == Term::TRUE || right == Term::FALSE)
			{
				return right;
			}
			else if (left == Term::FALSE || right == Term::TRUE)
			{
				return left;
			}
			return make_shared<Min>(left, right);
		}
	}

	shared_ptr<Term> operator&=(const shared_ptr<Term>& left, const shared_ptr<Term>& right) {
		return left & right;
	}
	shared_ptr<Term> operator|=(const shared_ptr<Term>& left, const shared_ptr<Term>& right) {
		return left | right;
	}

	shared_ptr<Term> operator|(const shared_ptr<Term>& left, const shared_ptr<Term>& right)
	{
		if (Term::getOr() == OrType::OR)
		{
			if (left == Term::TRUE || (right == Term::FALSE && left == Term::FALSE))
			{
				return left;
			}
			else if (right == Term::TRUE)
			{
				return right;
			}
			return make_shared<Or>(left, right);
		}
		else
		{
			if (left == Term::TRUE || (right == Term::FALSE && left == Term::FALSE))
			{
				return left;
			}
			else if (right == Term::TRUE)
			{
				return right;
			}
			return make_shared<Max>(left, right);
		}
	}

	shared_ptr<Term> operator%(const shared_ptr<Term>& left, const shared_ptr<Term>& right)
	{
		return left * right;
	}

	shared_ptr<Term> operator^(const shared_ptr<Term>& left, const shared_ptr<Term>& right)
	{
		return !(!left % !right);
	}

	shared_ptr<Term> operator>(const shared_ptr<Term>& left, const shared_ptr<Term>& right)
	{
		return make_shared<LTConstraint>(right, left, Term::getConstraintSteepness());
	}

	shared_ptr<Term> operator<(const shared_ptr<Term>& left, const shared_ptr<Term>& right)
	{
		return make_shared<LTConstraint>(left, right, Term::getConstraintSteepness());
	}

	shared_ptr<Term> operator<=(const shared_ptr<Term>& left, const shared_ptr<Term>& right)
	{
		return make_shared<LTEConstraint>(left, right, Term::getConstraintSteepness());
	}

	shared_ptr<Term> operator>=(const shared_ptr<Term>& left, const shared_ptr<Term>& right)
	{
		return make_shared<LTEConstraint>(right, left, Term::getConstraintSteepness());
	}
} /* namespace autodiff */
