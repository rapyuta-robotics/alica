
#include "Term.h"

#include "TermHolder.h"

#include "Abs.h"
#include "And.h"
#include "Atan2.h"
#include "ConstPower.h"
#include "Constant.h"
#include "ConstraintUtility.h"
#include "Cos.h"
#include "Exp.h"
#include "LTConstraint.h"
#include "LTEConstraint.h"
#include "LinSigmoid.h"
#include "Log.h"
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

#include <limits>
#include <typeinfo>

#include <iostream>

namespace autodiff
{

constexpr double Term::CONSTRAINTSTEEPNESS;
constexpr double Term::EPSILON;

OrType Term::_orop = OrType::MAX;
AndType Term::_andop = AndType::MIN;

Term::Term(TermHolder* owner)
    : _owner(owner)
    , _next(nullptr)
    , _localRange(Variable::minExpressibleValue, Variable::maxExpressibleValue)
    , _tapeIdx(-1)
{
}

Term::~Term() {}

TermPtr Term::negate() const
{
    return _owner->trueConstant() - TermPtr(this);
}

/**
 * Constructs a sum of the two given terms.
 *
 * @param left First term in the sum
 * @param right Second term in the sum
 *
 * @return A term representing the sum of left and right.
 */
TermPtr operator+(const TermPtr left, const TermPtr right)
{
    return left->getOwner()->sum(left, right);
}

/**
 * Constructs a product term of the two given terms.
 *
 * @param left The first term in the product
 * @param right The second term in the product
 *
 * @return A term representing the product of left and right.
 */
TermPtr operator*(const TermPtr left, const TermPtr right)
{
    return left->getOwner()->product(left, right);
}

/**
 * Constructs a fraction term of the two given terms.
 *
 * @param numerator The numerator of the fraction. That is, the "top" part.
 * @param denominator The denominator of the fraction. That is, the "bottom" part.
 *
 * @return A term representing the fraction numerator over denominator.
 */
TermPtr operator/(const TermPtr numerator, const TermPtr denominator)
{
    TermHolder* owner = numerator->getOwner();
    return owner->product(numerator, owner->power(denominator, -1));
}
/**
 * Constructs a difference of the two given terms.
 *
 * @param left The first term in the difference
 * @param right The second term in the difference.
 *
 * @return A term representing left - right.
 */
TermPtr operator-(const TermPtr left, const TermPtr right)
{
    return left + left->getOwner()->constant(-1) * right;
}

/*
 * Support double <operator> Term
 */
TermPtr operator+(const double left, const TermPtr right)
{
    return right->getOwner()->constant(left) + right;
}

TermPtr operator*(const double left, const TermPtr right)
{
    return right->getOwner()->constant(left) * right;
}

TermPtr operator/(const double numerator, const TermPtr denominator)
{
    return denominator->getOwner()->constant(numerator) / denominator;
}

TermPtr operator-(const double left, const TermPtr right)
{
    return right->getOwner()->constant(left) - right;
}

/*
 * Support Term <operator> double
 */
TermPtr operator+(const TermPtr left, const double right)
{
    return left + left->getOwner()->constant(right);
}

TermPtr operator*(const TermPtr left, const double right)
{
    return left * left->getOwner()->constant(right);
}

TermPtr operator/(const TermPtr numerator, const double denominator)
{
    return numerator / numerator->getOwner()->constant(denominator);
}

TermPtr operator-(const TermPtr left, const double right)
{
    return left - left->getOwner()->constant(right);
}

TermPtr operator-(const TermPtr term)
{
    return term->getOwner()->constant(-1) * term;
}

TermPtr operator!(const TermPtr term)
{
    return term->negate();
}

TermPtr operator&(const TermPtr left, const TermPtr right)
{
    if (left == left->getOwner()->trueConstant() || right == right->getOwner()->falseConstant()) {
        return right;
    } else if (left == left->getOwner()->falseConstant() || right == right->getOwner()->trueConstant()) {
        return left;
    }
    if (Term::getAnd() == AndType::AND) {
        return left->getOwner()->and_(left, right);
    } else {
        return left->getOwner()->min(left, right);
    }
}

TermPtr operator|(const TermPtr left, const TermPtr right)
{
    if (left == left->getOwner()->trueConstant() || right == right->getOwner()->falseConstant()) {
        return left;
    } else if (right == right->getOwner()->trueConstant() || left == left->getOwner()->falseConstant()) {
        return right;
    }
    if (Term::getOr() == OrType::OR) {
        return left->getOwner()->or_(left, right);
    } else {
        return left->getOwner()->max(left, right);
    }
}

TermPtr operator>(const TermPtr left, const TermPtr right)
{
    return left->getOwner()->lessThan(right, left);
}

TermPtr operator<(const TermPtr left, const TermPtr right)
{
    return left->getOwner()->lessThan(left, right);
}

TermPtr operator<=(const TermPtr left, const TermPtr right)
{
    return left->getOwner()->lessThanEqual(left, right);
}

TermPtr operator>=(const TermPtr left, const TermPtr right)
{
    return left->getOwner()->lessThanEqual(right, left);
}

TermPtr operator&=(const TermPtr left, const TermPtr right)
{
    return left & right;
}

// some static dirt...
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

} /* namespace autodiff */
