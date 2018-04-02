/*
 * ConstraintBuilder.cpp
 *
 *  Created on: Sep 5, 2014
 *      Author: psp
 */

#include "ConstraintBuilder.h"

#include "ConstraintUtility.h"
#include "Term.h"
#include "TermBuilder.h"
#include "TVec.h"

#include <limits>

namespace alica {
const shared_ptr<Term> ConstraintBuilder::TRUE = TermBuilder::constant(1);
const shared_ptr<Term> ConstraintBuilder::FALSE = TermBuilder::constant(numeric_limits<double>::min());

static void setSteepness(double s) {
    Term::setConstraintSteepness(s);
}

/**
 * Returns the euclidean distance between two n-dimenasional vectors.
 *
 * @param t1 A TVec
 * @param t2 A TVec
 *
 * @return A Term
 */
shared_ptr<Term> ConstraintBuilder::distance(shared_ptr<TVec> t1, shared_ptr<TVec> t2) {
    return TermBuilder::euclidianDistance(t1, t2);
}

/**
 * Returns the square of the euclidean distance between two n-dimensional vectors.
 *
 * @param t1 A TVec
 * @param t2 A TVec
 *
 * @return A Term
 */
shared_ptr<Term> ConstraintBuilder::distanceSqr(shared_ptr<TVec> t1, shared_ptr<TVec> t2) {
    return TermBuilder::euclidianDistanceSqr(t1, t2);
}

/**
 * Rotates vec by alpha
 *
 * @param vec A TVec
 * @param alpha A double
 *
 * @return A TVec
 */
shared_ptr<TVec> ConstraintBuilder::rotate(shared_ptr<TVec> vec, double alpha) {
    return make_shared<TVec>(
            initializer_list<shared_ptr<Term>>{TermBuilder::cos(TermBuilder::constant(alpha)) * vec->getX() -
                                                       TermBuilder::sin(TermBuilder::constant(alpha)) * vec->getY(),
                    TermBuilder::sin(TermBuilder::constant(alpha)) * vec->getX() +
                            TermBuilder::cos(TermBuilder::constant(alpha)) * vec->getY()});
}

shared_ptr<Term> ConstraintBuilder::projectVectorOntoX(
        shared_ptr<TVec> origin, shared_ptr<TVec> dir, shared_ptr<Term> x) {
    return origin->getY() + dir->getY() * (x - origin->getX()) * TermBuilder::power(dir->getX(), -1);
}

/**
 * Returns point in the coordinate system defined by vec and its rectangular.
 *
 * @param point A two-dimensional TVec
 * @param vec A two-dimensional TVec
 *
 * @return A two-dimensional TVec
 */
shared_ptr<TVec> ConstraintBuilder::inCoordsOf(shared_ptr<TVec> point, shared_ptr<TVec> vec) {
    shared_ptr<Term> quo = TermBuilder::power(vec->normSquared(), -1);
    return make_shared<TVec>(
            initializer_list<shared_ptr<Term>>{(point->getX() * vec->getX() + point->getY() * vec->getY()) * quo,
                    (point->getX() * vec->getY() - point->getY() * vec->getX()) * quo});
}

/**
 * Two dimensional geometry:
 * Returns if toCheck is left of vec.
 *
 * @param vec A TVec
 * @param toCheck A TVec
 *
 * @result A Term
 */
shared_ptr<Term> ConstraintBuilder::leftOf(shared_ptr<TVec> vec, shared_ptr<TVec> toCheck) {
    return ((toCheck->getX() * vec->getY()) - (toCheck->getY() * vec->getX())) < TermBuilder::constant(0);
}

/**
 * Two dimensional geometry:
 * Returns if toCheck is right of vec.
 *
 * @param vec A TVec
 * @param toCheck A TVec
 *
 * @return A Term
 */
shared_ptr<Term> ConstraintBuilder::rightOf(shared_ptr<TVec> vec, shared_ptr<TVec> toCheck) {
    return ((toCheck->getX() * vec->getY()) - (toCheck->getY() * vec->getX())) > TermBuilder::constant(0);
}

shared_ptr<Term> ConstraintBuilder::equals(shared_ptr<Term> t1, shared_ptr<Term> t2, shared_ptr<Term> tolerance) {
    return (t1 < (t2 + tolerance)) & (t1 > (t2 - tolerance));
}

/**
 * Returns wether the distance between two n-dimensional vectors is less than tolerance.
 *
 * @param t1 A TVec
 * @param t2 A TVec
 * @param tolerance A double
 *
 * @return A Term
 */
shared_ptr<Term> ConstraintBuilder::equals(shared_ptr<TVec> t1, shared_ptr<TVec> t2, double tolerance) {
    return distance(t1, t2) < TermBuilder::constant(tolerance);
}
shared_ptr<Term> ConstraintBuilder::not_(shared_ptr<Term> t) {
    return !t;
}

shared_ptr<Term> ConstraintBuilder::and_(shared_ptr<Term> t1, shared_ptr<Term> t2) {
    return t1 & t2;
}

shared_ptr<Term> ConstraintBuilder::or_(shared_ptr<Term> t1, shared_ptr<Term> t2) {
    return t1 | t2;
}

shared_ptr<Term> ConstraintBuilder::ifThen(shared_ptr<Term> tif, shared_ptr<Term> tthen) {
    return tif->negate() | tthen;
}

shared_ptr<Term> ConstraintBuilder::ifThenElse(shared_ptr<Term> tif, shared_ptr<Term> tthen, shared_ptr<Term> telse) {
    return (tif->negate() | tthen) & (tif | telse);
}

shared_ptr<Term> ConstraintBuilder::equiv(shared_ptr<Term> a, shared_ptr<Term> b) {
    return (a & b) | (a->negate() & b->negate());
}

/**
 * Combines a constraint and a utility. This is usually only used by the CGSolver.
 * Use if you need to circumvent CGSolver and work diectly with GSolver.
 *
 * @param constraint A Term
 * @param utility A Term
 *
 * @result A Term
 */
shared_ptr<Term> ConstraintBuilder::constraintApply(shared_ptr<Term> constraint, shared_ptr<Term> utility) {
    return make_shared<ConstraintUtility>(constraint, utility);
}
} /* namespace alica */
