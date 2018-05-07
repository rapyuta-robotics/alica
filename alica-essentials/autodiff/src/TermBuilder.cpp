/*
 * TermBuilder.cpp
 *
 *  Created on: Jun 6, 2014
 *      Author: psp
 */

#include "TermBuilder.h"

#include "Constant.h"
#include "ConstPower.h"
#include "Cos.h"
#include "Exp.h"
#include "Log.h"
#include "LTConstraint.h"
#include "LTEConstraint.h"
#include "Product.h"
#include "Sigmoid.h"
#include "Sin.h"
#include "Sum.h"
#include "Term.h"
#include "TermPower.h"
#include "TVec.h"
#include "Zero.h"

#include <cmath>

#include <iostream>

namespace autodiff {
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
 * @param terms The collection of terms in the sum.
 *
 * @return A term representing the sum of the terms in terms.
 */
shared_ptr<Term> TermBuilder::sum(vector<shared_ptr<Term>> terms) {
    vector<shared_ptr<Term>> allTerms;
    for (int i = 0; i < terms.size(); i++) {
        shared_ptr<Term> term = terms[i];
        if (!(dynamic_pointer_cast<Zero>(term) != 0)) {
            allTerms.push_back(term);
        }
    }
    return make_shared<Sum>(allTerms);
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
shared_ptr<Term> TermBuilder::sum(shared_ptr<Term> v1, shared_ptr<Term> v2, vector<shared_ptr<Term>> rest) {
    vector<shared_ptr<Term>> allTerms = {v1, v2};
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
shared_ptr<Term> TermBuilder::product(shared_ptr<Term> v1, shared_ptr<Term> v2, vector<shared_ptr<Term>> rest) {
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
shared_ptr<Term> TermBuilder::power(shared_ptr<Term> t, double power) {
    return make_shared<ConstPower>(t, power);
}

/**
 * Builds a power term given a base term and an exponent term.
 *
 * @param baseTerm The base term
 * @param exponent The exponent term
 *
 * @return
 */
shared_ptr<Term> TermBuilder::power(shared_ptr<Term> baseTerm, shared_ptr<Term> exponent) {
    return make_shared<TermPower>(baseTerm, exponent);
}

/**
 * Builds a term representing the exponential function e^x.
 *
 * @param arg The function's exponent
 *
 * @return A term representing e^arg.
 */
shared_ptr<Term> TermBuilder::exp(shared_ptr<Term> arg) {
    return make_shared<Exp>(arg);
}

/**
 * Builds a term representing the natural logarithm.
 *
 * @param arg The natural logarithm's argument.
 *
 * @return A term representing the natural logarithm of arg
 */
shared_ptr<Term> TermBuilder::log(shared_ptr<Term> arg) {
    return make_shared<Log>(arg);
}

/**
 * Builds a term representing the sine function.
 *
 * @param arg The sine argument.
 *
 * @return A term representing the sine of arg.
 */
shared_ptr<Term> TermBuilder::sin(shared_ptr<Term> arg) {
    return make_shared<Sin>(arg);
}

/**
 * Builds a term representing the cosine function.
 *
 * @param arg The cosine argument.
 *
 * @return A term representing the cosine of arg.
 */
shared_ptr<Term> TermBuilder::cos(shared_ptr<Term> arg) {
    return make_shared<Cos>(arg);
}

/**
 * Constructs a 2D quadratic form given the vector components x1, x2 and the matrix coefficients a11, a12, a21, a22.
 *
 * @param x1 First vector component
 * @param x2 Second vector component
 * @param a11 First row, first column matrix component
 * @param a12 First row, second column matrix component
 * @param a21 Second row, first column matrix component
 * @param a22 Second row, second column matrix component
 *
 * @return A term describing the quadratic form
 */
shared_ptr<Term> TermBuilder::quadform(shared_ptr<Term> x1, shared_ptr<Term> x2, shared_ptr<Term> a11,
        shared_ptr<Term> a12, shared_ptr<Term> a21, shared_ptr<Term> a22) {
    vector<shared_ptr<Term>> rest = {a22 * power(x2, 2)};
    return make_shared<Sum>(a11 * power(x1, 2), (a12 + a21) * x1 * x2, rest);
}

shared_ptr<Term> TermBuilder::normalDistribution(shared_ptr<TVec> args, shared_ptr<TVec> mean, double variance) {
    return exp((args - mean)->normSquared() * (-0.5 / variance)) * (1 / sqrt(2.0 * M_PI * variance));
}

shared_ptr<Term> TermBuilder::gaussian(shared_ptr<TVec> args, shared_ptr<TVec> mean, double variance) {
    return exp((args - mean)->normSquared() * (-0.5 / variance));
}

shared_ptr<Term> TermBuilder::sigmoid(shared_ptr<Term> arg, shared_ptr<Term> upperBound, shared_ptr<Term> lowerBound,
        shared_ptr<Term> mid, double steepness) {
    return (upperBound - lowerBound) * (make_shared<Sigmoid>(arg, mid, steepness)) + lowerBound;
}

shared_ptr<Term> TermBuilder::boundedValue(
        shared_ptr<Term> arg, shared_ptr<Term> leftBound, shared_ptr<Term> rightBound, double steepness) {
    return make_shared<LTConstraint>(leftBound, arg, steepness) & make_shared<LTConstraint>(arg, rightBound, steepness);
}

shared_ptr<Term> TermBuilder::boundedRectangle(
        shared_ptr<TVec> arg, shared_ptr<TVec> rightLower, shared_ptr<TVec> leftUpper, double steepness) {
    return boundedValue(arg->getX(), rightLower->getX(), leftUpper->getX(), steepness) &
           boundedValue(arg->getY(), rightLower->getY(), leftUpper->getY(), steepness);
}

shared_ptr<Term> TermBuilder::euclidianDistanceSqr(shared_ptr<TVec> one, shared_ptr<TVec> two) {
    return (one - two)->normSquared();
}

shared_ptr<Term> TermBuilder::euclidianDistance(shared_ptr<TVec> one, shared_ptr<TVec> two) {
    return power(euclidianDistanceSqr(one, two), 0.5);
}

shared_ptr<Term> TermBuilder::polynom(vector<shared_ptr<Term>> input, int degree, vector<shared_ptr<Term>> param) {
    shared_ptr<Term> ret = constant(0);
    for (int i = 0; i < input.size(); ++i) {
        shared_ptr<Term> t = constant(1);
        for (int j = 1; j < degree; ++j) {
            t = t * input[i];
        }
        ret = ret + t * param[i];
    }
    return ret;
}

} /* namespace autodiff */
