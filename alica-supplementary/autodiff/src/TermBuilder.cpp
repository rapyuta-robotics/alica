/*
 * TermBuilder.cpp
 *
 *  Created on: Jun 6, 2014
 *      Author: psp
 */

#include "TermBuilder.h"

#include "ConstPower.h"
#include "Constant.h"
#include "Cos.h"
#include "Exp.h"
#include "LTConstraint.h"
#include "LTEConstraint.h"
#include "Log.h"
#include "Product.h"
#include "Sigmoid.h"
#include "Sin.h"
#include "Sum.h"
#include "TVec.h"
#include "Term.h"
#include "TermPower.h"
#include "TermPtr.h"

#include <cmath>

namespace autodiff
{

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
TermPtr TermBuilder::quadform(const TermPtr x1, const TermPtr x2, const TermPtr a11, const TermPtr a12, const TermPtr a21, const TermPtr a22)
{
    return a11 * x1->getOwner()->constPower(x1, 2) + (a12 + a21) * x1 * x2 + a22 * x2->getOwner()->constPower(x2, 2);
}

TermPtr TermBuilder::sigmoid(const TermPtr arg, const TermPtr upperBound, const TermPtr lowerBound, const TermPtr mid, double steepness)
{
    return (upperBound - lowerBound) * (arg->getOwner()->sigmoid(arg, mid, steepness)) + lowerBound;
}

TermPtr TermBuilder::boundedValue(TermPtr arg, TermPtr leftBound, TermPtr rightBound)
{
    return arg->getOwner()->lessThanEqual(leftBound, arg) & arg->getOwner()->lessThanEqual(arg, rightBound);
}

TermPtr TermBuilder::boundedRectangle(const TVec<2>& arg, const TVec<2>& rightLower, const TVec<2>& leftUpper)
{
    return boundedValue(arg.getX(), rightLower.getX(), leftUpper.getX()) & boundedValue(arg.getY(), rightLower.getY(), leftUpper.getY());
}
} /* namespace autodiff */
