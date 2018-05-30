/*
 * ConstraintUtility.cpp
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#include "ConstraintUtility.h"

#include "Constant.h"
#include "TermHolder.h"

#include <cmath>
#include <sstream>

namespace autodiff
{
ConstraintUtility::ConstraintUtility(TermPtr constraint, TermPtr utility, TermHolder* owner)
    : BinaryFunction(constraint, utility, owner)

{
}

int ConstraintUtility::accept(ITermVisitor* visitor)
{
    return visitor->visit(this);
}

void ConstraintUtility::acceptRecursive(ITermVisitor* visitor)
{
    _left->acceptRecursive(visitor);
    _right->acceptRecursive(visitor);
    visitor->visit(this);
}

TermPtr ConstraintUtility::aggregateConstants()
{
    _left = _left->aggregateConstants();
    _right = _right->aggregateConstants();
    return this;
}

TermPtr ConstraintUtility::derivative(VarPtr v) const
{
    throw "Symbolic derivation of ConstraintUtility not supported.";
    return nullptr;
}

TermPtr ConstraintUtility::negate() const
{
    throw "Cannot negate a Constraint Utility";
    return nullptr;
}

std::string ConstraintUtility::toString() const
{
    std::stringstream str;
    str << "[ConstraintUtility: Constraint=" << _left->toString();
    str << ", Utility=" << _right->toString() << "]";
    return str.str();
}

void ConstraintUtility::Eval(const Tape& tape, const Parameter* params, double* result, const double* vars, int dim)
{
    const double* l = tape.getValues(params[0].asIdx);
    const double* r = tape.getValues(params[1].asIdx);
    if (l[0] > 0.75) {
        for (int i = 0; i <= dim; ++i) {
            result[i] = r[i];
        }
    } else {
        for (int i = 0; i <= dim; ++i) {
            result[i] = l[i];
        }
    }
}

} /* namespace autodiff */
