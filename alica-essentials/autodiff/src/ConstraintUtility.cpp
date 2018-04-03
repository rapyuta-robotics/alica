/*
 * ConstraintUtility.cpp
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#include "ConstraintUtility.h"

#include "TermBuilder.h"
#include "Constant.h"

#include <cmath>

namespace autodiff {
ConstraintUtility::ConstraintUtility(shared_ptr<Term> constraint, shared_ptr<Term> utility)
        : Term() {
    this->constraint = constraint;
    this->utility = utility;
}

int ConstraintUtility::accept(shared_ptr<ITermVisitor> visitor) {
    shared_ptr<ConstraintUtility> thisCasted = dynamic_pointer_cast<ConstraintUtility>(shared_from_this());
    return visitor->visit(thisCasted);
}

shared_ptr<Term> ConstraintUtility::aggregateConstants() {
    constraint = constraint->aggregateConstants();
    utility = utility->aggregateConstants();
    return shared_from_this();
}

shared_ptr<Term> ConstraintUtility::derivative(shared_ptr<Variable> v) {
    throw "Symbolic Derivation of ConstraintUtility not supported.";
}

shared_ptr<Term> ConstraintUtility::negate() {
    throw "Do not negate a Constraint Utility";
}

string ConstraintUtility::toString() {
    string str;
    str.append("[ConstraintUtility: Constraint=");
    str.append(constraint->toString());
    str.append(", Utility=");
    str.append(utility->toString());
    str.append("]");
    return str;
}
} /* namespace autodiff */
