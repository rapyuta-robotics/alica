/*
 * Max.cpp
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#include "Max.h"

#include "TermBuilder.h"
#include "Constant.h"

#include <cmath>

namespace autodiff {
Max::Max(shared_ptr<Term> left, shared_ptr<Term> right)
        : Term() {
    this->left = left;
    this->right = right;
}

int Max::accept(shared_ptr<ITermVisitor> visitor) {
    shared_ptr<Max> thisCasted = dynamic_pointer_cast<Max>(shared_from_this());
    return visitor->visit(thisCasted);
}

shared_ptr<Term> Max::aggregateConstants() {
    left = left->aggregateConstants();
    if (left == Term::TRUE) {
        return left;
    }
    right = right->aggregateConstants();
    if (left == Term::FALSE) {
        return right;
    }
    if (right == Term::TRUE) {
        return right;
    }
    if (right == Term::FALSE) {
        return left;
    }
    if (dynamic_pointer_cast<Constant>(left) != 0 && dynamic_pointer_cast<Constant>(right) != 0) {
        shared_ptr<Constant> leftConstant = dynamic_pointer_cast<Constant>(left);
        shared_ptr<Constant> rightConstant = dynamic_pointer_cast<Constant>(right);
        return make_shared<Constant>(std::max(leftConstant->value, rightConstant->value));
    } else {
        return shared_from_this();
    }
}

shared_ptr<Term> Max::derivative(shared_ptr<Variable> v) {
    throw "Symbolic Derivation of Max not supported.";
}

shared_ptr<Term> Max::negate() {
    return left->negate() & right->negate();
}

string Max::toString() {
    string str;
    str.append("max( ");
    str.append(left->toString());
    str.append(", ");
    str.append(right->toString());
    str.append(" )");
    return str;
}
} /* namespace autodiff */
