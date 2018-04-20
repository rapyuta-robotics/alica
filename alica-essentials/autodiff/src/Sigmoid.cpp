/*
 * Sigmoid.cpp
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#include "Sigmoid.h"

#include "TermBuilder.h"
#include "Constant.h"
#include "ConstPower.h"
#include "Exp.h"

#include <cmath>
#include <limits>

namespace autodiff {

Sigmoid::Sigmoid(shared_ptr<Term> arg, shared_ptr<Term> mid)
        : Term() {
    this->arg = arg;
    this->mid = mid;
    this->steepness = 1;
}

Sigmoid::Sigmoid(shared_ptr<Term> arg, shared_ptr<Term> mid, double steppness)
        : Term() {
    this->arg = arg;
    this->mid = mid;
    this->steepness = steppness;
}

int Sigmoid::accept(shared_ptr<ITermVisitor> visitor) {
    shared_ptr<Sigmoid> thisCasted = dynamic_pointer_cast<Sigmoid>(shared_from_this());
    return visitor->visit(thisCasted);
}

shared_ptr<Term> Sigmoid::aggregateConstants() {
    arg = arg->aggregateConstants();
    mid = mid->aggregateConstants();
    if (dynamic_pointer_cast<Constant>(arg) != 0 && dynamic_pointer_cast<Constant>(mid) != 0) {
        shared_ptr<Constant> arg = dynamic_pointer_cast<Constant>(arg);
        shared_ptr<Constant> mid = dynamic_pointer_cast<Constant>(mid);
        double e = exp(steepness * (-arg->value + mid->value));
        if (e == numeric_limits<double>::infinity()) {
            return TermBuilder::constant(Term::EPSILON);
        } else {
            e = 1.0 / (1.0 + e);
        }
        if (e < Term::EPSILON) {
            return TermBuilder::constant(Term::EPSILON);
        } else {
            return TermBuilder::constant(e);
        }
    } else {
        return shared_from_this();
    }
}

shared_ptr<Term> Sigmoid::derivative(shared_ptr<Variable> v) {
    shared_ptr<Term> t =
            steepness * (arg->derivative(v) - mid->derivative(v)) * make_shared<Exp>(steepness * (-1 * arg + mid));
    return t / make_shared<ConstPower>(make_shared<Exp>(steepness * arg) + make_shared<Exp>(steepness * mid), 2);
}

string Sigmoid::toString() {
    string str;
    str.append("sigmoid( ");
    str.append(arg->toString());
    str.append(", ");
    str.append(mid->toString());
    str.append(", ");
    str.append("", steepness);
    str.append(" )");
    return str;
}
} /* namespace autodiff */
