/*
 * Abs.cpp
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#include "Abs.h"

#include "TermBuilder.h"
#include "Constant.h"

#include <cmath>

namespace autodiff {
Abs::Abs(shared_ptr<Term> arg)
        : Term() {
    this->arg = arg;
}

int Abs::accept(shared_ptr<ITermVisitor> visitor) {
    shared_ptr<Abs> thisCasted = dynamic_pointer_cast<Abs>(shared_from_this());
    return visitor->visit(thisCasted);
}

shared_ptr<Term> Abs::aggregateConstants() {
    arg = arg->aggregateConstants();
    if (dynamic_pointer_cast<Constant>(arg) != 0) {
        shared_ptr<Constant> constArg = dynamic_pointer_cast<Constant>(arg);
        return TermBuilder::constant(fabs(constArg->value));
    } else {
        return shared_from_this();
    }
}
shared_ptr<Term> Abs::derivative(shared_ptr<Variable> v) {
    return arg->derivative(v) * arg / shared_from_this();
}

string Abs::toString() {
    string str;
    str.append("abs( ");
    str.append(arg->toString());
    str.append(" )");
    return str;
}
} /* namespace autodiff */
