/*
 * Sin.cpp
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#include "Sin.h"

#include "TermBuilder.h"
#include "Constant.h"
#include "Cos.h"
#include "Zero.h"

#include <cmath>

namespace autodiff {
Sin::Sin(shared_ptr<Term> arg) : Term() {
    this->arg = arg;
}

int Sin::accept(shared_ptr<ITermVisitor> visitor) {
    shared_ptr<Sin> thisCasted = dynamic_pointer_cast<Sin>(shared_from_this());
    return visitor->visit(thisCasted);
}

shared_ptr<Term> Sin::aggregateConstants() {
    arg = arg->aggregateConstants();
    if (dynamic_pointer_cast<Constant>(arg) != 0) {
        shared_ptr<Constant> arg = dynamic_pointer_cast<Constant>(arg);
        return TermBuilder::constant(sin(arg->value));
    } else {
        if (dynamic_pointer_cast<Zero>(arg) != 0) {
            return TermBuilder::constant(0);
        }
        return shared_from_this();
    }
}

shared_ptr<Term> Sin::derivative(shared_ptr<Variable> v) {
    return make_shared<Cos>(arg) * arg->derivative(v);
}

string Sin::toString() {
    string str;
    str.append("sin( ");
    str.append(arg->toString());
    str.append(" )");
    return str;
}
} /* namespace autodiff */
