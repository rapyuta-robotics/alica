/*
 * Log.cpp
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#include "Log.h"

#include "TermBuilder.h"
#include "Constant.h"

#include <cmath>
#include <limits>

namespace autodiff {
Log::Log(shared_ptr<Term> arg)
        : Term() {
    this->arg = arg;
}

int Log::accept(shared_ptr<ITermVisitor> visitor) {
    shared_ptr<Log> thisCasted = dynamic_pointer_cast<Log>(shared_from_this());
    return visitor->visit(thisCasted);
}

shared_ptr<Term> Log::aggregateConstants() {
    arg = arg->aggregateConstants();
    if (dynamic_pointer_cast<Constant>(arg) != 0) {
        shared_ptr<Constant> arg = dynamic_pointer_cast<Constant>(arg);
        return TermBuilder::constant(log(arg->value));
    } else {
        return shared_from_this();
    }
}

shared_ptr<Term> Log::derivative(shared_ptr<Variable> v) {
    return arg->derivative(v) / arg;
}

string Log::toString() {
    string str;
    str.append("log( ");
    str.append(arg->toString());
    str.append(" )");
    return str;
}
} /* namespace autodiff */
