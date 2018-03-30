/*
 * Zero.cpp
 *
 *  Created on: Jun 6, 2014
 *      Author: psp
 */

#include "Zero.h"

namespace autodiff {
int Zero::accept(shared_ptr<ITermVisitor> visitor) {
    shared_ptr<Zero> thisCasted = dynamic_pointer_cast<Zero>(shared_from_this());
    return visitor->visit(thisCasted);
}

shared_ptr<Term> Zero::aggregateConstants() {
    return shared_from_this();
}

shared_ptr<Term> Zero::derivative(shared_ptr<Variable> v) {
    return shared_from_this();
}

string Zero::toString() {
    return "0";
}
} /* namespace autodiff */
