/*
 * Gp.cpp
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#include "Gp.h"

namespace autodiff {
Gp::Gp(vector<shared_ptr<Term>> args, shared_ptr<GaussianProcess> gp, int dc)
        : Term() {
    this->divCount = dc;
    this->args = args;
    this->gpr = gp;
}

int Gp::accept(shared_ptr<ITermVisitor> visitor) {
    shared_ptr<Gp> thisCasted = dynamic_pointer_cast<Gp>(shared_from_this());
    return visitor->visit(thisCasted);
}

shared_ptr<Term> Gp::aggregateConstants() {
    return shared_from_this();
}
shared_ptr<Term> Gp::derivative(shared_ptr<Variable> v) {
    throw "NotImplementedException";
}

string Gp::toString() {
    throw "NotImplementedException";
    string str;
    return str;
}

double Gp::eval() {
    return 0;
}
} /* namespace autodiff */
