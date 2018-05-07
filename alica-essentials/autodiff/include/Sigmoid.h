/*
 * Sigmoid.h
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#ifndef SIGMOID_H_
#define SIGMOID_H_

#include "Term.h"

namespace autodiff {

class Sigmoid : public Term {
public:
    Sigmoid(shared_ptr<Term> arg, shared_ptr<Term> mid);
    Sigmoid(shared_ptr<Term> arg, shared_ptr<Term> mid, double steppness);

    shared_ptr<Term> arg;
    shared_ptr<Term> mid;
    double steepness;

    int accept(shared_ptr<ITermVisitor> visitor);

    shared_ptr<Term> aggregateConstants();
    shared_ptr<Term> derivative(shared_ptr<Variable> v);

    string toString();
};

} /* namespace autodiff */

#endif /* SIGMOID_H_ */
