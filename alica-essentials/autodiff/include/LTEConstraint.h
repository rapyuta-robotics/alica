/*
 * LTEConstraint.h
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#ifndef LTECONSTRAINT_H_
#define LTECONSTRAINT_H_

#include "Term.h"

namespace autodiff {

class LTEConstraint : public Term {
public:
    LTEConstraint(shared_ptr<Term> x, shared_ptr<Term> y, double steppness);
    LTEConstraint(shared_ptr<Term> x, shared_ptr<Term> y, double steppness, shared_ptr<Term> negatedForm);

    shared_ptr<Term> left;
    shared_ptr<Term> right;
    double steppness;
    shared_ptr<Term> negatedform;

    int accept(shared_ptr<ITermVisitor> visitor);

    shared_ptr<Term> aggregateConstants();
    shared_ptr<Term> derivative(shared_ptr<Variable> v);
    shared_ptr<Term> negate();

    string toString();
};

} /* namespace autodiff */

#endif /* LTECONSTRAINT_H_ */
