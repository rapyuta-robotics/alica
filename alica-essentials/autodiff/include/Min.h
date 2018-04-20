/*
 * Min.h
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#ifndef MIN_H_
#define MIN_H_

#include "Term.h"

namespace autodiff {

class Min : public Term {
public:
    Min(shared_ptr<Term> left, shared_ptr<Term> right);

    shared_ptr<Term> left;
    shared_ptr<Term> right;

    int accept(shared_ptr<ITermVisitor> visitor);

    shared_ptr<Term> aggregateConstants();
    shared_ptr<Term> derivative(shared_ptr<Variable> v);
    shared_ptr<Term> negate();

    string toString();
};

} /* namespace autodiff */

#endif /* MIN_H_ */
