/*
 * Sin.h
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#ifndef SIN_H_
#define SIN_H_

#include "Term.h"

namespace autodiff {

class Sin : public Term {
public:
    Sin(shared_ptr<Term> arg);

    shared_ptr<Term> arg;

    int accept(shared_ptr<ITermVisitor> visitor);

    shared_ptr<Term> aggregateConstants();
    shared_ptr<Term> derivative(shared_ptr<Variable> v);

    string toString();
};

} /* namespace autodiff */

#endif /* SIN_H_ */
