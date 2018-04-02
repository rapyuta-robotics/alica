/*
 * Abs.h
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#ifndef ABS_H_
#define ABS_H_

#include "Term.h"

#include <iostream>

using namespace std;

namespace autodiff {

class Abs : public Term {
public:
    Abs(shared_ptr<Term> arg);

    shared_ptr<Term> arg;

    int accept(shared_ptr<ITermVisitor> visitor);

    shared_ptr<Term> aggregateConstants();
    shared_ptr<Term> derivative(shared_ptr<Variable> v);

    string toString();
};

} /* namespace autodiff */

#endif /* ABS_H_ */
