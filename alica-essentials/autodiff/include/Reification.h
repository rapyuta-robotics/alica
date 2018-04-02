/*
 * Reification.h
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#ifndef REIFICATION_H_
#define REIFICATION_H_

#include "Term.h"

#include <iostream>

using namespace std;

namespace autodiff {

class Reification : public Term {
public:
    Reification(shared_ptr<Term> condition, double min, double max);

    shared_ptr<Term> condition;
    shared_ptr<Term> negatedCondition;
    double min;
    double max;

    int accept(shared_ptr<ITermVisitor> visitor);

    shared_ptr<Term> aggregateConstants();
    shared_ptr<Term> derivative(shared_ptr<Variable> v);

    string toString();
};

} /* namespace autodiff */

#endif /* REIFICATION_H_ */
