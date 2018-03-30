/*
 * Sum.h
 *
 *  Created on: Jun 12, 2014
 *      Author: psp
 */

#ifndef SUM_H_
#define SUM_H_

#include "Term.h"

#include <vector>

namespace autodiff {

class Sum : public Term {
public:
    Sum(vector<shared_ptr<Term>> terms);
    Sum(shared_ptr<Term> first, shared_ptr<Term> second, vector<shared_ptr<Term>> rest);

    vector<shared_ptr<Term>> terms;

    int accept(shared_ptr<ITermVisitor> visitor);

    shared_ptr<Term> aggregateConstants();
    shared_ptr<Term> derivative(shared_ptr<Variable> v);

    string toString();
};

} /* namespace autodiff */

#endif /* SUM_H_ */
