/*
 * UpwardPropagator.h
 *
 *  Created on: Dec 12, 2014
 *      Author: Philipp
 */

#ifndef UPWARDPROPAGATOR_H_
#define UPWARDPROPAGATOR_H_

#include <AutoDiff.h>

#include <memory>

using namespace std;
using namespace autodiff;

namespace alica {
namespace reasoner {
namespace intervalpropagation {
class DownwardPropagator;
class TermList;

class UpwardPropagator : public autodiff::ITermVisitor {
public:
    UpwardPropagator();
    virtual ~UpwardPropagator();

    shared_ptr<TermList> changed;
    shared_ptr<DownwardPropagator> dp;

    int visit(shared_ptr<Abs> abs);
    int visit(shared_ptr<And> and_);
    int visit(shared_ptr<Atan2> atan2);
    int visit(shared_ptr<Constant> constant);
    int visit(shared_ptr<ConstPower> intPower);
    int visit(shared_ptr<ConstraintUtility> cu);
    int visit(shared_ptr<Cos> cos);
    int visit(shared_ptr<Exp> exp);
    int visit(shared_ptr<Gp> gp);
    int visit(shared_ptr<LinSigmoid> sigmoid);
    int visit(shared_ptr<Log> log);
    int visit(shared_ptr<LTConstraint> constraint);
    int visit(shared_ptr<LTEConstraint> constraint);
    int visit(shared_ptr<Max> max);
    int visit(shared_ptr<Min> min);
    int visit(shared_ptr<Or> or_);
    int visit(shared_ptr<Product> product);
    int visit(shared_ptr<Reification> dis);
    int visit(shared_ptr<Sigmoid> sigmoid);
    int visit(shared_ptr<Sin> sin);
    int visit(shared_ptr<Sum> sum);
    int visit(shared_ptr<TermPower> power);
    int visit(shared_ptr<Variable> var);
    int visit(shared_ptr<Zero> zero);

protected:
    void addChanged(shared_ptr<Term> t);
    void outputChange(shared_ptr<Term> t, double oldmin, double oldmax);
    bool updateInterval(shared_ptr<Term> t, double min, double max);
};

} /* namespace intervalpropagation */
} /* namespace reasoner */
} /* namespace alica */

#endif /* UPWARDPROPAGATOR_H_ */
