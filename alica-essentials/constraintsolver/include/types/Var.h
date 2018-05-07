/*
 * Var.h
 *
 *  Created on: Dec 4, 2014
 *      Author: Philipp
 */

#ifndef VAR_H_
#define VAR_H_

#include "types/Assignment.h"

#include <AutoDiff.h>

#include <memory>

using namespace std;
using namespace autodiff;
using namespace autodiff::compiled;

namespace alica {
namespace reasoner {
namespace cnsat {
class Clause;
class DecisionLevel;
class Watcher;

class Var {
public:
    Var(int index, bool prefSign = true);
    virtual ~Var();

    void reset();
    shared_ptr<Clause> getReason();
    void setReason(shared_ptr<Clause> reason);

    void print();
    string toString();

    shared_ptr<vector<Watcher*>> watchList;
    int index;
    int activity;
    int negActivity;
    bool locked;
    Assignment assignment;
    bool seen;
    bool preferedSign;
    shared_ptr<DecisionLevel> decisionLevel;

    int positiveAppearance;
    int negativeAppearance;

    shared_ptr<Term> term;
    shared_ptr<vector<shared_ptr<vector<double>>>> positiveRanges;
    shared_ptr<vector<shared_ptr<vector<double>>>> negativeRanges;

    double positiveRangeSize;
    double negativeRangeSize;

    shared_ptr<ICompiledTerm> positiveTerm;
    shared_ptr<ICompiledTerm> negativeTerm;
    shared_ptr<ICompiledTerm> curTerm;

private:
    shared_ptr<Clause> reason;
};

} /* namespace cnsat */
} /* namespace reasoner */
} /* namespace alica */

#endif /* VAR_H_ */
