/*
 * IntervalPropagator.cpp
 *
 *  Created on: Dec 5, 2014
 *      Author: Philipp
 */

#include "intervals/IntervalPropagator.h"

#include "intervals/RecursivePropagate.h"
#include "intervals/ResetIntervals.h"
#include "intervals/UnsolveableException.h"
#include "types/Var.h"
#include "CNSat.h"

#include <math.h>

#include <iostream>

namespace alica {
namespace reasoner {
namespace intervalpropagation {
int IntervalPropagator::updates = 0;
int IntervalPropagator::visits = 0;

IntervalPropagator::IntervalPropagator() {
    this->ri = make_shared<ResetIntervals>();
    this->rp = make_shared<RecursivePropagate>();
}

IntervalPropagator::~IntervalPropagator() {
    // TODO Auto-generated destructor stub
}

void IntervalPropagator::setGlobalRanges(shared_ptr<vector<shared_ptr<Variable>>> vars,
        shared_ptr<vector<shared_ptr<vector<double>>>> ranges, shared_ptr<cnsat::CNSat> solver) {
    for (int i = 0; i < vars->size(); ++i) {
        vars->at(i)->globalMin = ranges->at(i)->at(0);
        vars->at(i)->globalMax = ranges->at(i)->at(1);
    }
    this->globalRanges = ranges;
    this->vars = vars;
    this->dim = vars->size();
    this->solver = solver;
    this->updates = 0;
    this->visits = 0;
}

bool IntervalPropagator::propagate(shared_ptr<vector<shared_ptr<cnsat::Var>>> decisions,
        shared_ptr<vector<shared_ptr<vector<double>>>>& completeRanges,
        shared_ptr<vector<shared_ptr<cnsat::Var>>>& offenders) {
    cout << "IntervalPropagator::propagate" << endl;
    for (auto& it : *decisions) {
        cout << "\t" << it->toString() << endl;
    }
    offenders = nullptr;
    completeRanges = make_shared<vector<shared_ptr<vector<double>>>>(dim);
    completeRanges->insert(completeRanges->begin(), globalRanges->begin(), globalRanges->end());
    for (int i = decisions->size() - 1; i >= 0; i--) {
        shared_ptr<vector<shared_ptr<vector<double>>>> curRanges;
        if (decisions->at(i)->assignment == cnsat::Assignment::TRUE) {
            if (decisions->at(i)->positiveRanges == nullptr) {
                if (!propagateSingle(decisions->at(i), true)) {
                    offenders = make_shared<vector<shared_ptr<cnsat::Var>>>();
                    offenders->push_back(decisions->at(i));
                    return false;
                }
            }
            curRanges = decisions->at(i)->positiveRanges;
        } else {
            if (decisions->at(i)->negativeRanges == nullptr) {
                if (!propagateSingle(decisions->at(i), false)) {
                    offenders = make_shared<vector<shared_ptr<cnsat::Var>>>();
                    offenders->push_back(decisions->at(i));
                    return false;
                }
            }
            curRanges = decisions->at(i)->negativeRanges;
        }
        for (int j = dim - 1; j >= 0; j--) {
            completeRanges->at(j)->at(0) = max(completeRanges->at(j)->at(0), curRanges->at(j)->at(0));
            completeRanges->at(j)->at(1) = min(completeRanges->at(j)->at(1), curRanges->at(j)->at(1));
            if (completeRanges->at(j)->at(0) > completeRanges->at(j)->at(1)) {  // ranges collapsed, build offenders
                offenders = make_shared<vector<shared_ptr<cnsat::Var>>>();
                for (int k = decisions->size() - 1; k >= i; k--) {
                    offenders->push_back(decisions->at(k));
                }
                return false;
            }
        }
    }
    return true;
}

bool IntervalPropagator::prePropagate(shared_ptr<vector<shared_ptr<cnsat::Var>>> vars) {
    for (int i = vars->size() - 1; i >= 0; --i) {
        if (vars->at(i)->assignment != cnsat::Assignment::FALSE && !propagateSingle(vars->at(i), true)) {
            if (!solver->preAddIUnitClause(vars->at(i), cnsat::Assignment::FALSE)) {
                return false;
            }
        }
        if (vars->at(i)->assignment != cnsat::Assignment::TRUE && !propagateSingle(vars->at(i), false)) {
            if (!solver->preAddIUnitClause(vars->at(i), cnsat::Assignment::TRUE)) {
                return false;
            }
        }
    }
    return true;
}

bool IntervalPropagator::propagate(shared_ptr<Term> term) {
    term->accept(this->ri);
    // term->accept(this->sp);
    term->min = 1;
    term->max = 1;
    updates = 0;
    try {
        this->rp->propagate(term);
    } catch (int& i) {
#ifdef RecPropDEBUG
        //				cout << "Unsolvable: " << ue << endl;
        cout << updates << " update steps" << endl;
#endif
        return false;
    }
#ifdef RecPropDEBUG
    cout << updates << " update steps" << endl;
#endif
    return true;
}

bool IntervalPropagator::propagateSingle(shared_ptr<cnsat::Var> v, bool sign) {
    for (int i = dim - 1; i >= 0; i--) {
        vars->at(i)->max = vars->at(i)->globalMax;
        vars->at(i)->min = vars->at(i)->globalMin;
    }

    if (sign) {
        if (!propagate(v->term))
            return false;
    } else {
        if (!propagate(v->term->negate()))
            return false;
    }

    double rangeSize = 0;
    shared_ptr<vector<shared_ptr<vector<double>>>> range = make_shared<vector<shared_ptr<vector<double>>>>(dim);
    for (int i = dim - 1; i >= 0; --i) {
        range->at(i) = make_shared<vector<double>>(2);
        range->at(i)->at(0) = vars->at(i)->min;
        range->at(i)->at(1) = vars->at(i)->max;
        double d = vars->at(i)->max - vars->at(i)->min;
        rangeSize += d * d;
    }
    if (sign) {
        v->positiveRanges = range;
        v->positiveRangeSize = rangeSize;
    } else {
        v->negativeRanges = range;
        v->negativeRangeSize = rangeSize;
    }
    return true;
}

} /* namespace intervalpropagation */
} /* namespace reasoner */
} /* namespace alica */
