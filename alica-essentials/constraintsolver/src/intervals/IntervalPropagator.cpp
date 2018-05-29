/*
 * IntervalPropagator.cpp
 *
 *  Created on: Dec 5, 2014
 *      Author: Philipp
 */

#include "intervals/IntervalPropagator.h"

#include "CNSat.h"
#include "intervals/RecursivePropagate.h"
#include "intervals/ResetIntervals.h"
#include "intervals/UnsolveableException.h"
#include "types/Var.h"

#include <autodiff/TermPtr.h>
#include <autodiff/Variable.h>

#include <math.h>

#include <iostream>

//#define DEBUG_INTERVALPROP

namespace alica
{
namespace reasoner
{
namespace intervalpropagation
{
using std::make_shared;
using std::shared_ptr;
using std::vector;
int IntervalPropagator::updates = 0;
int IntervalPropagator::visits = 0;

IntervalPropagator::IntervalPropagator()
    : dim(0)
{
}

void IntervalPropagator::setGlobalRanges(autodiff::TermHolder& holder, std::shared_ptr<std::vector<std::shared_ptr<std::vector<double>>>> ranges,
                                         std::shared_ptr<cnsat::CNSat> solver)
{
    for (int i = 0; i < static_cast<int>(holder.getVariables().size()); ++i) {
        holder.getVariables()[i]->editRange().setMin(ranges->at(i)->at(0));
        holder.getVariables()[i]->editRange().setMax(ranges->at(i)->at(1));
    }
    this->globalRanges = ranges;
    this->vars = vars;
    this->dim = vars->size();
    this->solver = solver;
    this->updates = 0;
    this->visits = 0;
}

bool IntervalPropagator::propagate(std::shared_ptr<std::vector<std::shared_ptr<cnsat::Var>>> decisions,
                                   shared_ptr<std::vector<shared_ptr<vector<double>>>>& completeRanges,
                                   shared_ptr<std::vector<shared_ptr<cnsat::Var>>>& offenders)
{
#ifdef DEBUG_INTERVALPROP
    std::cout << "IntervalPropagator::propagate" << std::endl;
    for (auto& it : *decisions) {
        std::cout << "\t" << it->toString() << std::endl;
    }
#endif
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
            completeRanges->at(j)->at(0) = std::max(completeRanges->at(j)->at(0), curRanges->at(j)->at(0));
            completeRanges->at(j)->at(1) = std::min(completeRanges->at(j)->at(1), curRanges->at(j)->at(1));
            if (completeRanges->at(j)->at(0) > completeRanges->at(j)->at(1)) { // ranges collapsed, build offenders
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

bool IntervalPropagator::prePropagate(shared_ptr<vector<shared_ptr<cnsat::Var>>> vars)
{
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

bool IntervalPropagator::propagate(autodiff::TermPtr term)
{
    term->acceptRecursive(&_ri); // TODO: fold this into rp's visitation
    // term->accept(this->sp);
    term->setMin(1.0);
    term->setMax(1.0);
    updates = 0;
    try {
        _rp.propagate(term);
    } catch (const int& i) {
#ifdef RecPropDEBUG
        //				cout << "Unsolvable: " << ue << endl;
        std::cout << updates << " update steps" << std::endl;
#endif
        return false;
    }
#ifdef RecPropDEBUG
    std::cout << updates << " update steps" << std::endl;
#endif
    return true;
}

bool IntervalPropagator::propagateSingle(std::shared_ptr<cnsat::Var> v, bool sign)
{
    for (int i = dim - 1; i >= 0; --i) {
        vars->at(i)->setMax(vars->at(i)->getRange().getMax());
        vars->at(i)->setMin(vars->at(i)->getRange().getMin());
    }

    if (sign) {
        if (!propagate(v->_term))
            return false;
    } else {
        if (!propagate(v->_term->negate()))
            return false;
    }

    double rangeSize = 0;
    shared_ptr<vector<shared_ptr<vector<double>>>> range = make_shared<vector<shared_ptr<vector<double>>>>(dim);
    for (int i = dim - 1; i >= 0; --i) {
        range->at(i) = make_shared<vector<double>>(2);
        range->at(i)->at(0) = vars->at(i)->getMin();
        range->at(i)->at(1) = vars->at(i)->getMax();
        double d = vars->at(i)->getMax() - vars->at(i)->getMin();
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
