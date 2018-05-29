/*
 * UpwardPropagator.cpp
 *
 *  Created on: Dec 12, 2014
 *      Author: Philipp
 */

#include "intervals/UpwardPropagator.h"
//#define DEBUG_UP

#include "intervals/DownwardPropagator.h"
#include "intervals/IntervalPropagator.h"
#include "intervals/UnsolveableException.h"

#include "autodiff/AutoDiff.h"

#include <cmath>
#include <limits>
#include <math.h>

#include <iostream>

namespace alica
{
namespace reasoner
{
namespace intervalpropagation
{

using std::numeric_limits;

UpwardPropagator::UpwardPropagator() {}

int UpwardPropagator::visit(autodiff::Abs* abs)
{
    bool containsZero = abs->getArg()->getMin() * abs->getArg()->getMax() <= 0;
    bool c = false;
    if (containsZero)
        c = updateInterval(abs, 0, std::max(std::abs(abs->getArg()->getMin()), std::abs(abs->getArg()->getMax())));
    else
        c = updateInterval(abs, std::min(std::abs(abs->getArg()->getMin()), std::abs(abs->getArg()->getMax())),
                           std::max(std::abs(abs->getArg()->getMin()), std::abs(abs->getArg()->getMax())));
    if (c)
        addChanged(abs);
    return c;
}

int UpwardPropagator::visit(autodiff::And* and_)
{
    if (and_->getLeft()->getMin() > 0 && and_->getRight()->getMin() > 0) {
        if (updateInterval(and_, 1, 1)) {
            addChanged(and_);
            return true;
        }
    } else if (and_->getLeft()->getMax() <= 0 || and_->getRight()->getMax() <= 0) {
        if (updateInterval(and_, -numeric_limits<double>::infinity(), 0)) {
            addChanged(and_);
            return true;
        }
    }
    return false;
}

int UpwardPropagator::visit(autodiff::Atan2* atan2)
{
    throw "Atan2 propagation not implemented!";
}

int UpwardPropagator::visit(autodiff::Constant* constant)
{
    return false;
}

int UpwardPropagator::visit(autodiff::ConstPower* intPower)
{
    bool includesZero = intPower->getBase()->getMin() * intPower->getBase()->getMax() <= 0;
    if (intPower->getExponent() > 0) {
        double a = pow(intPower->getBase()->getMax(), intPower->getExponent());
        double b = pow(intPower->getBase()->getMin(), intPower->getExponent());
        if (includesZero) {
            if (updateInterval(intPower, std::min(0.0, std::min(a, b)), std::max(0.0, std::max(a, b)))) {
                // if(updateInterval(intPower,std::min(0,pow(intPower->base->getMin(),intPower->exponent)),std::max(0,pow(intPower->base->getMax(),intPower->exponent))))
                // {
                addChanged(intPower);
                return true;
            }
        } else {
            if (updateInterval(intPower, std::min(a, b), std::max(a, b))) {
                // if(updateInterval(intPower,pow(intPower->base->getMin(),intPower->exponent),pow(intPower->base->getMax(),intPower->exponent)))
                // {
                addChanged(intPower);
                return true;
            }
        }
    } else if (!includesZero) {
        double a = pow(intPower->getBase()->getMax(), intPower->getExponent());
        double b = pow(intPower->getBase()->getMin(), intPower->getExponent());

        // Console.WriteLine("Cur: {0} [{1} : {2}]",intPower,intPower->getMin(),intPower->getMax());
        // Console.WriteLine("Base: [{0} : {1}]",intPower->base->getMin(),intPower->base->getMax());

        if (updateInterval(intPower, std::min(a, b), std::max(a, b))) {
            // Console.WriteLine("From UW intpower {0}",intPower->exponent);
            // if(updateInterval(intPower,pow(intPower->base->getMax(),intPower->exponent),pow(intPower->base->getMin(),intPower->exponent)))
            // {
            addChanged(intPower);
            return true;
        }
    } // else +- Infinity is possible
    return false;
}

int UpwardPropagator::visit(autodiff::ConstraintUtility* cu)
{
    if (cu->getLeft()->getMax() < 1) {
        if (updateInterval(cu, -std::numeric_limits<double>::infinity(), cu->getLeft()->getMax())) {
            addChanged(cu);
            return true;
        }
    }
    if (updateInterval(cu, -std::numeric_limits<double>::infinity(), cu->getRight()->getMax())) {
        addChanged(cu);
        return true;
    }
    return false;
}

int UpwardPropagator::visit(autodiff::Cos* cos)
{
    double size = cos->getArg()->getMax() - cos->getArg()->getMin();
    bool c = false;
    if (size <= 2 * M_PI) {
        double a = std::cos(cos->getArg()->getMax());
        double b = std::cos(cos->getArg()->getMin());
        double x = ceil(cos->getArg()->getMin() / M_PI);
        double y = floor(cos->getArg()->getMax() / M_PI);
        if (x == y) {                // single extrema
            if (((int)x) % 2 == 0) { // maxima
                c = updateInterval(cos, std::min(a, b), 1);
            } else { // minima
                c = updateInterval(cos, -1, std::max(a, b));
            }
        } else if (x > y) { // no extrema
            c = updateInterval(cos, std::min(a, b), std::max(a, b));
        } // multiple extrema, don't update
    }
    if (c) {
        addChanged(cos);
    }
    return c;
}

int UpwardPropagator::visit(autodiff::Exp* exp)
{
    if (updateInterval(exp, std::exp(exp->getArg()->getMin()), std::exp(exp->getArg()->getMax()))) {
        addChanged(exp);
        return true;
    }
    return false;
}

int UpwardPropagator::visit(autodiff::LinSigmoid* sigmoid)
{
    throw "Sigmoidal propagation not implemented";
}

int UpwardPropagator::visit(autodiff::Log* log)
{
    if (updateInterval(log, std::log(log->getArg()->getMin()), std::log(log->getArg()->getMax()))) {
        addChanged(log);
        return true;
    }
    return false;
}

int UpwardPropagator::visit(autodiff::LTConstraint* constraint)
{
    if (constraint->getLeft()->getMax() < constraint->getRight()->getMin()) {
        if (updateInterval(constraint, 1, 1)) {
            addChanged(constraint);
            return true;
        }
    } else if (constraint->getLeft()->getMin() >= constraint->getRight()->getMax()) {
        // Console.WriteLine("LT UP negated: {0} {1}",constraint->getLeft()->getMin() ,constraint->getRight()->getMax());
        if (updateInterval(constraint, -numeric_limits<double>::infinity(), 0)) {
            addChanged(constraint);
            return true;
        }
    }
    return false;
}

int UpwardPropagator::visit(autodiff::LTEConstraint* constraint)
{
    if (constraint->getLeft()->getMax() <= constraint->getRight()->getMin()) {
        if (updateInterval(constraint, 1, 1)) {
            addChanged(constraint);
            return true;
        }
    } else if (constraint->getLeft()->getMin() > constraint->getRight()->getMax()) {
        if (updateInterval(constraint, -numeric_limits<double>::infinity(), 0)) {
            addChanged(constraint);
            return true;
        }
    }

    return false;
}

int UpwardPropagator::visit(autodiff::Max* max)
{
    if (updateInterval(max, std::min(max->getLeft()->getMin(), max->getRight()->getMin()), std::max(max->getLeft()->getMax(), max->getRight()->getMax()))) {
        addChanged(max);
        return true;
    }
    return false;
}

int UpwardPropagator::visit(autodiff::Min* min)
{
    if (updateInterval(min, std::min(min->getLeft()->getMin(), min->getRight()->getMin()), std::max(min->getLeft()->getMax(), min->getRight()->getMax()))) {
        addChanged(min);
        return true;
    }
    return false;
}

int UpwardPropagator::visit(autodiff::Or* or_)
{
    if (or_->getLeft()->getMin() > 0 || or_->getRight()->getMin() > 0) {
        if (updateInterval(or_, 1, 1)) {
            addChanged(or_);
            return 1;
        }
    } else if (or_->getLeft()->getMax() <= 0 && or_->getRight()->getMax() <= 0) {
        if (updateInterval(or_, -numeric_limits<double>::infinity(), 0)) {
            addChanged(or_);
            return 1;
        }
    }
    return 0;
}

int UpwardPropagator::visit(autodiff::Product* product)
{
    double aa = product->getLeft()->getMin() * product->getRight()->getMin();
    double bb = product->getLeft()->getMax() * product->getRight()->getMax();
    double max;
    double min;
    if (product->getLeft() == product->getRight()) {
        min = std::min(aa, bb);
        max = std::max(aa, bb);
        if (product->getLeft()->getMin() * product->getLeft()->getMax() <= 0) {
            min = 0;
        }
    } else {
        double ab = product->getLeft()->getMin() * product->getRight()->getMax();
        double ba = product->getLeft()->getMax() * product->getRight()->getMin();
        max = std::max(aa, std::max(ab, std::max(ba, bb)));
        min = std::min(aa, std::min(ab, std::min(ba, bb)));
    }
    if (updateInterval(product, min, max)) {
        addChanged(product);
        return 1;
    }
    return 0;
}

int UpwardPropagator::visit(autodiff::Reification* dis)
{
    throw "Reification propagation not implemented";
    return 0;
}

int UpwardPropagator::visit(autodiff::Sigmoid* sigmoid)
{
    throw "Sigmoidal propagation not implemented";
    return 0;
}

int UpwardPropagator::visit(autodiff::Sin* sin)
{
    double size = sin->getArg()->getMax() - sin->getArg()->getMin();
    bool c = false;
    if (size <= 2 * M_PI) {
        double a = std::sin(sin->getArg()->getMax());
        double b = std::sin(sin->getArg()->getMin());
        double halfPI = M_PI / 2;
        double x = ceil((sin->getArg()->getMin() - halfPI) / M_PI);
        double y = floor((sin->getArg()->getMax() - halfPI) / M_PI);
        if (x == y) {                // single extrema
            if (((int)x) % 2 == 0) { // maxima
                c = updateInterval(sin, std::min(a, b), 1);
            } else { // minima
                c = updateInterval(sin, -1, std::max(a, b));
            }
        } else if (x > y) { // no extrema
            c = updateInterval(sin, std::min(a, b), std::max(a, b));
        } // multiple extrema, don't update
    }
    if (c)
        addChanged(sin);
    return c;
}

int UpwardPropagator::visit(autodiff::Sum* sum)
{
    double min = sum->getLeft()->getMin() + sum->getRight()->getMin();
    double max = sum->getLeft()->getMax() + sum->getRight()->getMax();

    if (updateInterval(sum, min, max)) {
        addChanged(sum);
        return true;
    }
    return false;
}

int UpwardPropagator::visit(autodiff::TermPower* power)
{
    throw "Propagation for TemPower not implemented";
}

int UpwardPropagator::visit(autodiff::Variable* var)
{
    return true;
}

void UpwardPropagator::addChanged(autodiff::TermPtr t)
{
    for (autodiff::Term* s : t->getParents()) {
        _changed->enqueueUnique(s);
    }
    _changed->enqueueUnique(t);
}

void UpwardPropagator::outputChange(autodiff::TermPtr t, Interval<double> old) const
{
    double oldwidth = old.size();
    double newwidth = t->getLocalRange().size();
    if (dynamic_cast<autodiff::Variable*>(t.get()) != nullptr) {
        std::cout << "UW shrinking " << old << " to " << t->getLocalRange() << " by " << (oldwidth - newwidth) << " ("
                  << ((oldwidth - newwidth) / oldwidth * 100) << "%)" << std::endl;
    }
}

bool UpwardPropagator::updateInterval(autodiff::TermPtr t, Interval<double> limit) const
{
    if (std::isnan(limit.getMax())) {
        limit.setMax(std::numeric_limits<double>::max());
    }
    if (std::isnan(limit.getMin())) {
        limit.setMin(std::numeric_limits<double>::min());
    }

    bool changes = !limit.contains(t->getLocalRange());
    if (changes) {
#ifdef DEBUG_DP
        Interval<double> old = t->getLocalRange();
#endif
        t->editLocalRange().intersect(limit);
        ++IntervalPropagator::updates;
#ifdef DEBUG_DP
        OutputChange(t, old);
#endif
    }
    ++IntervalPropagator::visits;
    if (!t->getLocalRange().isValid()) {
        throw 1; // not solvable
    }
    return changes;
}

} /* namespace intervalpropagation */
} /* namespace reasoner */
} /* namespace alica */
