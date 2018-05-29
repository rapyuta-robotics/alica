
#include "intervals/DownwardPropagator.h"
//#define DEBUG_DP
//#define DownwardPropagator_Call_Debug

#include "intervals/IntervalPropagator.h"
#include "intervals/UnsolveableException.h"
#include "intervals/UpwardPropagator.h"

#include <autodiff/AutoDiff.h>

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

using namespace autodiff;

DownwardPropagator::DownwardPropagator() {}

int DownwardPropagator::visit(Abs* abs)
{
#ifdef DownwardPropagator_Call_Debug
    std::cout << "DownwardPropagator::visit(Abs* abs)" << std::endl;
#endif
    double max = abs->getMax();
    if (updateInterval(abs->getArg(), Interval<double>(-max, max))) {
        addChanged(abs->getArg());
        return true;
    }
    return false;
}

int DownwardPropagator::visit(And* and_)
{
#ifdef DownwardPropagator_Call_Debug
    std::cout << "DownwardPropagator::visit(And* and_)" << std::endl;
#endif
    bool changed = false;
    if (and_->getMin() > 0) {
        if (updateInterval(and_->getLeft(), Interval<double>(1, 1))) {
            addChanged(and_->getLeft());
            changed = true;
        }
        if (updateInterval(and_->getRight(), Interval<double>(1, 1))) {
            addChanged(and_->getRight());
            changed = true;
        }
    }
    return changed;
}

int DownwardPropagator::visit(Atan2* /*atan2*/)
{
    throw "Atan2 propagation not implemented";
}

int DownwardPropagator::visit(Constant* /*constant*/)
{
#ifdef DownwardPropagator_Call_Debug
    std::cout << "DownwardPropagator::visit(Constant* constant)" << std::endl;
#endif
    return false;
}

int DownwardPropagator::visit(ConstPower* intPower)
{
#ifdef DownwardPropagator_Call_Debug
    std::cout << "DownwardPropagator::visit(ConstPower* intPower)" << std::endl;
#endif
    if (intPower->getMax() == std::numeric_limits<double>::infinity() || intPower->getMin() == -std::numeric_limits<double>::infinity())
        return false;
    double a = pow(intPower->getMin(), 1 / intPower->getExponent());
    double b = pow(intPower->getMax(), 1 / intPower->getExponent());

    bool isRational = intPower->getExponent() != round(intPower->getExponent());
    if (isRational) {
        if (updateInterval(intPower->getBase(), std::max(0.0, std::min(a, b)), std::max(a, std::max(-a, std::max(b, -b))))) {
            // Console.WriteLine("From DW intpower (ir) {0}",intPower);
            addChanged(intPower->getBase());
            return true;
        }
    } else {
        double min;
        double max;
        if (intPower->getExponent() >= 0) {
            if (intPower->getBase()->getMax() <= 0) {
                max = std::max(-abs(a), -abs(b));
            } else
                max = std::max(a, std::max(-a, std::max(b, -b)));
            if (intPower->getBase()->getMin() >= 0) {
                min = std::min(abs(a), abs(b));
            } else
                min = std::min(a, std::min(-a, std::min(b, -b)));
        } else { // this case can be improved
            max = std::max(a, std::max(-a, std::max(b, -b)));
            min = std::min(a, std::min(-a, std::min(b, -b)));
        }
        if (updateInterval(intPower->getBase(), min, max)) {
            // Console.WriteLine("From DW intpower {0} [{1} : {2}]",intPower,intPower->getMin(),intPower->getMax());
            addChanged(intPower->getBase());
            return true;
        }
    }
    return false;
}

int DownwardPropagator::visit(ConstraintUtility* cu)
{
#ifdef DownwardPropagator_Call_Debug
    std::cout << "DownwardPropagator::visit(ConstraintUtility* cu)" << std::endl;
#endif
    bool c = false;
    if (cu->getMin() >= 1) {
        if (updateInterval(cu->getLeft(), 1, 1)) {
            addChanged(cu->getLeft());
            c = true;
        }
        if (updateInterval(cu->getRight(), 1, cu->getMax())) {
            addChanged(cu->getRight());
            c = true;
        }
    }
    return c;
}

int DownwardPropagator::visit(Cos* cos)
{
#ifdef DownwardPropagator_Call_Debug
    std::cout << "DownwardPropagator::visit(Cos* cos)" << std::endl;
#endif
    if (cos->getMin() == -1.0 && cos->getMax() == 1.0) {
        return false;
    }
    double argMin = cos->getArg()->getMin();
    double argMax = cos->getArg()->getMax();
    double cdist = argMax - argMin;
    if (cdist >= M_PI) {
        return false;
    }

    double a = acos(cos->getMin());
    double b = acos(cos->getMax()); // 0..pi
    if (a > b) {
        std::swap(a, b);
    } // now a<= b;

    double c = -b;
    double d = -a;

    double n1 = ceil((argMin - a) / (2 * M_PI));
    double n2 = floor((argMax - b) / (2 * M_PI));
    double n3 = ceil((argMin - c) / (2 * M_PI));
    double n4 = floor((argMax - d) / (2 * M_PI));

    double min = std::numeric_limits<double>::max();
    double max = std::numeric_limits<double>::lowest();
    double n1a = n1 * 2 * M_PI + a;
    double n2b = n2 * 2 * M_PI + b;
    bool faulty = true;
    if (n1a <= argMax && n2b >= argMin) { // interval 1 completely enclosed
        min = std::min(min, n1a);
        max = std::max(max, n2b);
        faulty = false;
    } else { // no bound is inside as adding interval is smaller than pi
    }

    double n3c = n3 * 2 * M_PI + c;
    double n4d = n4 * 2 * M_PI + d;

    if (n3c <= argMax && n4d >= argMin) { // interval 2 completely enclosed
        min = std::min(min, n3c);
        max = std::max(max, n4d);
        faulty = false;
    } else { // no bound is inside as adding interval is smaller than pi
    }
    if (faulty) {
        throw 1;
        // return false;//return updateInterval(cos->getArg(),cos->getArg()->getMax(),cos->getArg()->getMin()); //no solution possible
    }

    if (min == std::numeric_limits<double>::max())
        min = std::numeric_limits<double>::lowest();
    if (max == std::numeric_limits<double>::lowest())
        max = std::numeric_limits<double>::max();
    if (updateInterval(cos->getArg(), min, max)) {
        addChanged(cos->getArg());
        return true;
    }
    return false;
}

int DownwardPropagator::visit(Exp* exp)
{
#ifdef DownwardPropagator_Call_Debug
    std::cout << "DownwardPropagator::visit(Exp* exp)" << std::endl;
#endif
    double a = log(exp->getMin());
    double b = log(exp->getMax());
    if (updateInterval(exp->getArg(), a, b)) {
        addChanged(exp->getArg());
        return true;
    }
    return false;
}

int DownwardPropagator::visit(LinSigmoid* sigmoid)
{
    throw "Sigmoidal propagation not implemented";
}

int DownwardPropagator::visit(Log* log)
{
#ifdef DownwardPropagator_Call_Debug
    std::cout << "DownwardPropagator::visit(Log* log)" << std::endl;
#endif
    double a = exp(log->getMin());
    double b = exp(log->getMax());
    if (updateInterval(log->getArg(), a, b)) {
        addChanged(log->getArg());
        return true;
    }
    return false;
}

int DownwardPropagator::visit(LTConstraint* constraint)
{
#ifdef DownwardPropagator_Call_Debug
    std::cout << "DownwardPropagator::visit(LTConstraint* constraint)" << std::endl;
#endif
    bool changed = false;
    if (constraint->getMin() > 0) {
        if (updateInterval(constraint->getRight(), constraint->getLeft()->getMin(), std::numeric_limits<double>::infinity())) {
            addChanged(constraint->getRight());
            changed = true;
        }
        if (updateInterval(constraint->getLeft(), -std::numeric_limits<double>::infinity(), constraint->getRight()->getMax())) {
            addChanged(constraint->getLeft());
            changed = true;
        }
    } else if (constraint->getMax() <= 0) {
        if (updateInterval(constraint->getRight(), -std::numeric_limits<double>::infinity(), constraint->getLeft()->getMax())) {
            addChanged(constraint->getRight());
            changed = true;
        }
        if (updateInterval(constraint->getLeft(), constraint->getRight()->getMin(), std::numeric_limits<double>::infinity())) {
            addChanged(constraint->getLeft());
            changed = true;
        }
    }
    return changed;
}

int DownwardPropagator::visit(LTEConstraint* constraint)
{
#ifdef DownwardPropagator_Call_Debug
    std::cout << "DownwardPropagator::visit(LTEConstraint* constraint)" << std::endl;
#endif
    bool changed = false;
    if (constraint->getMin() > 0) {
        if (updateInterval(constraint->getRight(), constraint->getLeft()->getMin(), std::numeric_limits<double>::infinity())) {
            addChanged(constraint->getRight());
            changed = true;
        }
        if (updateInterval(constraint->getLeft(), -std::numeric_limits<double>::infinity(), constraint->getRight()->getMax())) {
            addChanged(constraint->getLeft());
            changed = true;
        }
    } else if (constraint->getMax() <= 0) {
        if (updateInterval(constraint->getRight(), -std::numeric_limits<double>::infinity(), constraint->getLeft()->getMax())) {
            addChanged(constraint->getRight());
            changed = true;
        }
        if (updateInterval(constraint->getLeft(), constraint->getRight()->getMin(), std::numeric_limits<double>::infinity())) {
            addChanged(constraint->getLeft());
            changed = true;
        }
    }
    return changed;
}

int DownwardPropagator::visit(Max* max)
{
#ifdef DownwardPropagator_Call_Debug
    std::cout << "DownwardPropagator::visit(Max* max)" << std::endl;
#endif
    if (max->getMin() > 0) {
        bool c = false;
        if (max->getLeft()->getMax() <= 0) {
            bool c1 = updateInterval(max->getRight(), 1, 1);
            if (c1)
                addChanged(max->getRight());
            c |= c1;
        }
        if (max->getRight()->getMax() <= 0) {
            bool c2 = updateInterval(max->getLeft(), 1, 1);
            if (c2)
                addChanged(max->getLeft());
            c |= c2;
        }
        return c;
    }
    // bool c3 = updateInterval(max->getLeft(),numeric_limits<double>::lowest(),max->getMax());
    // bool c4 = updateInterval(max->getRight(),numeric_limits<double>::lowest(),max->getMax());
    return false;
}

int DownwardPropagator::visit(Min* min)
{
#ifdef DownwardPropagator_Call_Debug
    std::cout << "DownwardPropagator::visit(Min> min)" << std::endl;
#endif
    bool c1 = updateInterval(min->getLeft(), min->getMin(), std::numeric_limits<double>::infinity());
    bool c2 = updateInterval(min->getRight(), min->getMin(), std::numeric_limits<double>::infinity());
    if (c1)
        addChanged(min->getLeft());
    if (c2)
        addChanged(min->getRight());
    return c1 || c2;
}

int DownwardPropagator::visit(Or* or_)
{
    throw "Or operator progation not implemented (max is used)";
}

int DownwardPropagator::visit(Product* product)
{
#ifdef DownwardPropagator_Call_Debug
    std::cout << "DownwardPropagator::visit(Product* product)" << std::endl;
#endif
    /*
     * a*b = c
     * ==> a = c/b
     * */
    if (product->getLeft() == product->getRight()) {
        double a = sqrt(product->getMin());
        double b = sqrt(product->getMax());
        double min;
        double max;
        if (product->getLeft()->getMax() <= 0) {
            max = std::max(-a, -b);
        } else
            max = std::max(a, b);
        if (product->getLeft()->getMin() >= 0) {
            min = std::min(a, b);
        } else
            min = std::min(-a, -b);
        if (updateInterval(product->getLeft(), min, max)) {
            addChanged(product->getLeft());
            return true;
        }

    } else {
        bool c = false, d = false;
        if (product->getRight()->getMin() * product->getRight()->getMax() > 0) {
            // Left:
            double aa = product->getMin() / product->getRight()->getMin();
            double ab = product->getMin() / product->getRight()->getMax();
            double ba = product->getMax() / product->getRight()->getMin();
            double bb = product->getMax() / product->getRight()->getMax();

            double min = std::min(aa, std::min(ab, std::min(ba, bb)));
            double max = std::max(aa, std::max(ab, std::max(ba, bb)));

            c = updateInterval(product->getLeft(), min, max);
            if (c)
                addChanged(product->getLeft());
        }
        if (product->getLeft()->getMin() * product->getLeft()->getMax() > 0) {
            // Right:
            double aa = product->getMin() / product->getLeft()->getMin();
            double ab = product->getMin() / product->getLeft()->getMax();
            double ba = product->getMax() / product->getLeft()->getMin();
            double bb = product->getMax() / product->getLeft()->getMax();

            double min = std::min(aa, std::min(ab, std::min(ba, bb)));
            double max = std::max(aa, std::max(ab, std::max(ba, bb)));

            d = updateInterval(product->getRight(), min, max);
            if (d)
                addChanged(product->getRight());
        }
        return c || d;
    }
    return false;
}

int DownwardPropagator::visit(Reification* reif)
{
#ifdef DownwardPropagator_Call_Debug
    std::cout << "DownwardPropagator::visit(Reification* reif)" << std::endl;
#endif
    bool c = false;
    if (reif->getMax() < reif->getMax()) {
        c = updateInterval(reif, reif->getMin(), reif->getMin());
        if (c)
            addChanged(reif);
        if (updateInterval(reif->getLeft(), -std::numeric_limits<double>::infinity(), 0)) {
            addChanged(reif->getLeft());
            c = true;
        }
    } else if (reif->getMin() > reif->getMin()) {
        c = updateInterval(reif, reif->getMax(), reif->getMax());
        if (c)
            addChanged(reif);
        if (updateInterval(reif->getLeft(), 1, 1)) {
            addChanged(reif->getLeft());
            c = true;
        }
    }

    return c;
}

int DownwardPropagator::visit(Sigmoid* sigmoid)
{
    throw "Sigmoidal propagation not implemented";
}

int DownwardPropagator::visit(Sin* sin)
{
#ifdef DownwardPropagator_Call_Debug
    std::cout << "DownwardPropagator::visit(Sin* sin)" << std::endl;
#endif
    if (sin->getMin() == -1.0 && sin->getMax() == 1.0)
        return false;
    double cdist = sin->getArg()->getMax() - sin->getArg()->getMin();
    if (cdist >= M_PI)
        return false;
    // Console.WriteLine("Sine Prop Sine interval: [{0}, {1}]",sin->getMin(),sin->getMax());
    // Console.WriteLine("Arg interval: [{0}, {1}]",sin->getArg()->getMin(),sin->getArg()->getMax());
    double a = asin(sin->getMin());
    double b = asin(sin->getMax()); //-pi/2..pi/2
    double t;
    if (a > b) {
        t = b;
        b = a;
        a = t;
    } // now a<= b;

    double c = M_PI - b;
    double d = M_PI - a;

    double n1 = ceil((sin->getArg()->getMin() - a) / (2 * M_PI));
    // double n1a = floor((sin->getArg()->getMax() - a) / (2*M_PI));
    double n2 = floor((sin->getArg()->getMax() - b) / (2 * M_PI));
    // double n2a = ceil((sin->getArg()->getMin() - b)   /   (2*M_PI));
    double n3 = ceil((sin->getArg()->getMin() - c) / (2 * M_PI));
    // double n3a = floor((sin->getArg()->getMax() - c) / (2*M_PI));
    double n4 = floor((sin->getArg()->getMax() - d) / (2 * M_PI));
    // double n4a = ceil((sin->getArg()->getMin() - d)   /   (2*M_PI));
    // Console.WriteLine("N: {0} {1} {2} {3}",n1,n2,n3,n4);
    // Console.WriteLine("P: {0} {1} {2} {3}",n1*2*M_PI+a,n2*2*M_PI+b,n3*2*M_PI+c,n4*2*M_PI+d);
    double min = std::numeric_limits<double>::max();
    double max = std::numeric_limits<double>::lowest();
    double n1a = n1 * 2 * M_PI + a;
    double n2b = n2 * 2 * M_PI + b;
    bool faulty = true;
    if (n1a <= sin->getArg()->getMax() && n2b >= sin->getArg()->getMin()) { // interval 1 completely enclosed
        min = std::min(min, n1a);
        max = std::max(max, n2b);
        faulty = false;
    } else { // no bound is inside as adding interval is smaller than pi
    }

    double n3c = n3 * 2 * M_PI + c;
    double n4d = n4 * 2 * M_PI + d;

    if (n3c <= sin->getArg()->getMax() && n4d >= sin->getArg()->getMin()) { // interval 2 completely enclosed
        min = std::min(min, n3c);
        max = std::max(max, n4d);
        faulty = false;
    } else { // no bound is inside as adding interval is smaller than pi
    }

    if (faulty) {
        throw 1;
        // return false; //updateInterval(sin->getArg(),sin->getArg()->getMax(),sin->getArg()->getMin()); //no solution possible
    }
    /*if (n1 == n2) { //bound within interval
     min = std::min(min,n1*2*M_PI+a);
     max = std::max(max,n2*2*M_PI+b);
     } else {
     if (n1 > n2) { //lower bound cut

     min = std::min(min,sin->getArg()->getMin());
     max = std::max(max,n2*2*M_PI+b);

     double k =

     }
     }

     //if (n1 == n2 && n3 == n4) { //bind to rectangle:
     double min = std::min(n1*2*M_PI+a,n3*2*M_PI+c);
     double max = std::max(n2*2*M_PI+b,n4*2*M_PI+d);
     */
    //}
    if (min == std::numeric_limits<double>::max()) {
        min = std::numeric_limits<double>::lowest();
    }
    if (max == std::numeric_limits<double>::min()) {
        max = std::numeric_limits<double>::max();
    }
    if (updateInterval(sin->getArg(), min, max)) {
        addChanged(sin->getArg());
        return true;
    }
    return false;
}

int DownwardPropagator::visit(Sum* sum)
{
#ifdef DownwardPropagator_Call_Debug
    std::cout << "DownwardPropagator::visit(Sum* sum)" << std::endl;
#endif
    // a+b= c
    // a= b-c
    // a:
    bool changed = false;

    double minRight = sum->getRight()->getMin();
    double maxRight = sum->getRight()->getMax();

    double minLeft = sum->getLeft()->getMin();
    double maxLeft = sum->getLeft()->getMax();

    if (updateInterval(sum->getLeft(), sum->getMin() - maxRight, sum->getMax() - minRight)) {
        addChanged(sum->getLeft());
        changed = true;
    }
    if (updateInterval(sum->getRight(), sum->getMin() - maxLeft, sum->getMax() - minLeft)) {
        addChanged(sum->getRight());
        changed = true;
    }

    return changed;
}

int DownwardPropagator::visit(TermPower* power)
{
    throw "Propagation for TemPower not implemented";
    return false;
}

int DownwardPropagator::visit(Variable* var)
{
#ifdef DownwardPropagator_Call_Debug
    std::cout << "DownwardPropagator::visit(Variable* var)" << std::endl;
#endif
    return false;
}

void DownwardPropagator::addChanged(TermPtr t)
{
    _changed->enqueueUnique(t);
}

void DownwardPropagator::outputChange(TermPtr t, Interval<double> old) const
{
    double oldwidth = old.size();
    double newwidth = t->getLocalRange().size();
    if (dynamic_cast<Variable*>(t.get()) != nullptr) {
        std::cout << "DW shrinking " << old << " to " << t->getLocalRange() << " by " << (oldwidth - newwidth) << " ("
                  << ((oldwidth - newwidth) / oldwidth * 100) << "%)" << std::endl;
    }
}

bool DownwardPropagator::updateInterval(TermPtr t, Interval<double> limit) const
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
