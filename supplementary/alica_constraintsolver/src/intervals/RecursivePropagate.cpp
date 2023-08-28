/*
 * RecursivePropagate.cpp
 *
 *  Created on: Dec 11, 2014
 *      Author: Philipp
 */

#include "intervals/RecursivePropagate.h"

#include "intervals/DownwardPropagator.h"
#include "intervals/SetParents.h"
#include "intervals/UpwardPropagator.h"

#include "autodiff/AutoDiff.h"

//#define RecPropDEBUG
using namespace autodiff;

namespace alica
{
namespace reasoner
{
namespace intervalpropagation
{

RecursivePropagate::RecursivePropagate()
{
    _dp.setTermQueue(&_changed);
    _up.setTermQueue(&_changed);
}

RecursivePropagate::~RecursivePropagate() {}

bool RecursivePropagate::propagate(TermPtr term)
{
    _changed.clear();
    term->acceptRecursive(this);
#ifdef RecPropDEBUG
    std::cout << "Queued Terms: " << std::endl;
    for (TermPtr asd : _changed) {
        std::cout << asd->toString() << std::endl;
    }
    std::cout << "------------------------------------" << std::endl;
#endif

    TermPtr cur = _changed.dequeue();
    while (cur != nullptr) {
        cur->accept(&_dp);
        cur->accept(&_up);
        cur = _changed.dequeue();
    }
    return true;
}

void RecursivePropagate::addToQueue(TermPtr t)
{
    _changed.enqueueUnique(t);
}

int RecursivePropagate::visit(Abs* abs)
{
    addToQueue(abs);
    abs->accept(&_sp);
    return 0;
}

int RecursivePropagate::visit(And* and_)
{
    addToQueue(and_);
    and_->accept(&_sp);
    return 0;
}

int RecursivePropagate::visit(Atan2* atan2)
{
    addToQueue(atan2);
    atan2->accept(&_sp);
    return 0;
}

int RecursivePropagate::visit(Constant* constant)
{
    constant->accept(&_sp);
    return 0;
}

int RecursivePropagate::visit(ConstPower* intPower)
{
    addToQueue(intPower);
    intPower->accept(&_sp);
    return 0;
}

int RecursivePropagate::visit(ConstraintUtility* cu)
{
    addToQueue(cu);
    cu->accept(&_sp);
    return 0;
}

int RecursivePropagate::visit(Cos* cos)
{
    addToQueue(cos);
    cos->accept(&_sp);
    return 0;
}

int RecursivePropagate::visit(Exp* exp)
{
    addToQueue(exp);
    exp->accept(&_sp);
    return 0;
}

int RecursivePropagate::visit(LinSigmoid* sigmoid)
{
    addToQueue(sigmoid);
    sigmoid->accept(&_sp);
    return 0;
}

int RecursivePropagate::visit(Log* log)
{
    addToQueue(log);
    log->accept(&_sp);
    return 0;
}

int RecursivePropagate::visit(LTConstraint* constraint)
{
    addToQueue(constraint);
    constraint->accept(&_sp);
    return 0;
}

int RecursivePropagate::visit(LTEConstraint* constraint)
{
    addToQueue(constraint);
    constraint->accept(&_sp);
    return 0;
}

int RecursivePropagate::visit(Max* max)
{
    addToQueue(max);
    max->accept(&_sp);
    return 0;
}

int RecursivePropagate::visit(Min* min)
{
    addToQueue(min);
    min->accept(&_sp);
    return 0;
}

int RecursivePropagate::visit(Or* or_)
{
    addToQueue(or_);
    or_->accept(&_sp);
    return 0;
}

int RecursivePropagate::visit(Product* product)
{
    addToQueue(product);
    product->accept(&_sp);
    return 0;
}

int RecursivePropagate::visit(Reification* reif)
{
    addToQueue(reif);
    reif->accept(&_sp);
    return 0;
}

int RecursivePropagate::visit(Sigmoid* sigmoid)
{
    addToQueue(sigmoid);
    sigmoid->accept(&_sp);
    return 0;
}

int RecursivePropagate::visit(Sin* sin)
{
    addToQueue(sin);
    sin->accept(&_sp);
    return 0;
}

int RecursivePropagate::visit(Sum* sum)
{
    addToQueue(sum);
    sum->accept(&_sp);
    return 0;
}

int RecursivePropagate::visit(TermPower* power)
{
    addToQueue(power);
    power->accept(&_sp);
    return 0;
}

int RecursivePropagate::visit(Variable* var)
{
    addToQueue(var);
    var->accept(&_sp);
    return 0;
}

} /* namespace intervalpropagation */
} /* namespace reasoner */
} /* namespace alica */
