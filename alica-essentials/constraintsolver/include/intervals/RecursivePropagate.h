#pragma once

#include <autodiff/Types.h>

#include "DownwardPropagator.h"
#include "SetParents.h"
#include "UpwardPropagator.h"
#include <autodiff/ITermVisitor.h>
#include <autodiff/TermList.h>

namespace alica
{
namespace reasoner
{
namespace intervalpropagation
{

class RecursivePropagate : public autodiff::ITermVisitor
{
  public:
    RecursivePropagate();
    virtual ~RecursivePropagate();

    bool propagate(autodiff::TermPtr term);
    void addToQueue(autodiff::TermPtr t);

    int visit(autodiff::Abs* abs);
    int visit(autodiff::And* and_);
    int visit(autodiff::Atan2* atan2);
    int visit(autodiff::Constant* constant);
    int visit(autodiff::ConstPower* intPower);
    int visit(autodiff::ConstraintUtility* cu);
    int visit(autodiff::Cos* cos);
    int visit(autodiff::Exp* exp);
    int visit(autodiff::LinSigmoid* sigmoid);
    int visit(autodiff::Log* log);
    int visit(autodiff::LTConstraint* constraint);
    int visit(autodiff::LTEConstraint* constraint);
    int visit(autodiff::Max* max);
    int visit(autodiff::Min* min);
    int visit(autodiff::Or* or_);
    int visit(autodiff::Product* product);
    int visit(autodiff::Reification* dis);
    int visit(autodiff::Sigmoid* sigmoid);
    int visit(autodiff::Sin* sin);
    int visit(autodiff::Sum* sum);
    int visit(autodiff::TermPower* power);
    int visit(autodiff::Variable* var);

  protected:
    autodiff::TermList _changed;

    DownwardPropagator _dp;
    UpwardPropagator _up;
    SetParents _sp;
};

} /* namespace intervalpropagation */
} /* namespace reasoner */
} /* namespace alica */
