#pragma once

#include <autodiff/ITermVisitor.h>
#include <autodiff/TermPtr.h>
#include <autodiff/Types.h>

#include <alica_solver_interface/Interval.h>

namespace alica
{
namespace reasoner
{
namespace intervalpropagation
{
class DownwardPropagator;
class TermList;

class UpwardPropagator : public autodiff::ITermVisitor
{
  public:
    UpwardPropagator();

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

    void setTermQueue(autodiff::TermList* q) { _changed = q; }

  private:
    void addChanged(autodiff::TermPtr t);
    // TODO: unify common functions with downward propagator
    void outputChange(autodiff::TermPtr t, Interval<double> old) const;
    bool updateInterval(autodiff::TermPtr t, Interval<double> limit) const;
    bool updateInterval(autodiff::TermPtr t, double min, double max) const { return updateInterval(t, Interval<double>(min, max)); }

    autodiff::TermList* _changed;
    DownwardPropagator* _dp;
};

} /* namespace intervalpropagation */
} /* namespace reasoner */
} /* namespace alica */
