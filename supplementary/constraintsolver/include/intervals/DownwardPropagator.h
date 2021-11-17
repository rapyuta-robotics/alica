#pragma once

#include <alica_solver_interface/Interval.h>
#include <autodiff/ITermVisitor.h>
#include <autodiff/TermPtr.h>
#include <autodiff/Types.h>

namespace alica
{
namespace reasoner
{
namespace intervalpropagation
{
class UpwardPropagator;

class DownwardPropagator : public autodiff::ITermVisitor
{
  public:
    DownwardPropagator();

    int visit(autodiff::Abs* abs) override;
    int visit(autodiff::And* and_) override;
    int visit(autodiff::Atan2* atan2) override;
    int visit(autodiff::Constant* constant) override;
    int visit(autodiff::ConstPower* intPower) override;
    int visit(autodiff::ConstraintUtility* cu) override;
    int visit(autodiff::Cos* cos) override;
    int visit(autodiff::Exp* exp) override;
    int visit(autodiff::LinSigmoid* sigmoid) override;
    int visit(autodiff::Log* log) override;
    int visit(autodiff::LTConstraint* constraint) override;
    int visit(autodiff::LTEConstraint* constraint) override;
    int visit(autodiff::Max* max) override;
    int visit(autodiff::Min* min) override;
    int visit(autodiff::Or* or_) override;
    int visit(autodiff::Product* product) override;
    int visit(autodiff::Reification* reif) override;
    int visit(autodiff::Sigmoid* sigmoid) override;
    int visit(autodiff::Sin* sin) override;
    int visit(autodiff::Sum* sum) override;
    int visit(autodiff::TermPower* power) override;
    int visit(autodiff::Variable* var) override;
    void setTermQueue(autodiff::TermList* q) { _changed = q; }

  private:
    void addChanged(autodiff::TermPtr t);
    void outputChange(autodiff::TermPtr t, Interval<double> old) const;
    bool updateInterval(autodiff::TermPtr t, Interval<double> limit) const;
    bool updateInterval(autodiff::TermPtr t, double min, double max) const { return updateInterval(t, Interval<double>(min, max)); }

    autodiff::TermList* _changed;
    UpwardPropagator* _up;
};

} /* namespace intervalpropagation */
} /* namespace reasoner */
} /* namespace alica */
