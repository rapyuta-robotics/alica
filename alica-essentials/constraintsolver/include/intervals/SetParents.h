#pragma once

#include <autodiff/ITermVisitor.h>
#include <autodiff/Types.h>

namespace alica
{
namespace reasoner
{
namespace intervalpropagation
{

class SetParents : public autodiff::ITermVisitor
{
  public:
    SetParents();
    virtual ~SetParents();

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
    int visit(autodiff::Reification* dis) override;
    int visit(autodiff::Sigmoid* sigmoid) override;
    int visit(autodiff::Sin* sin) override;
    int visit(autodiff::Sum* sum) override;
    int visit(autodiff::TermPower* power) override;
    int visit(autodiff::Variable* var) override;
};

} /* namespace intervalpropagation */
} /* namespace reasoner */
} /* namespace alica */
