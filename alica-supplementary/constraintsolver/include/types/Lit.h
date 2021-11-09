#pragma once

#include "types/Assignment.h"

#include <autodiff/ITermVisitor.h>
#include <autodiff/TermPtr.h>
#include <autodiff/Types.h>

#include <memory>

namespace alica
{
namespace reasoner
{
namespace cnsat
{
class Var;

class Lit : public autodiff::ITermVisitor
{
  public:
    Lit(const std::shared_ptr<Var>& v, Assignment ass = Assignment::TRUE);
    Lit(autodiff::TermPtr t, Assignment ass, bool temp);
    virtual ~Lit();

    bool satisfied() const;
    bool conflicted() const;
    void computeVariableCount();

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

    Assignment sign;
    autodiff::TermPtr _atom;
    std::shared_ptr<Var> var;
    int variableCount;
    bool isTemporary;
};

} /* namespace cnsat */
} /* namespace reasoner */
} /* namespace alica */
