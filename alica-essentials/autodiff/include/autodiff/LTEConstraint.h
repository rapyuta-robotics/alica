#pragma once

#include "BinaryFunction.h"

namespace autodiff
{
class LTConstraint;
class LTEConstraint : public BinaryFunction
{
  public:
    void setNegation(const LTConstraint* negation) const { _negatedForm = negation; }

    int accept(ITermVisitor* visitor) override;
    void acceptRecursive(ITermVisitor* visitor) override;

    TermPtr aggregateConstants() override;
    TermPtr derivative(VarPtr v) const override;
    TermPtr negate() const override;

    std::string toString() const override;

    static void Eval(const Tape& tape, const Parameter* params, double* result, const double* vars, int dim);

    virtual EvalFunction getEvalFunction() const override { return &Eval; }

  private:
    friend TermHolder;
    LTEConstraint(TermPtr x, TermPtr y, TermHolder* owner);

    mutable const LTConstraint* _negatedForm;
};

} /* namespace autodiff */
