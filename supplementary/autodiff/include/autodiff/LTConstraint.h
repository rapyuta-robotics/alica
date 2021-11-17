#pragma once
#include "BinaryFunction.h"

namespace autodiff
{
class LTEConstraint;

class LTConstraint : public BinaryFunction
{
  public:
    int accept(ITermVisitor* visitor) override;
    void acceptRecursive(ITermVisitor* visitor) override;

    TermPtr aggregateConstants() override;
    TermPtr derivative(VarPtr v) const override;
    TermPtr negate() const override;

    std::string toString() const override;
    void setNegation(const LTEConstraint* negation) const { _negatedForm = negation; }

    static void Eval(const Tape& tape, const Parameter* params, double* result, const double* vars, int dim);

    virtual EvalFunction getEvalFunction() const override { return &Eval; }

  private:
    friend TermHolder;
    LTConstraint(TermPtr x, TermPtr y, TermHolder* owner);

    mutable const LTEConstraint* _negatedForm;
};

} /* namespace autodiff */
