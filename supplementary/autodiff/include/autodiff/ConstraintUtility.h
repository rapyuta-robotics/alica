#pragma once

#include "BinaryFunction.h"

namespace autodiff
{

class ConstraintUtility : public BinaryFunction
{
  public:
    int accept(ITermVisitor* visitor) override;
    void acceptRecursive(ITermVisitor* visitor) override;

    TermPtr aggregateConstants() override;
    TermPtr derivative(VarPtr v) const override;
    TermPtr negate() const override;

    std::string toString() const override;

    static void Eval(const Tape& tape, const Parameter* params, double* result, const double* vars, int dim);

    virtual EvalFunction getEvalFunction() const override { return &Eval; }

    TermPtr getConstraint() const { return _left; }
    TermPtr getUtility() const { return _right; }

  private:
    friend TermHolder;
    ConstraintUtility(TermPtr constraint, TermPtr utility, TermHolder* owner);
};

} /* namespace autodiff */
