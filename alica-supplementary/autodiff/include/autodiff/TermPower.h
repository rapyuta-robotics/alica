#pragma once
#include "BinaryFunction.h"

namespace autodiff
{

class TermPower : public BinaryFunction
{
  public:
    int accept(ITermVisitor* visitor) override;
    void acceptRecursive(ITermVisitor* visitor) override;

    TermPtr aggregateConstants() override;
    TermPtr derivative(VarPtr v) const override;

    std::string toString() const override;

    static void Eval(const Tape& tape, const Parameter* params, double* result, const double* vars, int dim);

    virtual EvalFunction getEvalFunction() const override { return &Eval; }

  private:
    friend TermHolder;
    TermPower(TermPtr baseTerm, TermPtr exponent, TermHolder* owner);
};

} /* namespace autodiff */
