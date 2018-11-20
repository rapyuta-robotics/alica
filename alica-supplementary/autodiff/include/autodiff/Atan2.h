#pragma once

#include "BinaryFunction.h"

namespace autodiff
{

class Atan2 : public BinaryFunction
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
    Atan2(TermPtr left, TermPtr right, TermHolder* owner);
};

} /* namespace autodiff */
