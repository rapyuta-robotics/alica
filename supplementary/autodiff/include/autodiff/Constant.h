#pragma once

#include "Term.h"

namespace autodiff
{

class Constant : public Term
{
  public:
    int accept(ITermVisitor* visitor) override;
    void acceptRecursive(ITermVisitor* visitor) override;

    TermPtr aggregateConstants() override;
    TermPtr derivative(VarPtr v) const override;

    std::string toString() const override;
    double getValue() const { return _value; }

    virtual bool isConstant() const override { return true; }

    static void Eval(const Tape& tape, const Parameter* params, double* result, const double* vars, int dim);

    virtual EvalFunction getEvalFunction() const override { return &Eval; }
    virtual void fillParameters(Parameter* params) const override { params[0].asDouble = _value; }

  private:
    friend TermHolder;
    Constant(double value, TermHolder* owner);
    double _value;
};

} /* namespace autodiff */
