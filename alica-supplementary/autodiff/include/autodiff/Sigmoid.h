#pragma once

#include "Term.h"

namespace autodiff
{

class Sigmoid : public Term
{
  public:
    int accept(ITermVisitor* visitor) override;
    void acceptRecursive(ITermVisitor* visitor) override;

    TermPtr aggregateConstants() override;
    TermPtr derivative(VarPtr v) const override;

    std::string toString() const override;

    static void Eval(const Tape& tape, const Parameter* params, double* result, const double* vars, int dim);

    virtual EvalFunction getEvalFunction() const override { return &Eval; }

    virtual void fillParameters(Parameter* params) const override
    {
        params[0].asIdx = _arg->getTapeIdx();
        params[1].asDouble = _steepness;
    }
    TermPtr getArg() const { return _arg; }

  private:
    friend TermHolder;
    Sigmoid(TermPtr arg, TermHolder* owner);
    Sigmoid(TermPtr arg, double steepness, TermHolder* owner);

    TermPtr _arg;
    double _steepness;
};

} /* namespace autodiff */
