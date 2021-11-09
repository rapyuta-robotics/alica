#pragma once

#include "Term.h"

namespace autodiff
{

class ConstPower : public Term
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
        params[0].asIdx = _base->getTapeIdx();
        params[1].asDouble = _exponent;
    }
    double getExponent() const { return _exponent; }
    TermPtr getBase() const { return _base; }

  private:
    friend TermHolder;
    ConstPower(TermPtr baseTerm, double exponent, TermHolder* owner);

    TermPtr _base;
    double _exponent;
};

} /* namespace autodiff */
