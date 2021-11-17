#pragma once

#include "Term.h"
namespace autodiff
{

class UnaryFunction : public Term
{
  public:
    virtual void fillParameters(Parameter* params) const override { params[0].asIdx = _arg->getTapeIdx(); }
    TermPtr getArg() const { return _arg; }

  protected:
    UnaryFunction(TermPtr arg, TermHolder* owner)
        : Term(owner)
        , _arg(arg)

    {
    }

    TermPtr _arg;
};
} // namespace autodiff