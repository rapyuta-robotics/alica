#pragma once

#include "Term.h"
namespace autodiff
{

class BinaryFunction : public Term
{
  public:
    virtual void fillParameters(Parameter* params) const override
    {
        params[0].asIdx = _left->getTapeIdx();
        params[1].asIdx = _right->getTapeIdx();
    }
    TermPtr getLeft() const { return _left; }
    TermPtr getRight() const { return _right; }

  protected:
    BinaryFunction(TermPtr left, TermPtr right, TermHolder* owner)
        : Term(owner)
        , _left(left)
        , _right(right)
    {
    }

    TermPtr _left;
    TermPtr _right;
};
}