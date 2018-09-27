
#include "Term.h"

#include "TermHolder.h"
#include "Variable.h"

#include <limits>
#include <typeinfo>

#include <iostream>

namespace autodiff
{

constexpr double Term::CONSTRAINTSTEEPNESS;
constexpr double Term::EPSILON;

OrType Term::_orop = OrType::MAX;
AndType Term::_andop = AndType::MIN;

Term::Term(TermHolder* owner)
    : _owner(owner)
    , _next(nullptr)
    , _localRange(Variable::minExpressibleValue, Variable::maxExpressibleValue)
    , _tapeIdx(-1)
{
}

Term::~Term() {}

TermPtr Term::negate() const
{
    return _owner->trueConstant() - TermPtr(this);
}

// some static dirt...
AndType Term::getAnd()
{
    return _andop;
}

void Term::setAnd(AndType a)
{
    _andop = a;
}

OrType Term::getOr()
{
    return _orop;
}

void Term::setOr(OrType o)
{
    _orop = o;
}

} /* namespace autodiff */
