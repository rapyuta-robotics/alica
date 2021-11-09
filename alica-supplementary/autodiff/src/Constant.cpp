
#include "Constant.h"
#include "TermHolder.h"

#include <cmath>
#include <cstring>

namespace autodiff
{

Constant::Constant(double value, TermHolder* owner)
    : Term(owner)
    , _value(value)
{
}

int Constant::accept(ITermVisitor* visitor)
{
    return visitor->visit(this);
}

void Constant::acceptRecursive(ITermVisitor* visitor)
{
    visitor->visit(this);
}

TermPtr Constant::aggregateConstants()
{
    return this;
}

TermPtr Constant::derivative(VarPtr v) const
{
    return _owner->zeroConstant();
}

std::string Constant::toString() const
{
    if (this == _owner->trueConstant()) {
        return "true";
    } else if (this == _owner->falseConstant()) {
        return "false";
    }

    return std::to_string(_value);
}

void Constant::Eval(const Tape& tape, const Parameter* params, double* result, const double* vars, int dim)
{
    result[0] = params[0].asDouble;
}

} /* namespace autodiff */
