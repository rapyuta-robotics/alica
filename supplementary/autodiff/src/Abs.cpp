
#include "Abs.h"

#include "Constant.h"
#include "Tape.h"
#include "TermHolder.h"

#include <cmath>
#include <sstream>

namespace autodiff
{

Abs::Abs(TermPtr arg, TermHolder* owner)
    : UnaryFunction(arg, owner)
{
}

int Abs::accept(ITermVisitor* visitor)
{
    return visitor->visit(this);
}
void Abs::acceptRecursive(ITermVisitor* visitor)
{
    _arg->acceptRecursive(visitor);
    visitor->visit(this);
}

TermPtr Abs::aggregateConstants()
{
    _arg = _arg->aggregateConstants();
    if (_arg->isConstant()) {
        return _owner->constant(fabs(static_cast<Constant*>(_arg)->getValue()));
    } else {
        return this;
    }
}

TermPtr Abs::derivative(VarPtr v) const
{
    return _arg->derivative(v) * _arg / this;
}

std::string Abs::toString() const
{
    std::stringstream str;
    str << "abs( " << _arg->toString() << " )";
    return str.str();
}

void Abs::Eval(const Tape& tape, const Parameter* params, double* result, const double* vars, int dim)
{
    const double* arg = tape.getValues(params[0].asIdx);
    result[0] = fabs(arg[0]);
    double sign = std::copysign(1.0, arg[0]);
    for (int i = 1; i <= dim; ++i) {
        result[i] = arg[i] * sign;
    }
}
} /* namespace autodiff */
