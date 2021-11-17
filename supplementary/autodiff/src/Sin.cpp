
#include "Sin.h"

#include "Constant.h"
#include "Cos.h"
#include "TermHolder.h"

#include <cmath>
#include <sstream>

namespace autodiff
{
Sin::Sin(TermPtr arg, TermHolder* owner)
    : UnaryFunction(arg, owner)
{
}

int Sin::accept(ITermVisitor* visitor)
{
    return visitor->visit(this);
}

void Sin::acceptRecursive(ITermVisitor* visitor)
{
    _arg->acceptRecursive(visitor);
    visitor->visit(this);
}

TermPtr Sin::aggregateConstants()
{
    _arg = _arg->aggregateConstants();
    if (_arg->isConstant()) {
        return _owner->constant(sin(static_cast<Constant*>(_arg)->getValue()));
    }
    return this;
}

TermPtr Sin::derivative(VarPtr v) const
{
    return _owner->cos(_arg) * _arg->derivative(v);
}

std::string Sin::toString() const
{
    std::stringstream str;
    str << "sin( " << _arg->toString() << " )";
    return str.str();
}

void Sin::Eval(const Tape& tape, const Parameter* params, double* result, const double* vars, int dim)
{
    const double* arg = tape.getValues(params[0].asIdx);
    result[0] = sin(arg[0]);
    const double s = cos(arg[0]);
    for (int i = 1; i <= dim; ++i) {
        result[i] = arg[i] * s;
    }
}

} /* namespace autodiff */
