#include "Log.h"

#include "Constant.h"
#include "TermHolder.h"

#include <cmath>
#include <limits>
#include <sstream>

namespace autodiff
{
Log::Log(TermPtr arg, TermHolder* owner)
    : UnaryFunction(arg, owner)
{
}

int Log::accept(ITermVisitor* visitor)
{
    return visitor->visit(this);
}

void Log::acceptRecursive(ITermVisitor* visitor)
{
    _arg->acceptRecursive(visitor);
    visitor->visit(this);
}

TermPtr Log::aggregateConstants()
{
    _arg = _arg->aggregateConstants();
    if (_arg->isConstant()) {
        return _owner->constant(log(static_cast<Constant*>(_arg)->getValue()));
    } else {
        return this;
    }
}

TermPtr Log::derivative(VarPtr v) const
{
    return _arg->derivative(v) / _arg;
}

std::string Log::toString() const
{
    std::stringstream str;
    str << "log( " << _arg->toString() << " )";
    return str.str();
}

void Log::Eval(const Tape& tape, const Parameter* params, double* result, const double* vars, int dim)
{
    const double* arg = tape.getValues(params[0].asIdx);
    result[0] = log(arg[0]);
    const double s = 1.0 / arg[0];
    for (int i = 1; i <= dim; ++i) {
        result[i] = arg[i] * s;
    }
}

} /* namespace autodiff */
