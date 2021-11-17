#include "LinSigmoid.h"

#include "Constant.h"
#include "TermHolder.h"

#include <cmath>
#include <limits>
#include <sstream>

namespace autodiff
{
LinSigmoid::LinSigmoid(TermPtr arg, TermHolder* owner)
    : UnaryFunction(arg, owner)
{
}

int LinSigmoid::accept(ITermVisitor* visitor)
{
    return visitor->visit(this);
}

void LinSigmoid::acceptRecursive(ITermVisitor* visitor)
{
    _arg->acceptRecursive(visitor);
    visitor->visit(this);
}

TermPtr LinSigmoid::aggregateConstants()
{
    _arg = _arg->aggregateConstants();
    if (_arg->isConstant()) {
        double e = exp(-static_cast<Constant*>(_arg)->getValue());
        if (e == std::numeric_limits<double>::infinity()) {
            return _owner->constant(Term::EPSILON);
        } else {
            e = 1.0 / (1.0 + e);
        }
        if (e < Term::EPSILON) {
            return _owner->constant(Term::EPSILON);
        } else {
            return _owner->constant(e);
        }
    }
    return this;
}

TermPtr LinSigmoid::derivative(VarPtr v) const
{
    // TermPtr e = _owner->exp(-arg);
    // return e / _owner->constantPower(e + 1.0, 2.0);
    return _arg->derivative(v); // The derivative is linear.
}

std::string LinSigmoid::toString() const
{
    std::stringstream str;
    str << "sigmoid( " << _arg->toString() << " )";
    return str.str();
}

void LinSigmoid::Eval(const Tape& tape, const Parameter* params, double* result, const double* vars, int dim)
{
    // The derivative is linear.
    const double* arg = tape.getValues(params[0].asIdx);
    const double e = exp(-arg[0]);
    result[0] = 1.0 / (1.0 + e);
    for (int i = 1; i <= dim; ++i) {
        result[i] = arg[i];
    }
}
} /* namespace autodiff */
