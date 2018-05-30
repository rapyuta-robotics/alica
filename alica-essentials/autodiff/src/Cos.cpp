/*
 * Cos.cpp
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#include "Cos.h"

#include "Constant.h"
#include "Sin.h"
#include "TermHolder.h"

#include <cmath>
#include <sstream>

namespace autodiff
{
Cos::Cos(TermPtr arg, TermHolder* owner)
    : UnaryFunction(arg, owner)
{
}

int Cos::accept(ITermVisitor* visitor)
{
    return visitor->visit(this);
}

void Cos::acceptRecursive(ITermVisitor* visitor)
{
    _arg->acceptRecursive(visitor);
    visitor->visit(this);
}

TermPtr Cos::aggregateConstants()
{
    _arg = _arg->aggregateConstants();
    if (_arg->isConstant()) {
        return _owner->constant(cos(static_cast<Constant*>(_arg)->getValue()));
    }
    return this;
}

TermPtr Cos::derivative(VarPtr v) const
{
    return _owner->sin(_arg) * -_arg->derivative(v);
}

std::string Cos::toString() const
{
    std::stringstream str;
    str << "cos( " << _arg->toString() << " )";
    return str.str();
}

void Cos::Eval(const Tape& tape, const Parameter* params, double* result, const double* vars, int dim)
{
    const double* arg = tape.getValues(params[0].asIdx);
    result[0] = cos(arg[0]);
    const double s = -sin(arg[0]);
    for (int i = 1; i <= dim; ++i) {
        result[i] = arg[i] * s;
    }
}

} /* namespace autodiff */
