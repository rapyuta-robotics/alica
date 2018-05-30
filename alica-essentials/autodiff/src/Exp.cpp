/*
 * Exp.cpp
 *
 *  Created on: Jun 12, 2014
 *      Author: psp
 */

#include "Exp.h"

#include "Constant.h"
#include "TermHolder.h"

#include <cmath>
#include <sstream>

namespace autodiff
{

Exp::Exp(TermPtr arg, TermHolder* owner)
    : UnaryFunction(arg, owner)
{
}

int Exp::accept(ITermVisitor* visitor)
{
    return visitor->visit(this);
}

void Exp::acceptRecursive(ITermVisitor* visitor)
{
    _arg->acceptRecursive(visitor);
    visitor->visit(this);
}

TermPtr Exp::aggregateConstants()
{
    _arg = _arg->aggregateConstants();
    if (_arg->isConstant()) {
        return _owner->constant(exp(static_cast<Constant*>(_arg)->getValue()));
    }
    return this;
}

TermPtr Exp::derivative(VarPtr v) const
{
    return this * _arg->derivative(v);
}

std::string Exp::toString() const
{
    std::stringstream str;
    str << "exp( " << _arg->toString() << " )";
    return str.str();
}

void Exp::Eval(const Tape& tape, const Parameter* params, double* result, const double* vars, int dim)
{
    const double* arg = tape.getValues(params[0].asIdx);
    result[0] = exp(arg[0]);
    for (int i = 1; i <= dim; ++i) {
        result[i] = arg[i] * result[0];
    }
}

} /* namespace autodiff */
