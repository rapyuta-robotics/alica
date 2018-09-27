/*
 * TermPower.cpp
 *
 *  Created on: Jul 18, 2014
 *      Author: psp
 */

#include "TermPower.h"

#include "ConstPower.h"
#include "Constant.h"
#include "Log.h"
#include "TermHolder.h"

#include <cmath>
#include <sstream>

namespace autodiff
{
TermPower::TermPower(TermPtr baseTerm, TermPtr exponent, TermHolder* owner)
    : BinaryFunction(baseTerm, exponent, owner)
{
}

int TermPower::accept(ITermVisitor* visitor)
{
    return visitor->visit(this);
}

void TermPower::acceptRecursive(ITermVisitor* visitor)
{
    _left->acceptRecursive(visitor);
    _right->acceptRecursive(visitor);
    visitor->visit(this);
}

TermPtr TermPower::aggregateConstants()
{
    _left = _left->aggregateConstants();
    _right = _right->aggregateConstants();
    TermPower* basetp = dynamic_cast<TermPower*>(_left.get());

    if (_left->isConstant() && _right->isConstant()) {
        return _owner->constant(pow(static_cast<Constant*>(_left)->getValue(), static_cast<Constant*>(_right)->getValue()));
    } else if (_right->isConstant()) {
        return _owner->constPower(_left, static_cast<Constant*>(_right)->getValue());
    } else if (basetp != nullptr) {
        _right = _right * basetp->_right;
        _left = basetp->_left;
    }
    return this;
}

TermPtr TermPower::derivative(VarPtr v) const
{
    return _owner->termPower(_left, _right - 1) * (_right * _left->derivative(v) + _left * _owner->log(_left) * _right->derivative(v));
}

std::string TermPower::toString() const
{
    std::stringstream str;
    str << "termPower( " << _left->toString() << ", " << _right->toString() << " )";
    return str.str();
}

void TermPower::Eval(const Tape& tape, const Parameter* params, double* result, const double* vars, int dim)
{
    const double* l = tape.getValues(params[0].asIdx);
    const double* r = tape.getValues(params[1].asIdx);
    const double outer = pow(l[0], r[0] - 1.0);
    result[0] = outer * l[0];

    const double lval = outer * r[0];
    const double rval = result[0] * log(l[0]);

    for (int i = 1; i <= dim; ++i) {
        result[i] = lval * l[i] + rval * r[i];
    }
}

} /* namespace autodiff */
