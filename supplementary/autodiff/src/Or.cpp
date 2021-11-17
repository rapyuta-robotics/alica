/*
 * Or.cpp
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#include "Or.h"

#include "Constant.h"
#include "TermHolder.h"

#include <cmath>
#include <sstream>

namespace autodiff
{
Or::Or(TermPtr left, TermPtr right, TermHolder* owner)
    : BinaryFunction(left, right, owner)
{
}

int Or::accept(ITermVisitor* visitor)
{
    return visitor->visit(this);
}

void Or::acceptRecursive(ITermVisitor* visitor)
{
    _left->acceptRecursive(visitor);
    _right->acceptRecursive(visitor);
    visitor->visit(this);
}

TermPtr Or::aggregateConstants()
{
    _left = _left->aggregateConstants();
    _right = _right->aggregateConstants();
    if (_left->isConstant() && _right->isConstant()) {
        if (static_cast<Constant*>(_left)->getValue() > 0.75 || static_cast<Constant*>(_right)->getValue() > 0.75) {
            return _owner->trueConstant();
        } else {
            return _owner->falseConstant();
        }
    } else {
        return this;
    }
}

TermPtr Or::derivative(VarPtr v) const
{
    throw "Symbolic Derivation of Or not supported.";
    return nullptr;
}

TermPtr Or::negate() const
{
    return _left->negate() & _right->negate();
}

std::string Or::toString() const
{
    std::stringstream str;
    str << "or( " << _left->toString() << ", " << _right->toString() << " )";
    return str.str();
}

void Or::Eval(const Tape& tape, const Parameter* params, double* result, const double* vars, int dim)
{
    const double* l = tape.getValues(params[0].asIdx);
    const double* r = tape.getValues(params[1].asIdx);

    if (l[0] <= 0.0 && r[0] <= 0.0) {
        for (int i = 0; i <= dim; ++i) {
            result[i] = l[i] + r[i];
        }
    } else {
        result[0] = 1.0;
        for (int i = 1; i <= dim; ++i) {
            result[i] = 0.0;
        }
    }
}

} /* namespace autodiff */
