/*
 * LTConstraint.cpp
 *
 *  Created on: Jul 17, 2014
 *      Author: psp
 */

#include "LTConstraint.h"

#include "Constant.h"
#include "LTEConstraint.h"
#include "TermHolder.h"

#include <cmath>
#include <sstream>

namespace autodiff
{
LTConstraint::LTConstraint(TermPtr x, TermPtr y, TermHolder* owner)
    : BinaryFunction(x, y, owner)
    , _negatedForm(nullptr)
{
}

int LTConstraint::accept(ITermVisitor* visitor)
{
    return visitor->visit(this);
}

void LTConstraint::acceptRecursive(ITermVisitor* visitor)
{
    _left->acceptRecursive(visitor);
    _right->acceptRecursive(visitor);
    visitor->visit(this);
}

TermPtr LTConstraint::aggregateConstants()
{
    _left = _left->aggregateConstants();
    _right = _right->aggregateConstants();
    if (_left->isConstant() && _right->isConstant()) {
        if (static_cast<Constant*>(_left)->getValue() < static_cast<Constant*>(_right)->getValue()) {
            return _owner->trueConstant();
        } else {
            return _owner->falseConstant();
        }
    } else {
        return this;
    }
}

TermPtr LTConstraint::derivative(VarPtr v) const
{
    throw "Symbolic Derivation of Less-Than not supported.";
    return nullptr;
}

TermPtr LTConstraint::negate() const
{
    if (_negatedForm == nullptr) {
        _negatedForm = _owner->lessThanEqual(_right, _left);
        _negatedForm->setNegation(this);
    }
    return _negatedForm;
}

std::string LTConstraint::toString() const
{
    std::stringstream str;
    str << _left->toString() << " < " << _right->toString();
    return str.str();
}

void LTConstraint::Eval(const Tape& tape, const Parameter* params, double* result, const double* vars, int dim)
{
    const double* l = tape.getValues(params[0].asIdx);
    const double* r = tape.getValues(params[1].asIdx);

    double val = r[0] - l[0];
    if (val > 0.0) {
        result[0] = 1.0;
        for (int i = 1; i <= dim; ++i) {
            result[i] = 0.0;
        }
    } else {
        const double steep = Term::getConstraintSteepness();
        result[0] = val * steep;
        for (int i = 1; i <= dim; ++i) {
            result[i] = (r[i] - l[i]) * steep;
        }
    }
}

} /* namespace autodiff */
