
#include "LTEConstraint.h"

#include "Constant.h"
#include "LTConstraint.h"
#include "TermHolder.h"

#include <cmath>
#include <sstream>

namespace autodiff
{
LTEConstraint::LTEConstraint(TermPtr x, TermPtr y, TermHolder* owner)
    : BinaryFunction(x, y, owner)
    , _negatedForm(nullptr)
{
}

int LTEConstraint::accept(ITermVisitor* visitor)
{
    return visitor->visit(this);
}

void LTEConstraint::acceptRecursive(ITermVisitor* visitor)
{
    _left->acceptRecursive(visitor);
    _right->acceptRecursive(visitor);
    visitor->visit(this);
}

TermPtr LTEConstraint::aggregateConstants()
{
    _left = _left->aggregateConstants();
    _right = _right->aggregateConstants();
    if (_left->isConstant() && _right->isConstant()) {
        if (static_cast<Constant*>(_left)->getValue() <= static_cast<Constant*>(_right)->getValue()) {
            return _owner->trueConstant();
        } else {
            return _owner->falseConstant();
        }
    } else {
        return this;
    }
}

TermPtr LTEConstraint::derivative(VarPtr v) const
{
    throw "Symbolic Derivation of Less-Than-Or-Equal not supported.";
    return nullptr;
}

TermPtr LTEConstraint::negate() const
{
    if (_negatedForm == nullptr) {
        _negatedForm = _owner->lessThan(_right, _left);
        _negatedForm->setNegation(this);
    }
    return _negatedForm;
}

std::string LTEConstraint::toString() const
{
    std::stringstream str;
    str << _left->toString() << " <= " << _right->toString();
    return str.str();
}

void LTEConstraint::Eval(const Tape& tape, const Parameter* params, double* result, const double* vars, int dim)
{
    const double* l = tape.getValues(params[0].asIdx);
    const double* r = tape.getValues(params[1].asIdx);

    double val = r[0] - l[0];
    if (val >= 0.0) {
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
