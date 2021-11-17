#include "And.h"
#include "Constant.h"
#include "TermHolder.h"

#include <sstream>

namespace autodiff
{
And::And(TermPtr left, TermPtr right, TermHolder* owner)
    : BinaryFunction(left, right, owner)
{
}

int And::accept(ITermVisitor* visitor)
{
    return visitor->visit(this);
}

void And::acceptRecursive(ITermVisitor* visitor)
{
    _left->acceptRecursive(visitor);
    _right->acceptRecursive(visitor);
    visitor->visit(this);
}

TermPtr And::aggregateConstants()
{
    _left = _left->aggregateConstants();
    if (_left == _owner->falseConstant()) {
        return _left;
    }
    _right = _right->aggregateConstants();
    if (_left == _owner->trueConstant()) {
        return _right;
    }
    if (_right == _owner->falseConstant()) {
        return _right;
    }
    if (_right == _owner->trueConstant()) {
        return _left;
    }
    if (_left->isConstant() && _right->isConstant()) {
        if (static_cast<Constant*>(_left)->getValue() > 0.75 && static_cast<Constant*>(_right)->getValue() > 0.75) {
            return _owner->trueConstant();
        } else {
            return _owner->falseConstant();
        }
    } else {
        return this;
    }
}

TermPtr And::derivative(VarPtr /*v*/) const
{
    throw "Symbolic Derivation of And not supported";
    return nullptr;
}

TermPtr And::negate() const
{
    return _left->negate() | _right->negate();
}

std::string And::toString() const
{
    std::stringstream str;
    str << "and( " << _left->toString();
    str << ", " << _right->toString();
    str << " )";
    return str.str();
}

void And::Eval(const Tape& tape, const Parameter* params, double* result, const double* vars, int dim)
{
    const double* l = tape.getValues(params[0].asIdx);
    const double* r = tape.getValues(params[1].asIdx);
    double val = std::min(l[0], r[0]);
    result[0] = val > 0.0 ? 1.0 : val;
    for (int i = 1; i <= dim; ++i) {
        result[i] = l[i] + r[i];
    }
}
} /* namespace autodiff */
