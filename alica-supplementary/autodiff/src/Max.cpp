
#include "Max.h"

#include "Constant.h"
#include "TermHolder.h"

#include <cmath>
#include <sstream>

namespace autodiff
{
Max::Max(TermPtr left, TermPtr right, TermHolder* owner)
    : BinaryFunction(left, right, owner)
{
}

int Max::accept(ITermVisitor* visitor)
{
    return visitor->visit(this);
}

void Max::acceptRecursive(ITermVisitor* visitor)
{
    _left->acceptRecursive(visitor);
    _right->acceptRecursive(visitor);
    visitor->visit(this);
}

TermPtr Max::aggregateConstants()
{
    _left = _left->aggregateConstants();
    if (_left == _owner->trueConstant()) {
        return _left;
    }
    _right = _right->aggregateConstants();
    if (_left == _owner->falseConstant()) {
        return _right;
    }
    if (_right == _owner->trueConstant()) {
        return _right;
    }
    if (_right == _owner->falseConstant()) {
        return _left;
    }
    if (_left->isConstant() && _right->isConstant()) {
        return _owner->constant(std::max(static_cast<Constant*>(_left)->getValue(), static_cast<Constant*>(_right)->getValue()));
    } else {
        return this;
    }
}

TermPtr Max::derivative(VarPtr v) const
{
    throw "Symbolic Derivation of Max not supported.";
    return nullptr;
}

TermPtr Max::negate() const
{
    return _left->negate() & _right->negate();
}

std::string Max::toString() const
{
    std::stringstream str;
    str << "max( " << _left->toString() << ", " << _right->toString() << " )";
    return str.str();
}

void Max::Eval(const Tape& tape, const Parameter* params, double* result, const double* vars, int dim)
{
    const double* l = tape.getValues(params[0].asIdx);
    const double* r = tape.getValues(params[1].asIdx);

    if (l[0] > r[0]) {
        for (int i = 0; i <= dim; ++i) {
            result[i] = l[i];
        }
    } else {
        for (int i = 0; i <= dim; ++i) {
            result[i] = r[i];
        }
    }
}

} /* namespace autodiff */
