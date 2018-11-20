
#include "Sum.h"

#include "Constant.h"
#include "TermHolder.h"

#include <cmath>
#include <sstream>

namespace autodiff
{

Sum::Sum(TermPtr first, TermPtr second, TermHolder* owner)
    : BinaryFunction(first, second, owner)
{
}

int Sum::accept(ITermVisitor* visitor)
{
    return visitor->visit(this);
}

void Sum::acceptRecursive(ITermVisitor* visitor)
{
    _left->acceptRecursive(visitor);
    _right->acceptRecursive(visitor);
    visitor->visit(this);
}

TermPtr Sum::aggregateConstants()
{
    _left = _left->aggregateConstants();
    _right = _right->aggregateConstants();
    if (_left->isConstant() && _right->isConstant()) {
        return _owner->constant(static_cast<Constant*>(_left)->getValue() + static_cast<Constant*>(_right)->getValue());
    }
    if (_left->isConstant()) {
        if (static_cast<Constant*>(_left)->getValue() == 0.0) {
            return _right;
        }
    }
    if (_right->isConstant()) {
        if (static_cast<Constant*>(_right)->getValue() == 0.0) {
            return _left;
        }
    }
    return this;
}

TermPtr Sum::derivative(VarPtr v) const
{
    return _left->derivative(v) + _right->derivative(v);
}

std::string Sum::toString() const
{
    std::stringstream str;
    str << "( " << _left->toString() << " + " << _right->toString() << " )";
    return str.str();
}

void Sum::Eval(const Tape& tape, const Parameter* params, double* result, const double* vars, int dim)
{
    const double* l = tape.getValues(params[0].asIdx);
    const double* r = tape.getValues(params[1].asIdx);
    for (int i = 0; i <= dim; ++i) {
        result[i] = l[i] + r[i];
    }
}

} /* namespace autodiff */
