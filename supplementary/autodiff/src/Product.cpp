
#include "Product.h"

#include "Constant.h"
#include "TermHolder.h"

#include <cmath>
#include <sstream>

namespace autodiff
{

Product::Product(TermPtr left, TermPtr right, TermHolder* owner)
    : BinaryFunction(left, right, owner)
{
}

int Product::accept(ITermVisitor* visitor)
{
    return visitor->visit(this);
}

void Product::acceptRecursive(ITermVisitor* visitor)
{
    _left->acceptRecursive(visitor);
    _right->acceptRecursive(visitor);
    visitor->visit(this);
}

TermPtr Product::aggregateConstants()
{
    _left = _left->aggregateConstants();
    _right = _right->aggregateConstants();
    if (_left->isConstant() && _right->isConstant()) {
        return _owner->constant(static_cast<Constant*>(_left)->getValue() * static_cast<Constant*>(_right)->getValue());
    }
    if (_left->isConstant()) {
        if (static_cast<Constant*>(_left)->getValue() == 0.0) {
            return _left;
        }
        if (static_cast<Constant*>(_left)->getValue() == 1.0) {
            return _right;
        }
    }
    if (_right->isConstant()) {
        if (static_cast<Constant*>(_right)->getValue() == 0.0) {
            return _right;
        }
        if (static_cast<Constant*>(_right)->getValue() == 1.0) {
            return _left;
        }
    }
    return this;
}

TermPtr Product::derivative(VarPtr v) const
{
    return _left * _right->derivative(v) + _right * _left->derivative(v);
}

std::string Product::toString() const
{
    std::stringstream str;
    str << "( " << _left->toString() << " * " << _right->toString() << " )";
    return str.str();
}

void Product::Eval(const Tape& tape, const Parameter* params, double* result, const double* vars, int dim)
{
    const double* l = tape.getValues(params[0].asIdx);
    const double* r = tape.getValues(params[1].asIdx);
    result[0] = l[0] * r[0];
    for (int i = 1; i <= dim; ++i) {
        result[i] = l[0] * r[i] + l[i] * r[0];
    }
}

} /* namespace autodiff */
