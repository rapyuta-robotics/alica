
#include "Atan2.h"

#include "ConstPower.h"
#include "Constant.h"
#include "TermHolder.h"

#include <cmath>
#include <sstream>

namespace autodiff
{
Atan2::Atan2(TermPtr left, TermPtr right, TermHolder* owner)
    : BinaryFunction(left, right, owner)
{
}

int Atan2::accept(ITermVisitor* visitor)
{
    return visitor->visit(this);
}
void Atan2::acceptRecursive(ITermVisitor* visitor)
{
    _left->acceptRecursive(visitor);
    _right->acceptRecursive(visitor);
    visitor->visit(this);
}

TermPtr Atan2::aggregateConstants()
{
    _left = _left->aggregateConstants();
    _right = _right->aggregateConstants();
    if (_left->isConstant() && _right->isConstant()) {
        return _owner->constant(atan2(static_cast<Constant*>(_left)->getValue(), static_cast<Constant*>(_right)->getValue()));
    } else {
        return this;
    }
}

TermPtr Atan2::derivative(VarPtr v) const
{
    TermPtr t = _left * _right->derivative(v) - _right * _left->derivative(v);
    return t / (_owner->constPower(_left, 2) + _owner->constPower(_right, 2));
}

std::string Atan2::toString() const
{
    std::stringstream str;
    str << "atan2( " << _left->toString();
    str << ", " << _right->toString();
    str << " )";
    return str.str();
}

void Atan2::Eval(const Tape& tape, const Parameter* params, double* result, const double* vars, int dim)
{
    const double* l = tape.getValues(params[0].asIdx);
    const double* r = tape.getValues(params[1].asIdx);
    result[0] = atan2(l[0], r[0]);
    const double denom = 1.0 / (l[0] * l[0] + r[0] * r[0]);
    for (int i = 1; i <= dim; ++i) {
        result[i] = (l[0] * r[i] - r[0] * l[i]) * denom;
    }
}
} /* namespace autodiff */
