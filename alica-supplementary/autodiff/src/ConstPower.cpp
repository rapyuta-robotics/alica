
#include "ConstPower.h"

#include "Constant.h"
#include "TermHolder.h"

#include <cmath>
#include <sstream>

namespace autodiff
{
ConstPower::ConstPower(TermPtr baseTerm, double exponent, TermHolder* owner)
    : Term(owner)
    , _base(baseTerm)
    , _exponent(exponent)
{
}

int ConstPower::accept(ITermVisitor* visitor)
{
    return visitor->visit(this);
}
void ConstPower::acceptRecursive(ITermVisitor* visitor)
{
    _base->acceptRecursive(visitor);
    visitor->visit(this);
}

TermPtr ConstPower::aggregateConstants()
{
    _base = _base->aggregateConstants();

    if (_base->isConstant()) {
        return _owner->constant(pow(static_cast<Constant*>(_base)->getValue(), _exponent));
    }

    ConstPower* constPowerBase = dynamic_cast<ConstPower*>(_base.get());

    if (constPowerBase != nullptr) {
        _exponent *= constPowerBase->_exponent;
        _base = constPowerBase->_base;
    }

    return this;
}

TermPtr ConstPower::derivative(VarPtr v) const
{
    return _owner->constant(_exponent) * _owner->constPower(_base, _exponent - 1) * _base->derivative(v);
}

std::string ConstPower::toString() const
{
    std::stringstream str;
    str << "constPower( ";
    if (_base != nullptr) {
        str << _base->toString();
    } else {
        str << "nullptr";
    }
    str << ", " << _exponent << " )";
    return str.str();
}

void ConstPower::Eval(const Tape& tape, const Parameter* params, double* result, const double* vars, int dim)
{
    const double* l = tape.getValues(params[0].asIdx);
    const double v = params[1].asDouble;
    double val = pow(l[0], v - 1.0);
    result[0] = val * l[0];
    val *= v;
    for (int i = 1; i <= dim; ++i) {
        result[i] = val * l[i];
    }
}

} /* namespace autodiff */
