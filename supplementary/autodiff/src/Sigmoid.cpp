#include "Sigmoid.h"

#include "ConstPower.h"
#include "Constant.h"
#include "Exp.h"
#include "TermHolder.h"

#include <cmath>
#include <limits>
#include <sstream>

namespace autodiff
{

Sigmoid::Sigmoid(TermPtr arg, TermHolder* owner)
    : Term(owner)
    , _arg(arg)
    , _steepness(1.0)
{
}

Sigmoid::Sigmoid(TermPtr arg, double steepness, TermHolder* owner)
    : Term(owner)
    , _arg(arg)
    , _steepness(steepness)
{
}

int Sigmoid::accept(ITermVisitor* visitor)
{
    return visitor->visit(this);
}

void Sigmoid::acceptRecursive(ITermVisitor* visitor)
{
    _arg->acceptRecursive(visitor);
    visitor->visit(this);
}

TermPtr Sigmoid::aggregateConstants()
{
    _arg = _arg->aggregateConstants();
    if (_arg->isConstant()) {
        double e = exp(-_steepness * static_cast<Constant*>(_arg)->getValue());
        if (e == std::numeric_limits<double>::infinity()) {
            return _owner->constant(Term::EPSILON);
        } else {
            e = 1.0 / (1.0 + e);
        }
        if (e < Term::EPSILON) {
            return _owner->constant(Term::EPSILON);
        } else {
            return _owner->constant(e);
        }
    } else {
        return this;
    }
}

TermPtr Sigmoid::derivative(VarPtr v) const
{
    TermPtr epos = _owner->exp(_steepness * _arg);
    TermPtr t = _steepness * _arg->derivative(v) * epos;
    return t / _owner->constPower(epos + _owner->constant(1.0), 2);
}

std::string Sigmoid::toString() const
{
    std::stringstream str;
    str << "sigmoid( " << _arg->toString() << ", " << _steepness << " )";
    return str.str();
}

void Sigmoid::Eval(const Tape& tape, const Parameter* params, double* result, const double* vars, int dim)
{
    const double* l = tape.getValues(params[0].asIdx);
    const double steep = params[1].asDouble;
    const double e = exp(steep * -l[0]);
    result[0] = 1.0 / (1.0 + e);
    const double val = steep * e * result[0] * result[0];
    for (int i = 1; i <= dim; ++i) {
        result[i] = l[i] * val;
    }
}

} /* namespace autodiff */
