

#include "Min.h"

#include "Constant.h"
#include "TermHolder.h"

#include <cmath>
#include <sstream>

namespace autodiff
{
Min::Min(TermPtr left, TermPtr right, TermHolder* owner)
    : BinaryFunction(left, right, owner)
{
}

int Min::accept(ITermVisitor* visitor)
{
    return visitor->visit(this);
}

void Min::acceptRecursive(ITermVisitor* visitor)
{
    _left->acceptRecursive(visitor);
    _right->acceptRecursive(visitor);
    visitor->visit(this);
}

TermPtr Min::aggregateConstants()
{
    _left = _left->aggregateConstants();
    _right = _right->aggregateConstants();
    if (_left->isConstant() && _right->isConstant()) {
        return _owner->constant(std::min(static_cast<Constant*>(_left)->getValue(), static_cast<Constant*>(_right)->getValue()));
    } else {
        return this;
    }
}

TermPtr Min::derivative(VarPtr v) const
{
    throw "Symbolic Derivation of Min not supported.";
    return nullptr;
}

TermPtr Min::negate() const
{
    return _left->negate() | _right->negate();
}

std::string Min::toString() const
{
    std::stringstream str;
    str << "min( " << _left->toString() << ", " << _right->toString() << " )";
    return str.str();
}

void Min::Eval(const Tape& tape, const Parameter* params, double* result, const double* vars, int dim)
{
    const double* l = tape.getValues(params[0].asIdx);
    const double* r = tape.getValues(params[1].asIdx);

    if (l[0] < r[0]) {
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
