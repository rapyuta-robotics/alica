
#include "Reification.h"
#include "Tape.h"
#include <sstream>

namespace autodiff
{
Reification::Reification(TermPtr condition, TermHolder* owner)
    : BinaryFunction(condition, condition->negate(), owner)
{
}

int Reification::accept(ITermVisitor* visitor)
{
    return visitor->visit(this);
}

void Reification::acceptRecursive(ITermVisitor* visitor)
{
    _left->acceptRecursive(visitor);
    _right->acceptRecursive(visitor);
    visitor->visit(this);
}

TermPtr Reification::aggregateConstants()
{
    return this;
}

TermPtr Reification::derivative(VarPtr v) const
{
    throw "Symbolic Derivation of Discretizer not supported.";
    return nullptr;
}

std::string Reification::toString() const
{
    std::stringstream str;
    str << "Discretizer( " << _left->toString() << " )";
    return str.str();
}

void Reification::Eval(const Tape& tape, const Parameter* params, double* result, const double* vars, int dim)
{
    const double* l = tape.getValues(params[0].asIdx);
    const double* r = tape.getValues(params[1].asIdx);
    if (l[0] > 0.75) {
        result[0] = 1.0;
        for (int i = 1; i <= dim; ++i) {
            result[i] = -r[i];
        }
    } else {
        result[0] = 0.0;
        for (int i = 1; i <= dim; ++i) {
            result[i] = l[i];
        }
    }
}

} /* namespace autodiff */
