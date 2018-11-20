#include "Variable.h"

#include "TermHolder.h"

#include <limits>
#include <sstream>

namespace autodiff
{
constexpr double Variable::minExpressibleValue;
constexpr double Variable::maxExpressibleValue;

Variable::Variable(TermHolder* owner, int64_t id)
    : Term(owner)
    , alica::SolverVariable(id)
    , _globalRange(minExpressibleValue, maxExpressibleValue)
    , _varIdx(-1)
{
}

int Variable::accept(ITermVisitor* visitor)
{
    return visitor->visit(this);
}

void Variable::acceptRecursive(ITermVisitor* visitor)
{
    visitor->visit(this);
}

TermPtr Variable::aggregateConstants()
{
    return this;
}

TermPtr Variable::derivative(VarPtr v) const
{
    if (this == v) {
        return _owner->constant(1);
    } else {
        return _owner->zeroConstant();
    }
}

std::string Variable::toString() const
{
    std::stringstream str;
    str << "Var";
    str << std::to_string(getId());
    return str.str();
}

void Variable::Eval(const Tape& tape, const Parameter* params, double* result, const double* vars, int dim)
{
    result[0] = vars[params[0].asIdx];
    result[params[0].asIdx + 1] = 1.0;
}

} /* namespace autodiff */
