/*
 * Lit.cpp
 *
 *  Created on: Dec 4, 2014
 *      Author: Philipp
 */

#include "types/Lit.h"
#include "types/Var.h"

#include <autodiff/AutoDiff.h>

#include <iostream>

namespace alica
{
namespace reasoner
{
namespace cnsat
{

Lit::Lit(const std::shared_ptr<Var>& v, Assignment ass)
{
    this->var = v;
    this->sign = ass;
    if (ass == Assignment::TRUE) {
        var->positiveAppearance++;
    } else {
        var->negActivity++;
    }
    this->isTemporary = false;
    variableCount = -1;
}

Lit::Lit(autodiff::TermPtr t, Assignment ass, bool temp)
    : _atom(t)
    , isTemporary(temp)
    , sign(ass)
    , variableCount(-1)
{
}

Lit::~Lit() {}

bool Lit::satisfied() const
{
    return sign == var->assignment;
}

bool Lit::conflicted() const
{
    return var->assignment != sign && var->assignment != Assignment::UNASSIGNED;
}

void Lit::computeVariableCount()
{
    variableCount = 0;
    _atom->acceptRecursive(this);
}

int Lit::visit(autodiff::Abs* abs)
{
    return 0;
}

int Lit::visit(autodiff::And* and_)
{
    return 0;
}

int Lit::visit(autodiff::Atan2* atan2)
{
    return 0;
}

int Lit::visit(autodiff::Constant* constant)
{
    return 0;
}

int Lit::visit(autodiff::ConstPower* intPower)
{
    return 0;
}

int Lit::visit(autodiff::ConstraintUtility* cu)
{
    return 0;
}

int Lit::visit(autodiff::Cos* cos)
{
    return 0;
}

int Lit::visit(autodiff::Exp* exp)
{
    return 0;
}

int Lit::visit(autodiff::LinSigmoid* sigmoid)
{
    return 0;
}

int Lit::visit(autodiff::Log* log)
{
    return 0;
}

int Lit::visit(autodiff::LTConstraint* constraint)
{
    return 0;
}

int Lit::visit(autodiff::LTEConstraint* constraint)
{
    return 0;
}

int Lit::visit(autodiff::Max* max)
{
    return 0;
}

int Lit::visit(autodiff::Min* min)
{
    return 0;
}

int Lit::visit(autodiff::Or* or_)
{
    return 0;
}

int Lit::visit(autodiff::Product* product)
{
    return 0;
}

int Lit::visit(autodiff::Reification* dis)
{
    return 0;
}

int Lit::visit(autodiff::Sigmoid* sigmoid)
{
    return 0;
}

int Lit::visit(autodiff::Sin* sin)
{
    return 0;
}

int Lit::visit(autodiff::Sum* sum)
{
    return 0;
}

int Lit::visit(autodiff::TermPower* power)
{
    return 0;
}

int Lit::visit(autodiff::Variable* var)
{
    ++variableCount;
    return 0;
}

} /* namespace cnsat */
} /* namespace reasoner */
} /* namespace alica */
