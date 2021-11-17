
#include "intervals/ResetIntervals.h"

#include <autodiff/AutoDiff.h>

#include <limits>
#include <math.h>

namespace alica
{
namespace reasoner
{
namespace intervalpropagation
{
using std::numeric_limits;

ResetIntervals::ResetIntervals() {}

int ResetIntervals::visit(autodiff::Abs* abs)
{
    abs->editParents().clear();
    updateInterval(abs, 0, numeric_limits<double>::infinity());
    return true;
}

int ResetIntervals::visit(autodiff::And* and_)
{
    and_->editParents().clear();
    updateInterval(and_, -numeric_limits<double>::infinity(), 1);
    // updateInterval(and,1,1); //enforce the purely conjunctive problem
    return true;
}

int ResetIntervals::visit(autodiff::Atan2* atan2)
{
    atan2->editParents().clear();
    updateInterval(atan2, -M_PI, M_PI);
    return true;
}

int ResetIntervals::visit(autodiff::Constant* constant)
{
    constant->editParents().clear();
    updateInterval(constant, constant->getValue(), constant->getValue());
    return true;
}

int ResetIntervals::visit(autodiff::ConstPower* intPower)
{
    intPower->editParents().clear();

    if (intPower->getExponent() == 0) {
        updateInterval(intPower, 1, 1);
        return true;
    }
    double e = std::round(intPower->getExponent());
    if (intPower->getExponent() == e && ((int)e) % 2 == 0) {
        updateInterval(intPower, 0, numeric_limits<double>::infinity());
        return true;
    }
    updateInterval(intPower, -numeric_limits<double>::infinity(), numeric_limits<double>::infinity());
    return false;
}

int ResetIntervals::visit(autodiff::ConstraintUtility* cu)
{
    cu->editParents().clear();
    updateInterval(cu, 1, numeric_limits<double>::infinity());
    return true;
}

int ResetIntervals::visit(autodiff::Cos* cos)
{
    cos->editParents().clear();
    updateInterval(cos, -1, 1);
    return true;
}

int ResetIntervals::visit(autodiff::Exp* exp)
{
    exp->editParents().clear();
    updateInterval(exp, 0, numeric_limits<double>::infinity());
    return true;
}

int ResetIntervals::visit(autodiff::LinSigmoid* sigmoid)
{
    sigmoid->editParents().clear();
    updateInterval(sigmoid, 0, 1);
    return true;
}

int ResetIntervals::visit(autodiff::Log* log)
{
    log->editParents().clear();
    updateInterval(log, -numeric_limits<double>::infinity(), numeric_limits<double>::infinity());
    return true;
}

int ResetIntervals::visit(autodiff::LTConstraint* constraint)
{
    constraint->editParents().clear();
    updateInterval(constraint, -numeric_limits<double>::infinity(), 1);
    return true;
}

int ResetIntervals::visit(autodiff::LTEConstraint* constraint)
{
    constraint->editParents().clear();
    updateInterval(constraint, -numeric_limits<double>::infinity(), 1);
    return true;
}

int ResetIntervals::visit(autodiff::Max* max)
{
    max->editParents().clear();
    updateInterval(max, -numeric_limits<double>::infinity(), numeric_limits<double>::infinity());
    return true;
}

int ResetIntervals::visit(autodiff::Min* min)
{
    min->editParents().clear();
    updateInterval(min, -numeric_limits<double>::infinity(), numeric_limits<double>::infinity());
    return true;
}

int ResetIntervals::visit(autodiff::Or* or_)
{
    or_->editParents().clear();
    updateInterval(or_, -numeric_limits<double>::infinity(), 1);
    return true;
}

int ResetIntervals::visit(autodiff::Product* product)
{
    product->editParents().clear();
    updateInterval(product, -numeric_limits<double>::infinity(), numeric_limits<double>::infinity());
    return false;
}

int ResetIntervals::visit(autodiff::Reification* reif)
{
    reif->editParents().clear();
    updateInterval(reif, reif->getMin(), reif->getMax());
    return true;
}

int ResetIntervals::visit(autodiff::Sigmoid* sigmoid)
{
    sigmoid->editParents().clear();
    updateInterval(sigmoid, 0, 1);
    return true;
}

int ResetIntervals::visit(autodiff::Sin* sin)
{
    sin->editParents().clear();
    updateInterval(sin, -1, 1);
    return true;
}

int ResetIntervals::visit(autodiff::Sum* sum)
{
    sum->editParents().clear();
    updateInterval(sum, -numeric_limits<double>::infinity(), numeric_limits<double>::infinity());
    return true;
}

int ResetIntervals::visit(autodiff::TermPower* power)
{
    power->editParents().clear();
    updateInterval(power, -numeric_limits<double>::infinity(), numeric_limits<double>::infinity());
    return false;
}

int ResetIntervals::visit(autodiff::Variable* var)
{
    var->editParents().clear();
    updateInterval(var, var->getRange().getMin(), var->getRange().getMax());
    return true;
}

void ResetIntervals::updateInterval(autodiff::TermPtr t, double min, double max)
{
    t->setMin(min);
    t->setMax(max);
    return;
}

} /* namespace intervalpropagation */
} /* namespace reasoner */
} /* namespace alica */
