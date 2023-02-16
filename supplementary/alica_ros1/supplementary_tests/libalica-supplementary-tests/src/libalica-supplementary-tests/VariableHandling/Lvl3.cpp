#include <assert.h>
#include <autodiff/AutoDiff.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/constraintmodul/ProblemDescriptor.h>
#include <libalica-supplementary-tests/VariableHandling/Lvl3.h>
using autodiff::TermPtr;

namespace alica
{
Lvl3::Lvl3(PlanContext& context)
        : BasicPlan(context)
{
}

std::shared_ptr<UtilityFunction> Lvl3UtilityFunction::getUtilityFunction(Plan* plan)
{
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
}

void Lvl3RuntimeConditionConstraint::getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp)
{
    assert(c->getStaticVars().size() == 2);
    assert(c->getAgentVars().size() == 1);
    assert(c->getAgentVars()[0].getVars().size() == 2);
    TermPtr l1a = static_cast<autodiff::Variable*>(c->getStaticVars()[0]);
    TermPtr l1b = static_cast<autodiff::Variable*>(c->getStaticVars()[1]);

    autodiff::Variable* xv = static_cast<autodiff::Variable*>(c->getAgentVars()[0].getVars()[0]);
    autodiff::Variable* yv = static_cast<autodiff::Variable*>(c->getAgentVars()[0].getVars()[1]);

    TermPtr x = xv;
    TermPtr y = yv;
    xv->editRange().intersect(-5, 5);
    yv->editRange().intersect(-5, 5);

    TermPtr constraint = x * l1a < x->getOwner()->constant(0.0);
    constraint = constraint & (y * l1b > x->getOwner()->constant(0.0));

    c->setConstraint(constraint);
    TermPtr utility = l1a * l1a + l1b * l1b;

    c->setUtility(utility);
    c->setUtilitySufficiencyThreshold(10.0);
}

bool Lvl3RuntimeCondition::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    return true;
}

std::unique_ptr<Lvl3> Lvl3::create(PlanContext& context)
{
    return std::make_unique<Lvl3>(context);
}

std::unique_ptr<Lvl3UtilityFunction> Lvl3UtilityFunction::create(UtilityFunctionContext& context)
{
    return std::make_unique<Lvl3UtilityFunction>();
}

std::unique_ptr<Lvl3RuntimeCondition> Lvl3RuntimeCondition::create(alica::ConditionContext& context)
{
    return std::make_unique<Lvl3RuntimeCondition>();
}

std::unique_ptr<Lvl3RuntimeConditionConstraint> Lvl3RuntimeConditionConstraint::create(alica::ConstraintContext& context)
{
    return std::make_unique<Lvl3RuntimeConditionConstraint>();
}
} // namespace alica
