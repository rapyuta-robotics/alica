#include <assert.h>
#include <autodiff/AutoDiff.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/constraintmodul/ProblemDescriptor.h>
#include <libalica-supplementary-tests/VariableHandling/Lvl2.h>
using autodiff::TermPtr;

namespace alica
{
Lvl2::Lvl2(PlanContext& context)
        : BasicPlan(context)
{
}

std::shared_ptr<UtilityFunction> Lvl2UtilityFunction::getUtilityFunction(Plan* plan)
{
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
}

void Lvl2RuntimeConditionConstraint::getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp)
{
    assert(c->getStaticVars().size() == 3);
    assert(c->getAgentVars().size() == 1);
    assert(c->getAgentVars()[0].getVars().size() == 2);

    autodiff::Variable* xv = static_cast<autodiff::Variable*>(c->getAgentVars()[0].getVars()[0]);
    autodiff::Variable* yv = static_cast<autodiff::Variable*>(c->getAgentVars()[0].getVars()[1]);

    xv->editRange().intersect(50.0, 150.0);
    yv->editRange().intersect(50.0, 150.0);
    TermPtr x = xv;
    TermPtr y = yv;

    TermPtr constraint = x > y;

    c->setConstraint(constraint);
    TermPtr utility = x + y;

    c->setUtility(utility);
    c->setUtilitySufficiencyThreshold(280.0);
}

bool Lvl2RuntimeCondition::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    return true;
}

std::unique_ptr<Lvl2> Lvl2::create(PlanContext& context)
{
    return std::make_unique<Lvl2>(context);
}

std::shared_ptr<Lvl2UtilityFunction> Lvl2UtilityFunction::create(UtilityFunctionContext& context)
{
    return std::make_shared<Lvl2UtilityFunction>();
}

std::shared_ptr<Lvl2RuntimeCondition> Lvl2RuntimeCondition::create(alica::ConditionContext& context)
{
    return std::make_shared<Lvl2RuntimeCondition>();
}

std::shared_ptr<Lvl2RuntimeConditionConstraint> Lvl2RuntimeConditionConstraint::create(alica::ConstraintContext& context)
{
    return std::make_shared<Lvl2RuntimeConditionConstraint>();
}

} // namespace alica
