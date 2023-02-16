#include <libalica-supplementary-tests/VariableHandling/Lvl1.h>
#include <engine/DefaultUtilityFunction.h>
#include <assert.h>
#include <autodiff/AutoDiff.h>
#include <engine/constraintmodul/ProblemDescriptor.h>
using autodiff::TermPtr;

namespace alica
{
Lvl1::Lvl1(PlanContext& context)
        : BasicPlan(context)
{
}

std::shared_ptr<UtilityFunction> Lvl1UtilityFunction::getUtilityFunction(Plan* plan)
{
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
}

void Lvl1RuntimeConditionConstraint::getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp)
{
    autodiff::Variable* l1av = static_cast<autodiff::Variable*>(c->getStaticVars()[0]);
    autodiff::Variable* l1bv = static_cast<autodiff::Variable*>(c->getStaticVars()[1]);

    TermPtr l1a = l1av;
    TermPtr l1b = l1bv;

    assert(c->getStaticVars().size() == 2);
    assert(c->getAgentVars().empty());

    l1av->editRange().intersect(-200.0, -100.0);
    l1bv->editRange().intersect(100.0, 200.0);

    TermPtr constraint = l1a + l1b < l1b->getOwner()->constant(10.0);
    constraint &= l1a + l1b > l1b->getOwner()->constant(-10.0);

    c->setConstraint(constraint);

    TermPtr utility = l1b->getOwner()->constant(1);
    c->setUtility(utility);
    c->setUtilitySufficiencyThreshold(10.0);
}

void Lvl1PreConditionConstraint::getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp)
{
    assert(c->getStaticVars().empty());
    assert(c->getAgentVars().size() == 2);

    autodiff::Variable* x1v = static_cast<autodiff::Variable*>(c->getAgentVars()[0].getVars()[0]);
    autodiff::Variable* y1v = static_cast<autodiff::Variable*>(c->getAgentVars()[0].getVars()[1]);

    autodiff::Variable* x2v = static_cast<autodiff::Variable*>(c->getAgentVars()[1].getVars()[0]);
    autodiff::Variable* y2v = static_cast<autodiff::Variable*>(c->getAgentVars()[1].getVars()[1]);

    TermPtr x1 = x1v;
    TermPtr y1 = y1v;
    TermPtr x2 = x2v;
    TermPtr y2 = y2v;

    TermPtr constraint = (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) > x1->getOwner()->constant(10000.0);
    constraint &= x1 > y1;
    constraint &= x2 > y2;

    c->setConstraint(constraint);
    TermPtr utility = x1->getOwner()->constant(1.0);

    c->setUtility(utility);
}

bool Lvl1RuntimeCondition::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    return true;
}

std::unique_ptr<Lvl1> Lvl1::create(PlanContext& context)
{
    return std::make_unique<Lvl1>(context);
}

std::unique_ptr<Lvl1UtilityFunction> Lvl1UtilityFunction::create(UtilityFunctionContext& context)
{
    return std::make_unique<Lvl1UtilityFunction>();
}

std::unique_ptr<Lvl1RuntimeCondition> Lvl1RuntimeCondition::create(alica::ConditionContext& context)
{
    return std::make_unique<Lvl1RuntimeCondition>();
}

std::unique_ptr<Lvl1RuntimeConditionConstraint> Lvl1RuntimeConditionConstraint::create(alica::ConstraintContext& context)
{
    return std::make_unique<Lvl1RuntimeConditionConstraint>();
}

std::unique_ptr<Lvl1PreConditionConstraint> Lvl1PreConditionConstraint::create(alica::ConstraintContext& context)
{
    return std::make_unique<Lvl1PreConditionConstraint>();
}

} // namespace alica
