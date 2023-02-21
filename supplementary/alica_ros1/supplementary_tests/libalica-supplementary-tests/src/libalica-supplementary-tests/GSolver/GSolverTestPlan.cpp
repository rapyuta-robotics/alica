#include <autodiff/AutoDiff.h>
#include <engine/constraintmodul/ProblemDescriptor.h>
#include <iostream>
#include <libalica-supplementary-tests/GSolver/GSolverTestPlan.h>

using namespace autodiff;

namespace alica
{
GSolverTestPlan::GSolverTestPlan(PlanContext& context)
        : BasicPlan(context)
{
}

std::shared_ptr<UtilityFunction> GSolverTestPlanUtilityFunction::getUtilityFunction(Plan* plan)
{
    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
}

void GSolverTestPlanRuntimeConditionConstraint::getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp)
{
    autodiff::Variable* xv = dynamic_cast<autodiff::Variable*>(c->getStaticVars()[0]);
    if (xv == nullptr) {
        std::cout << "Constraint1417424512343::getConstraint !!!!!!!!!!!!!!!!!!!!! error" << std::endl;
        return;
    }
    xv->editRange().intersect(0.0, 10000.0);

    autodiff::Variable* yv = dynamic_cast<autodiff::Variable*>(c->getStaticVars()[1]);
    if (yv == nullptr) {
        std::cout << "Constraint1417424512343::getConstraint !!!!!!!!!!!!!!!!!!!!! error" << std::endl;
        return;
    }
    yv->editRange().intersect(0.0, 10000.0);

    TermPtr x = xv;
    TermPtr y = yv;

    TermPtr constraint = x->getOwner()->trueConstant();
    constraint = constraint & (x->getOwner()->constant(5000) > x);
    constraint = constraint & (x->getOwner()->constant(4000) < x);

    constraint = constraint & (x->getOwner()->constant(8000) > y);
    constraint = constraint & (x->getOwner()->constant(7000) < y);
    c->setConstraint(constraint);

    TermPtr utility = x->getOwner()->constant(1.0);
    c->setUtility(utility);
    c->setUtilitySufficiencyThreshold(1.0);
}

bool GSolverTestPlanRuntimeCondition::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    return true;
}

std::unique_ptr<GSolverTestPlan> GSolverTestPlan::create(alica::PlanContext& context)
{
    return std::make_unique<GSolverTestPlan>(context);
}

std::unique_ptr<GSolverTestPlanUtilityFunction> GSolverTestPlanUtilityFunction::create(alica::UtilityFunctionContext& context)
{
    return std::make_unique<GSolverTestPlanUtilityFunction>();
}

std::unique_ptr<GSolverTestPlanRuntimeCondition> GSolverTestPlanRuntimeCondition::create(alica::ConditionContext& context)
{
    return std::make_unique<GSolverTestPlanRuntimeCondition>();
}

std::unique_ptr<GSolverTestPlanRuntimeConditionConstraint> GSolverTestPlanRuntimeConditionConstraint::create(alica::ConstraintContext& context)
{
    return std::make_unique<GSolverTestPlanRuntimeConditionConstraint>();
}

} // namespace alica
