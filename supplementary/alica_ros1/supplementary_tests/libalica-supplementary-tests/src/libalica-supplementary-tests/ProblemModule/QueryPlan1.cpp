#include <autodiff/AutoDiff.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/constraintmodul/ProblemDescriptor.h>
#include <libalica-supplementary-tests/ProblemModule/QueryPlan1.h>

namespace alica
{
QueryPlan1::QueryPlan1(PlanContext& context)
        : BasicPlan(context)
{
}

std::shared_ptr<UtilityFunction> QueryPlan1UtilityFunction::getUtilityFunction(Plan* plan)
{
    shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::unique_ptr<QueryPlan1> QueryPlan1::create(alica::PlanContext& context)
{
    return std::make_unique<QueryPlan1>(context);
}

std::unique_ptr<QueryPlan1UtilityFunction> QueryPlan1UtilityFunction::create(alica::UtilityFunctionContext& context)
{
    return std::make_unique<QueryPlan1UtilityFunction>();
}

std::unique_ptr<QueryPlan1RuntimeCondition> QueryPlan1RuntimeCondition::create(alica::ConditionContext& context)
{
    return std::make_unique<QueryPlan1RuntimeCondition>();
}

bool QueryPlan1RuntimeCondition::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    return true;
}

std::unique_ptr<QueryPlan1RuntimeConditionConstraint> QueryPlan1RuntimeConditionConstraint::create(alica::ConstraintContext& context)
{
    return std::make_unique<QueryPlan1RuntimeConditionConstraint>();
}

void QueryPlan1RuntimeConditionConstraint::getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp)
{
    autodiff::TermHolder* h = static_cast<autodiff::TermHolder*>(c->getContext());
    c->setConstraint(h->trueConstant());
    c->setUtility(h->constant(1));
}

} // namespace alica
