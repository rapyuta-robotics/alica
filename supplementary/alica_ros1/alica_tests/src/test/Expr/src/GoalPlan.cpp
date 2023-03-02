#include <alica_tests/GoalPlan.h>

namespace alica
{
GoalPlan::GoalPlan(PlanContext& context)
        : DomainPlan(context)
{
}

bool GoalPlanPreCondition::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    return true;
}

bool GoalPlanRuntimeCondition::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    return true;
}

bool GoalPlanPostCondition::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    return false;
}

void GoalPlanRuntimeConditionConstraint::getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp) {}
} // namespace alica
