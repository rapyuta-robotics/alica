#include <alica_tests/GoalPlan.h>

namespace alica
{
GoalPlan::GoalPlan(PlanContext& context)
        : DomainPlan(context)
{
}

bool PreCondition1402489131988::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    return true;
}

bool RunTimeCondition1403773741874::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    return true;
}

bool PostCondition1402489620773::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    return false;
}
} // namespace alica
