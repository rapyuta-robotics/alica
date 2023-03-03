#include <alica_tests/MidFieldPlayPlan.h>

namespace alica
{
MidFieldPlayPlan::MidFieldPlayPlan(PlanContext& context)
        : BasicPlan(context)
{
}
bool MidFieldPlayPlanRuntimeCondition::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    return true;
}
} // namespace alica
