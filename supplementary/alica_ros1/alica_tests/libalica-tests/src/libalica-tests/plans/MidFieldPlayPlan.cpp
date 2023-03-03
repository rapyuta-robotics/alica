#include <libalica-tests/plans/MidFieldPlayPlan.h>

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
