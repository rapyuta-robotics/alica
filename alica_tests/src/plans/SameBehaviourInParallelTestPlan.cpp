#include <alica_tests/plans/SameBehaviourInParallelTestPlan.h>

namespace alica
{
SameBehaviourInParallelTestPlan::SameBehaviourInParallelTestPlan(PlanContext& context)
        : BasicPlan(context)
{
}

std::unique_ptr<SameBehaviourInParallelTestPlan> SameBehaviourInParallelTestPlan::create(alica::PlanContext& context)
{
    return std::make_unique<SameBehaviourInParallelTestPlan>(context);
}
} // namespace alica
