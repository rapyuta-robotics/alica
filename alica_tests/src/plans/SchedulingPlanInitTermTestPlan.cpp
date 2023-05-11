#include <alica_tests/TestWorldModel.h>
#include <alica_tests/plans/SchedulingPlanInitTermTestPlan.h>

namespace alica
{
SchedulingPlanInitTermTestPlan::SchedulingPlanInitTermTestPlan(PlanContext& context)
        : AlicaTestsPlan(context)
{
}
void SchedulingPlanInitTermTestPlan::onInit()
{
    LockedBlackboardRW gb(*getGlobalBlackboard());
    gb.set("counter", 1);
}

void SchedulingPlanInitTermTestPlan::onTerminate()
{
    LockedBlackboardRW gb(*getGlobalBlackboard());
    gb.set("counter", gb.get<int64_t>("counter") + 1);
}
} // namespace alica
