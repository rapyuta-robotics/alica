#include <alica_tests/plans/TestPlanKeyMappingPlan.h>

namespace alica
{
TestPlanKeyMappingPlan::TestPlanKeyMappingPlan(PlanContext& context)
        : AlicaTestsPlan(context)
{
}

void TestPlanKeyMappingPlan::onInit()
{
    LockedBlackboardRW bb(*getBlackboard());
    bb.set("parentPlanKey", 5);
}

} // namespace alica
