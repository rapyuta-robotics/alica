#include <alica_tests/plans/GlobalCounterIncreasePlan.h>

namespace alica
{
GlobalCounterIncreasePlan::GlobalCounterIncreasePlan(PlanContext& context)
        : AlicaTestsPlan(context)
{
}
void GlobalCounterIncreasePlan::onInit()
{
    LockedBlackboardRW gb(*getGlobalBlackboard());
    gb.set("counter", gb.get<int64_t>("counter") + 1);
}

void GlobalCounterIncreasePlan::onTerminate()
{
    LockedBlackboardRW gb(*getGlobalBlackboard());
    gb.set("counter", gb.get<int64_t>("counter") + 1);
}
} // namespace alica
