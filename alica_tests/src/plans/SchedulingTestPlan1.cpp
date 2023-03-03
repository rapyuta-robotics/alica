#include <alica/test/CounterClass.h>
#include <alica_tests/plans/SchedulingTestPlan1.h>
#include <assert.h>

namespace alica
{
SchedulingTestPlan1::SchedulingTestPlan1(PlanContext& context)
        : AlicaTestsPlan(context)
{
}

void SchedulingTestPlan1::onInit()
{
    LockedBlackboardRW bb(*(getBlackboard()));
    bb.set<int64_t>("Plan2Sub", 2);
    bb.set<int64_t>("Init2Term", 5);
    CounterClass::called = 1;
}

void SchedulingTestPlan1::onTerminate()
{
    CounterClass::called += 1;
}
} // namespace alica
