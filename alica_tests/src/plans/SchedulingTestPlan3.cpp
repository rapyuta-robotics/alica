#include <alica/test/CounterClass.h>
#include <alica_tests/plans/SchedulingTestPlan3.h>
#include <assert.h>

namespace alica
{
SchedulingTestPlan3::SchedulingTestPlan3(PlanContext& context)
        : AlicaTestsPlan(context)
{
}

void SchedulingTestPlan3::onInit()
{
    CounterClass::called += 1;
}

void SchedulingTestPlan3::onTerminate()
{
    CounterClass::called += 1;
}
} // namespace alica
