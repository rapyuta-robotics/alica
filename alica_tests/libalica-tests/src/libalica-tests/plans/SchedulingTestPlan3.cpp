#include <alica/test/CounterClass.h>
#include <assert.h>
#include <libalica-tests/plans/SchedulingTestPlan3.h>

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
