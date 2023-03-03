#include <alica/test/CounterClass.h>
#include <assert.h>
#include <libalica-tests/plans/SchedulingTestPlan2.h>

namespace alica
{
SchedulingTestPlan2::SchedulingTestPlan2(PlanContext& context)
        : AlicaTestsPlan(context)
{
}

void SchedulingTestPlan2::onInit()
{
    CounterClass::called += 1;
}

void SchedulingTestPlan2::onTerminate()
{
    CounterClass::called += 1;
}
} // namespace alica
