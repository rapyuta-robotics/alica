#include <alica_tests/SchedulingTestPlan11613378406860.h>
/*PROTECTED REGION ID(eph1613378406860) ENABLED START*/
// Add additional options here
#include <alica/test/CounterClass.h>
#include <assert.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  SchedulingTestPlan1 (1613378406860)
//
// Tasks:
//   - SchedulerTestSubPlanEntrypoint (1613372009777) (Entrypoint: 1613378541158)
//
// States:
//   - InitPlan1 (1613378543512)
//   - TerminateSubPlans (1613977406218)
//   - InitSubPlans (1614960038398)
SchedulingTestPlan11613378406860::SchedulingTestPlan11613378406860(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con1613378406860) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
SchedulingTestPlan11613378406860::~SchedulingTestPlan11613378406860()
{
    /*PROTECTED REGION ID(dcon1613378406860) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: SchedulerTestSubPlanEntrypoint  -> EntryPoint-ID: 1613378541158
 */
std::shared_ptr<UtilityFunction> UtilityFunction1613378406860::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1613378406860) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1613378406860) ENABLED START*/
// Add additional options here
void SchedulingTestPlan11613378406860::onInit()
{
    LockedBlackboardRW bb(*(getBlackboard()));
    bb.set("Plan2Sub", 2);
    bb.set("Init2Term", 5);
    CounterClass::called = 1;
}

void SchedulingTestPlan11613378406860::onTerminate()
{
    CounterClass::called += 1;
}
/*PROTECTED REGION END*/
} // namespace alica
