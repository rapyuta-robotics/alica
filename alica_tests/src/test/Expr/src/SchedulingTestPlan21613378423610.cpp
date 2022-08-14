#include <alica_tests/SchedulingTestPlan21613378423610.h>
/*PROTECTED REGION ID(eph1613378423610) ENABLED START*/
// Add additional options here
#include <alica_tests/CounterClass.h>
#include <assert.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  SchedulingTestPlan2 (1613378423610)
//
// Tasks:
//   - SchedulerTestSubPlanEntrypoint (1613372009777) (Entrypoint: 1613378624077)
//
// States:
//   - InitPlan2 (1613378625757)
SchedulingTestPlan21613378423610::SchedulingTestPlan21613378423610(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con1613378423610) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
SchedulingTestPlan21613378423610::~SchedulingTestPlan21613378423610()
{
    /*PROTECTED REGION ID(dcon1613378423610) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: SchedulerTestSubPlanEntrypoint  -> EntryPoint-ID: 1613378624077
 */

UtilityFunction1613378423610::UtilityFunction1613378423610(IAlicaLogger& logger)
        : BasicUtilityFunction(logger)
{
}

std::shared_ptr<UtilityFunction> UtilityFunction1613378423610::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1613378423610) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan, _logger);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1613378423610) ENABLED START*/
// Add additional options here
void SchedulingTestPlan21613378423610::onInit()
{
    CounterClass::called += 1;
}

void SchedulingTestPlan21613378423610::onTerminate()
{
    CounterClass::called += 1;
}
/*PROTECTED REGION END*/
} // namespace alica
