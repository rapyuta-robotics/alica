#include "SchedulingTestPlan11613378406860.h"
/*PROTECTED REGION ID(eph1613378406860) ENABLED START*/
// Add additional options here
#include "CounterClass.h"
#include <assert.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:SchedulingTestPlan11613378406860
SchedulingTestPlan11613378406860::SchedulingTestPlan11613378406860()
        : DomainPlan("SchedulingTestPlan11613378406860")
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
/**
 * Outgoing transition:
 *   - Name: 1614960055821, ConditionString: , Comment: MISSING_COMMENT
 *
 * Abstract plans in current state:
 *
 * Tasks in plan:
 *   - SchedulerTestSubPlanEntrypoint (1613372009777) (Entrypoint: 1613378541158)
 *
 * States in plan:
 *   - InitPlan1 (1613378543512)
 *   - TerminateSubPlans (1613977406218)
 *   - InitSubPlans (1614960038398)
 *
 * Variables of precondition:
 */
bool PreCondition1614960055821::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1614960055819) ENABLED START*/
    return CounterClass::called == 2;
    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: 1614960063843, ConditionString: , Comment: MISSING_COMMENT
 *
 * Abstract plans in current state:
 *   - SchedulingTestPlan2 (1613378423610)
 *   - SchedulingTestPlan3 (1613378433623)
 *
 * Tasks in plan:
 *   - SchedulerTestSubPlanEntrypoint (1613372009777) (Entrypoint: 1613378541158)
 *
 * States in plan:
 *   - InitPlan1 (1613378543512)
 *   - TerminateSubPlans (1613977406218)
 *   - InitSubPlans (1614960038398)
 *
 * Variables of precondition:
 */
bool PreCondition1614960063843::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1614960063842) ENABLED START*/
    return CounterClass::called == 5;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1613378406860) ENABLED START*/
// Add additional options here
void SchedulingTestPlan11613378406860::init()
{
    CounterClass::called = 1;
}

void SchedulingTestPlan11613378406860::onTermination()
{
    CounterClass::called += 1;
}
/*PROTECTED REGION END*/
} // namespace alica
