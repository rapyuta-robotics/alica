#include "SchedulingTestMasterPlan1613378382024.h"
/*PROTECTED REGION ID(eph1613378382024) ENABLED START*/
// Add additional options here
#include "CounterClass.h"
/*PROTECTED REGION END*/

namespace alica
{
// Plan:SchedulingTestMasterPlan1613378382024
SchedulingTestMasterPlan1613378382024::SchedulingTestMasterPlan1613378382024()
        : DomainPlan()
{
    /*PROTECTED REGION ID(con1613378382024) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
SchedulingTestMasterPlan1613378382024::~SchedulingTestMasterPlan1613378382024()
{
    /*PROTECTED REGION ID(dcon1613378382024) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
/**
 * Task: SchedulerTestEntrypoint  -> EntryPoint-ID: 1615797283419
 */
std::shared_ptr<UtilityFunction> UtilityFunction1613378382024::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1613378382024) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: 1613530643882, ConditionString: , Comment: MISSING_COMMENT
 *
 * Abstract plans in current state:
 *   - SchedulingTestPlan1 (1613378406860)
 *
 * Tasks in plan:
 *   - SchedulerTestEntrypoint (1613371619454) (Entrypoint: 1615797283419)
 *
 * States in plan:
 *   - InitTest (1613378474109)
 *   - EndTest (1613530614559)
 *   - Default Name (1615797271229)
 *   - Default Name (1615797319003)
 *
 * Variables of precondition:
 */
bool PreCondition1613530643882::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1613530643879) ENABLED START*/
    return CounterClass::called == 8;
    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: 1615797316171, ConditionString: , Comment: MISSING_COMMENT
 *
 * Abstract plans in current state:
 *
 * Tasks in plan:
 *   - SchedulerTestEntrypoint (1613371619454) (Entrypoint: 1615797283419)
 *
 * States in plan:
 *   - InitTest (1613378474109)
 *   - EndTest (1613530614559)
 *   - Default Name (1615797271229)
 *   - Default Name (1615797319003)
 *
 * Variables of precondition:
 */
bool PreCondition1615797316171::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1615797316170) ENABLED START*/
    return CounterClass::called == 0;
    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: 1615797327077, ConditionString: , Comment: MISSING_COMMENT
 *
 * Abstract plans in current state:
 *
 * Tasks in plan:
 *   - SchedulerTestEntrypoint (1613371619454) (Entrypoint: 1615797283419)
 *
 * States in plan:
 *   - InitTest (1613378474109)
 *   - EndTest (1613530614559)
 *   - Default Name (1615797271229)
 *   - Default Name (1615797319003)
 *
 * Variables of precondition:
 */
bool PreCondition1615797327077::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1615797327076) ENABLED START*/
    return CounterClass::called == 1;
    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: 1615797365364, ConditionString: , Comment: MISSING_COMMENT
 *
 * Abstract plans in current state:
 *   - SchedulingTestSequencePlan1 (1614963946725)
 *
 * Tasks in plan:
 *   - SchedulerTestEntrypoint (1613371619454) (Entrypoint: 1615797283419)
 *
 * States in plan:
 *   - InitTest (1613378474109)
 *   - EndTest (1613530614559)
 *   - Default Name (1615797271229)
 *   - Default Name (1615797319003)
 *
 * Variables of precondition:
 */
bool PreCondition1615797365364::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1615797365363) ENABLED START*/
    return CounterClass::called == 4;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1613378382024) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
