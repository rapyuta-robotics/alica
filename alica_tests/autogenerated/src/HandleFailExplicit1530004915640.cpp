#include "HandleFailExplicit1530004915640.h"
/*PROTECTED REGION ID(eph1530004915640) ENABLED START*/
// Add additional using directives here
#include "SimpleSwitches.h"
/*PROTECTED REGION END*/

namespace alica
{
// Plan:HandleFailExplicit
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1530004915642
 */
std::shared_ptr<UtilityFunction> UtilityFunction1530004915640::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1530004915640) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: MISSING_NAME, ConditionString: , Comment: From A to B, isset(0)
 *
 * Abstract plans in current state:
 *
 * Tasks in plan:
 *   - DefaultTask (1225112227903) (Entrypoint: 1530004915642)
 *
 * States in plan:
 *   - A (1530004915641)
 *   - B (1530004973591)
 *   - C (1530004975275)
 *   - D (1532424087894)
 *   - E (1532424097662)
 *
 * Variables of precondition:
 */
bool PreCondition1530004993493::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1530004992551) ENABLED START*/
    return SimpleSwitches::isSet(0);
    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: MISSING_NAME, ConditionString: , Comment: AnyChildFail
 *
 * Abstract plans in current state:
 *   - FailsOnOne (1530069246103)
 *
 * Tasks in plan:
 *   - DefaultTask (1225112227903) (Entrypoint: 1530004915642)
 *
 * States in plan:
 *   - A (1530004915641)
 *   - B (1530004973591)
 *   - C (1530004975275)
 *   - D (1532424087894)
 *   - E (1532424097662)
 *
 * Variables of precondition:
 */
bool PreCondition1530004994611::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1530004993680) ENABLED START*/
    return rp->isAnyChildStatus(PlanStatus::Failed);
    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: MISSING_NAME, ConditionString: , Comment: C to D, isset(2)
 *
 * Abstract plans in current state:
 *
 * Tasks in plan:
 *   - DefaultTask (1225112227903) (Entrypoint: 1530004915642)
 *
 * States in plan:
 *   - A (1530004915641)
 *   - B (1530004973591)
 *   - C (1530004975275)
 *   - D (1532424087894)
 *   - E (1532424097662)
 *
 * Variables of precondition:
 */
bool PreCondition1532424093178::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1532424092280) ENABLED START*/
    return SimpleSwitches::isSet(2);
    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: MISSING_NAME, ConditionString: , Comment: AnyChildFail
 *
 * Abstract plans in current state:
 *   - AlwaysFail (1532424188199)
 *
 * Tasks in plan:
 *   - DefaultTask (1225112227903) (Entrypoint: 1530004915642)
 *
 * States in plan:
 *   - A (1530004915641)
 *   - B (1530004973591)
 *   - C (1530004975275)
 *   - D (1532424087894)
 *   - E (1532424097662)
 *
 * Variables of precondition:
 */
bool PreCondition1532424113475::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1532424112331) ENABLED START*/
    return rp->isAnyChildStatus(PlanStatus::Failed);
    /*PROTECTED REGION END*/
}
} // namespace alica
