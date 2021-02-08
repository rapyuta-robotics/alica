#include "MasterPlanTestConditionPlanType1418042656594.h"
/*PROTECTED REGION ID(eph1418042656594) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:MasterPlanTestConditionPlanType
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1418042656596
 */
std::shared_ptr<UtilityFunction> UtilityFunction1418042656594::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1418042656594) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: MISSING_NAME, ConditionString: , Comment:
 *
 * Abstract plans in current state:
 *
 * Tasks in plan:
 *   - DefaultTask (1225112227903) (Entrypoint: 1418042656596)
 *
 * States in plan:
 *   - Start (1418042656595)
 *   - Plantype (1418042674811)
 *
 * Variables of precondition:
 */
bool PreCondition1418042683692::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1418042682960) ENABLED START*/
    return true;
    /*PROTECTED REGION END*/
}
} // namespace alica
