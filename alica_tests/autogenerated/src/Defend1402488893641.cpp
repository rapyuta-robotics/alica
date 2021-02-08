#include "Defend1402488893641.h"
/*PROTECTED REGION ID(eph1402488893641) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:Defend
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1402488903550
 */
std::shared_ptr<UtilityFunction> UtilityFunction1402488893641::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1402488893641) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: MISSING_NAME, ConditionString: , Comment:
 *
 * Abstract plans in current state:
 *   - Tackle (1402488939130)
 *   - Tackle (1402489318663)
 *
 * Tasks in plan:
 *   - DefaultTask (1225112227903) (Entrypoint: 1402488903550)
 *
 * States in plan:
 *   - Tackle (1402488903549)
 *   - GetGoal (1402488910751)
 *   - GetBall (1402488959965)
 *   - TryToDefendGoal (1402489037735)
 *
 * Variables of precondition:
 */
bool PreCondition1402488993122::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1402488991762) ENABLED START*/
    return false;
    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: MISSING_NAME, ConditionString: , Comment:
 *
 * Abstract plans in current state:
 *
 * Tasks in plan:
 *   - DefaultTask (1225112227903) (Entrypoint: 1402488903550)
 *
 * States in plan:
 *   - Tackle (1402488903549)
 *   - GetGoal (1402488910751)
 *   - GetBall (1402488959965)
 *   - TryToDefendGoal (1402489037735)
 *
 * Variables of precondition:
 */
bool PreCondition1402488991641::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1402488990761) ENABLED START*/
    return false;
    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: MISSING_NAME, ConditionString: , Comment:
 *
 * Abstract plans in current state:
 *
 * Tasks in plan:
 *   - DefaultTask (1225112227903) (Entrypoint: 1402488903550)
 *
 * States in plan:
 *   - Tackle (1402488903549)
 *   - GetGoal (1402488910751)
 *   - GetBall (1402488959965)
 *   - TryToDefendGoal (1402489037735)
 *
 * Variables of precondition:
 */
bool PreCondition1402489065962::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1402489064693) ENABLED START*/
    return false;
    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: MISSING_NAME, ConditionString: , Comment:
 *
 * Abstract plans in current state:
 *   - PlanType (1402489564599)
 *
 * Tasks in plan:
 *   - DefaultTask (1225112227903) (Entrypoint: 1402488903550)
 *
 * States in plan:
 *   - Tackle (1402488903549)
 *   - GetGoal (1402488910751)
 *   - GetBall (1402488959965)
 *   - TryToDefendGoal (1402489037735)
 *
 * Variables of precondition:
 */
bool PreCondition1402489073613::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1402489071510) ENABLED START*/
    return false;
    /*PROTECTED REGION END*/
}
} // namespace alica
