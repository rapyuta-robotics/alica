#include "AttackPlan1402488634525.h"
/*PROTECTED REGION ID(eph1402488634525) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:AttackPlan
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1402488646221
 */
std::shared_ptr<UtilityFunction> UtilityFunction1402488634525::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1402488634525) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: MISSING_NAME, ConditionString: , Comment:
 *
 * Abstract plans in current state:
 *   - Tackle (1402489318663)
 *   - AttackOpp (1402489351885)
 *
 * Tasks in plan:
 *   - DefaultTask (1225112227903) (Entrypoint: 1402488646221)
 *
 * States in plan:
 *   - Attack (1402488646220)
 *   - Shoot (1402489396914)
 *
 * Variables of preconditon:
 *	- ABC (1403772834750)
 *	- TestVar1 (1403772778288)
 */
bool PreCondition1402489460549::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1402489459382) ENABLED START*/
    return false;
    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: ConditionNameShootAttack, ConditionString: Some nice comment!, Comment:
 *
 * Abstract plans in current state:
 *   - Attack (1402488848841)
 *
 * Tasks in plan:
 *   - DefaultTask (1225112227903) (Entrypoint: 1402488646221)
 *
 * States in plan:
 *   - Attack (1402488646220)
 *   - Shoot (1402489396914)
 *
 * Variables of preconditon:
 *	- TestVar1 (1403772778288)
 *	- VarTest2 (1403772797469)
 *	- NewVar (1403772816953)
 *	- ABC (1403772834750)
 */
bool PreCondition1402489462088::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1402489460694) ENABLED START*/
    return false;
    /*PROTECTED REGION END*/
}
} // namespace alica
