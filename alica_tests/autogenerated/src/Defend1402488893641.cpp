#include "Defend1402488893641.h"
/*PROTECTED REGION ID(eph1402488893641) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  Defend (1402488893641)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1402488903550)
//
// States:
//   - Tackle (1402488903549)
//   - GetGoal (1402488910751)
//   - GetBall (1402488959965)
//   - TryToDefendGoal (1402489037735)
Defend1402488893641::Defend1402488893641()
        : DomainPlan()
{
    /*PROTECTED REGION ID(con1402488893641) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
Defend1402488893641::~Defend1402488893641()
{
    /*PROTECTED REGION ID(dcon1402488893641) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

void Defend1402488893641::run(void* msg)
{
    /*PROTECTED REGION ID(runDefend1402488893641) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
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
 * Transition: TackleToGetBall (1402488991762)
 *   - Comment:
 *   - Source2Dest: Tackle --> GetBall
 *
 * Precondition: MISSING_NAME (1402488993122)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in Tackle:
 *   - Tackle (1402488939130)
 *   - Tackle (1402489318663)
 */
bool PreCondition1402488993122::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1402488991762) ENABLED START*/
    return false;
    /*PROTECTED REGION END*/
}
/**
 * Transition: TackleToGetBall (1402488990761)
 *   - Comment:
 *   - Source2Dest: GetBall --> Tackle
 *
 * Precondition: MISSING_NAME (1402488991641)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in GetBall:
 */
bool PreCondition1402488991641::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1402488990761) ENABLED START*/
    return false;
    /*PROTECTED REGION END*/
}
/**
 * Transition: GetBallToTryToDefendGoal (1402489064693)
 *   - Comment:
 *   - Source2Dest: GetBall --> TryToDefendGoal
 *
 * Precondition: MISSING_NAME (1402489065962)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in GetBall:
 */
bool PreCondition1402489065962::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1402489064693) ENABLED START*/
    return false;
    /*PROTECTED REGION END*/
}
/**
 * Transition: TryToDefendGoalToGetGoal (1402489071510)
 *   - Comment:
 *   - Source2Dest: TryToDefendGoal --> GetGoal
 *
 * Precondition: MISSING_NAME (1402489073613)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in TryToDefendGoal:
 *   - PlanType (1402489564599)
 */
bool PreCondition1402489073613::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1402489071510) ENABLED START*/
    return false;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1402488893641) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
