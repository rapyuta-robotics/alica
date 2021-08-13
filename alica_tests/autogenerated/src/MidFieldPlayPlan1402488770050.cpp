#include "MidFieldPlayPlan1402488770050.h"
/*PROTECTED REGION ID(eph1402488770050) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:MidFieldPlayPlan1402488770050
MidFieldPlayPlan1402488770050::MidFieldPlayPlan1402488770050()
        : DomainPlan()
{
    /*PROTECTED REGION ID(con1402488770050) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
MidFieldPlayPlan1402488770050::~MidFieldPlayPlan1402488770050()
{
    /*PROTECTED REGION ID(dcon1402488770050) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
// Check of RuntimeCondition - (Name): NewRuntimeCondition, (ConditionString): , (Comment) :

/**
 * Available Vars:
 */
bool RunTimeCondition1402489260911::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1402489260911) ENABLED START*/
    return true;
    /*PROTECTED REGION END*/
}
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1402488787819
 * Task: DefaultTask  -> EntryPoint-ID: 1402500828244
 */
std::shared_ptr<UtilityFunction> UtilityFunction1402488770050::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1402488770050) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: MISSING_NAME, ConditionString: , Comment:
 *
 * Abstract plans in current state:
 *   - MidFieldStandard (1402488696205)
 *
 * Tasks in plan:
 *   - DefaultTask (1225112227903) (Entrypoint: 1402488787819)*   - DefaultTask (1225112227903) (Entrypoint: 1402500828244)
 *
 * States in plan:
 *   - Wander (1402488787818)
 *   - Tackle (1402489237914)
 *   - Sync (1402489273401)
 *   - Kill (1402500830885)
 *   - Shoot (1402500833246)
 *
 * Variables of precondition:
 */
bool PreCondition1402489258509::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1402489257607) ENABLED START*/
    return false;
    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: MISSING_NAME, ConditionString: , Comment:
 *
 * Abstract plans in current state:
 *   - MidFieldStandard (1402488696205)
 *
 * Tasks in plan:
 *   - DefaultTask (1225112227903) (Entrypoint: 1402488787819)*   - DefaultTask (1225112227903) (Entrypoint: 1402500828244)
 *
 * States in plan:
 *   - Wander (1402488787818)
 *   - Tackle (1402489237914)
 *   - Sync (1402489273401)
 *   - Kill (1402500830885)
 *   - Shoot (1402500833246)
 *
 * Variables of precondition:
 */
bool PreCondition1402489278408::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1402489276995) ENABLED START*/
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
 *   - DefaultTask (1225112227903) (Entrypoint: 1402488787819)*   - DefaultTask (1225112227903) (Entrypoint: 1402500828244)
 *
 * States in plan:
 *   - Wander (1402488787818)
 *   - Tackle (1402489237914)
 *   - Sync (1402489273401)
 *   - Kill (1402500830885)
 *   - Shoot (1402500833246)
 *
 * Variables of precondition:
 */
bool PreCondition1402500844446::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1402500843072) ENABLED START*/
    return false;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1402488770050) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
