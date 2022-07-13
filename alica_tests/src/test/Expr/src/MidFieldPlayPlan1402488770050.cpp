#include <alica_tests/MidFieldPlayPlan1402488770050.h>
/*PROTECTED REGION ID(eph1402488770050) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  MidFieldPlayPlan (1402488770050)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1402488787819)//   - DefaultTask (1225112227903) (Entrypoint: 1402500828244)
//
// States:
//   - Wander (1402488787818)
//   - Tackle (1402489237914)
//   - Sync (1402489273401)
//   - Kill (1402500830885)
//   - Shoot (1402500833246)
MidFieldPlayPlan1402488770050::MidFieldPlayPlan1402488770050(PlanContext& context)
        : DomainPlan(context)
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
bool RunTimeCondition1402489260911::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(1402489260911) ENABLED START*/
    return true;
    /*PROTECTED REGION END*/
}
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1402488787819
 * Task: DefaultTask  -> EntryPoint-ID: 1402500828244
 */

UtilityFunction1402488770050::UtilityFunction1402488770050(IAlicaLogger& logger)
        : BasicUtilityFunction(logger)
{
}

std::shared_ptr<UtilityFunction> UtilityFunction1402488770050::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1402488770050) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan, _logger);
    return defaultFunction;

    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1402488770050) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
