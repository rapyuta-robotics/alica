#include <alica_tests/Defend1402488893641.h>
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
Defend1402488893641::Defend1402488893641(PlanContext& context)
        : DomainPlan(context)
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

/**
 * Task: DefaultTask  -> EntryPoint-ID: 1402488903550
 */

UtilityFunction1402488893641::UtilityFunction1402488893641()
        : BasicUtilityFunction()
{
}

std::shared_ptr<UtilityFunction> UtilityFunction1402488893641::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1402488893641) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1402488893641) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
