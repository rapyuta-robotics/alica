#include <alica_tests/MasterPlan1402488437260.h>
/*PROTECTED REGION ID(eph1402488437260) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  MasterPlan (1402488437260)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1402488437263)
//
// States:
//   - Attack (1402488437261)
//   - Defend (1402488463437)
//   - Goal (1402488470615)
//   - MidField (1402488477650)
//   - SucGoalState (1402488536570)
MasterPlan1402488437260::MasterPlan1402488437260(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con1402488437260) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
MasterPlan1402488437260::~MasterPlan1402488437260()
{
    /*PROTECTED REGION ID(dcon1402488437260) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 1402488437263
 */

UtilityFunction1402488437260::UtilityFunction1402488437260(IAlicaLogger& logger)
        : BasicUtilityFunction(logger)
{
}

std::shared_ptr<UtilityFunction> UtilityFunction1402488437260::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1402488437260) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan, _logger);
    return defaultFunction;

    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1402488437260) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
