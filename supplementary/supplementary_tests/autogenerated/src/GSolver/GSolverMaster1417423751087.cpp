#include "GSolver/GSolverMaster1417423751087.h"
/*PROTECTED REGION ID(eph1417423751087) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  GSolverMaster (1417423751087)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1417423751089)
//
// States:
//   - Init (1417423751088)
GSolverMaster1417423751087::GSolverMaster1417423751087(IAlicaWorldModel* wm)
        : DomainPlan(wm)
{
    /*PROTECTED REGION ID(con1417423751087) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
GSolverMaster1417423751087::~GSolverMaster1417423751087()
{
    /*PROTECTED REGION ID(dcon1417423751087) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 1417423751089
 */
std::shared_ptr<UtilityFunction> UtilityFunction1417423751087::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1417423751087) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1417423751087) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
