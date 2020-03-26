#include "GSolver/GSolverMaster1417423751087.h"
/*PROTECTED REGION ID(eph1417423751087) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:GSolverMaster
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
} // namespace alica
