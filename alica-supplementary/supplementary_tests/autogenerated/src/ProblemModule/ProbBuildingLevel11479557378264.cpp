#include "ProblemModule/ProbBuildingLevel11479557378264.h"
/*PROTECTED REGION ID(eph1479557378264) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:ProbBuildingLevel1
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1479557378266
 */
std::shared_ptr<UtilityFunction> UtilityFunction1479557378264::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1479557378264) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}
} // namespace alica
