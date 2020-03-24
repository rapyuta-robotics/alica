#include "HandleFailExplicitMaster1530004940652.h"
/*PROTECTED REGION ID(eph1530004940652) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:HandleFailExplicitMaster
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1530004940654
 */
std::shared_ptr<UtilityFunction> UtilityFunction1530004940652::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1530004940652) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}
} // namespace alica
