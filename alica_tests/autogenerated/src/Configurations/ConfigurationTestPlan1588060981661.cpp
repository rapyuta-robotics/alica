#include "Configurations/ConfigurationTestPlan1588060981661.h"
/*PROTECTED REGION ID(eph1588060981661) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:ConfigurationTestPlan
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1588061024407
 */
std::shared_ptr<UtilityFunction> UtilityFunction1588060981661::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1588060981661) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}
} // namespace alica
