#include "RealMasterPlanForSyncTest1418902217839.h"
/*PROTECTED REGION ID(eph1418902217839) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:RealMasterPlanForSyncTest
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1418902217841
 */
std::shared_ptr<UtilityFunction> UtilityFunction1418902217839::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1418902217839) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}
} // namespace alica
