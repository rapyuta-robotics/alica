#include "PlanFive1407153703092.h"
/*PROTECTED REGION ID(eph1407153703092) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:PlanFive
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1407153717809
 * Task: AttackTask  -> EntryPoint-ID: 1407153972059
 * Task: DefendTask  -> EntryPoint-ID: 1407153973706
 * Task: MidFieldTask  -> EntryPoint-ID: 1407153975075
 */
std::shared_ptr<UtilityFunction> UtilityFunction1407153703092::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1407153703092) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}
} // namespace alica
