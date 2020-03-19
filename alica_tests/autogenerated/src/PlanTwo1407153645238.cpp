#include "PlanTwo1407153645238.h"
/*PROTECTED REGION ID(eph1407153645238) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:PlanTwo
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1407153656782
 * Task: AttackTask  -> EntryPoint-ID: 1407153821287
 * Task: DefendTask  -> EntryPoint-ID: 1407153842648
 */
std::shared_ptr<UtilityFunction> UtilityFunction1407153645238::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1407153645238) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}
} // namespace alica
