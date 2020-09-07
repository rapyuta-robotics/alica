#include "MasterPlanTaskAssignment1407152758497.h"
/*PROTECTED REGION ID(eph1407152758497) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:MasterPlanTaskAssignment
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1407152758499
 * Task: AttackTask  -> EntryPoint-ID: 1407152894887
 * Task: MidFieldTask  -> EntryPoint-ID: 1407152900425
 * Task: DefendTask  -> EntryPoint-ID: 1407152902493
 */
std::shared_ptr<UtilityFunction> UtilityFunction1407152758497::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1407152758497) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}
} // namespace alica
