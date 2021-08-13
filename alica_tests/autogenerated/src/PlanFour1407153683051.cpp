#include "PlanFour1407153683051.h"
/*PROTECTED REGION ID(eph1407153683051) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:PlanFour
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1407153696703
 * Task: AttackTask  -> EntryPoint-ID: 1407153949327
 */
std::shared_ptr<UtilityFunction> UtilityFunction1407153683051::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1407153683051) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}
} // namespace alica
