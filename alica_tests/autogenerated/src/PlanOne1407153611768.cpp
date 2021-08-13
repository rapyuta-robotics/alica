#include "PlanOne1407153611768.h"
/*PROTECTED REGION ID(eph1407153611768) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:PlanOne
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1407153636262
 * Task: AttackTask  -> EntryPoint-ID: 1407153791141
 */
std::shared_ptr<UtilityFunction> UtilityFunction1407153611768::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1407153611768) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}
} // namespace alica
