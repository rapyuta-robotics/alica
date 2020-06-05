#include "FailsOnOne1530069246103.h"
/*PROTECTED REGION ID(eph1530069246103) ENABLED START*/
// Add additional using directives here
#include "SimpleSwitches.h"
/*PROTECTED REGION END*/

namespace alica
{
// Plan:FailsOnOne
// Check of RuntimeCondition - (Name): NewRuntimeCondition, (ConditionString): , (Comment) : Is not set 1

/**
 * Available Vars:
 */
bool RunTimeCondition1530069251117::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1530069251117) ENABLED START*/
    return !SimpleSwitches::isSet(1);
    /*PROTECTED REGION END*/
}
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1530069246105
 */
std::shared_ptr<UtilityFunction> UtilityFunction1530069246103::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1530069246103) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}
} // namespace alica
