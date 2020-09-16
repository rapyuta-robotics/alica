#include "VariableHandling/Lvl21524452793378.h"
/*PROTECTED REGION ID(eph1524452793378) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:Lvl2
// Check of RuntimeCondition - (Name): NewRuntimeCondition, (ConditionString): Lvl2 Runtime Condition, (Comment) :

/**
 * Available Vars:
 *	- L2A (1524453150187)
 *	- L2B (1524453155043)
 *	- L2C (1524453162883)
 */
bool RunTimeCondition1524453266123::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1524453266123) ENABLED START*/
    return true;
    /*PROTECTED REGION END*/
}
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1524452793380
 * Task: AttackTask  -> EntryPoint-ID: 1524453238753
 */
std::shared_ptr<UtilityFunction> UtilityFunction1524452793378::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1524452793378) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}
} // namespace alica
