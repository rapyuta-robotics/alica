#include "VariableHandling/VHMaster1524452721452.h"
/*PROTECTED REGION ID(eph1524452721452) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:VHMaster
// Check of RuntimeCondition - (Name): NewRuntimeCondition, (ConditionString): VHMaster Runtime Condition, (Comment) : Unrelated Condition

/**
 * Available Vars:
 *	- MA (1524463022262)
 *	- MB (1524463028066)
 */
bool RunTimeCondition1524463006078::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1524463006078) ENABLED START*/
    return true;
    /*PROTECTED REGION END*/
}
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1524452721454
 */
std::shared_ptr<UtilityFunction> UtilityFunction1524452721452::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1524452721452) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}
} // namespace alica
