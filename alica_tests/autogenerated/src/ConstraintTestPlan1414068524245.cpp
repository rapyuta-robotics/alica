#include "ConstraintTestPlan1414068524245.h"
/*PROTECTED REGION ID(eph1414068524245) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:ConstraintTestPlan
// Check of RuntimeCondition - (Name): NewRuntimeCondition, (ConditionString): , (Comment) :

/**
 * Available Vars:
 *	- X (1414068572540)
 *	- Y (1414068576620)
 */
bool RunTimeCondition1414068566297::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1414068566297) ENABLED START*/
    return true;
    /*PROTECTED REGION END*/
}
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1414068524247
 */
std::shared_ptr<UtilityFunction> UtilityFunction1414068524245::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1414068524245) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}
} // namespace alica
