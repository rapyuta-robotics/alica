#include "ProblemModule/QueryPlan11479556074049.h"
/*PROTECTED REGION ID(eph1479556074049) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:QueryPlan1
// Check of RuntimeCondition - (Name): NewRuntimeCondition, (ConditionString): , (Comment) :

/**
 * Available Vars:
 *	- QP1X (1479556220234)
 *	- QP1Y (1479556572534)
 */
bool RunTimeCondition1479556084493::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1479556084493) ENABLED START*/
    return true;
    /*PROTECTED REGION END*/
}
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1479556074051
 */
std::shared_ptr<UtilityFunction> UtilityFunction1479556074049::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1479556074049) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}
} // namespace alica
