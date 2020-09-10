#include "GSolver/GSolverTestPlan1417423757243.h"
/*PROTECTED REGION ID(eph1417423757243) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:GSolverTestPlan
// Check of RuntimeCondition - (Name): NewRuntimeCondition, (ConditionString): , (Comment) :

/**
 * Available Vars:
 *	- X (1417444589341)
 *	- Y (1417444593509)
 */
bool RunTimeCondition1417424512343::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1417424512343) ENABLED START*/
    return true;
    /*PROTECTED REGION END*/
}
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1417423777546
 */
std::shared_ptr<UtilityFunction> UtilityFunction1417423757243::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1417423757243) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}
} // namespace alica
