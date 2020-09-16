#include "ProblemModule/ProblemBuildingMaster1479556022226.h"
/*PROTECTED REGION ID(eph1479556022226) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:ProblemBuildingMaster
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1479556022228
 */
std::shared_ptr<UtilityFunction> UtilityFunction1479556022226::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1479556022226) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: MISSING_NAME, ConditionString: , Comment:
 *
 * Abstract plans in current state:
 *   - ProbBuildingLevel1 (1479557378264)
 *
 * Tasks in plan:
 *   - DefaultTask (1225112227903) (Entrypoint: 1479556022228)
 *
 * States in plan:
 *   - State1 (1479556022227)
 *   - State2 (1479557585252)
 *
 * Variables of preconditon:
 *	- PBMX (1479557337956)
 *	- PBMY (1479557345903)
 */
bool PreCondition1479557592662::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1479557591331) ENABLED START*/
    return false;
    /*PROTECTED REGION END*/
}
} // namespace alica
