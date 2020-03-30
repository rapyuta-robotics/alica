#include "Move1542882005838.h"
/*PROTECTED REGION ID(eph1542882005838) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:Move
// Check of RuntimeCondition - (Name): CircleRuntimeCondition, (ConditionString): , (Comment) :

/**
 * Available Vars:
 */
bool RunTimeCondition1543284793605::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1543284793605) ENABLED START*/
    return true;
    /*PROTECTED REGION END*/
}
/**
 * Task: LeaderTask  -> EntryPoint-ID: 1543227886876
 * Task: FollowerTask  -> EntryPoint-ID: 1543227889789
 */
std::shared_ptr<UtilityFunction> UtilityFunction1542882005838::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1542882005838) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}
} // namespace alica
