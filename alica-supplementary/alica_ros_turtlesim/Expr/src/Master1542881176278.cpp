#include "Master1542881176278.h"
/*PROTECTED REGION ID(eph1542881176278) ENABLED START*/
// Add additional using directives here
#include <alica_ros_turtlesim/world_model.hpp>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:Master
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1543227864154
 */
std::shared_ptr<UtilityFunction> UtilityFunction1542881176278::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1542881176278) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: MISSING_NAME, ConditionString: , Comment:
 *
 * Abstract plans in current state:
 *   - Go2RandomPosition (1542881969548)
 *
 * Tasks in plan:
 *   - DefaultTask (1542881176318) (Entrypoint: 1543227864154)
 *
 * States in plan:
 *   - Init (1542881176280)
 *   - Move (1542881580237)
 *
 * Variables of preconditon:
 */
bool PreCondition1542881647180::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1542881645594) ENABLED START*/
    return rp->isAnyChildStatus(PlanStatus::Success);
    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: MISSING_NAME, ConditionString: , Comment: Transition from Move to Init
 *
 * Abstract plans in current state:
 *   - Move (1542882005838)
 *
 * Tasks in plan:
 *   - DefaultTask (1542881176318) (Entrypoint: 1543227864154)
 *
 * States in plan:
 *   - Init (1542881176280)
 *   - Move (1542881580237)
 *
 * Variables of preconditon:
 */
bool PreCondition1542881650423::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1542881648973) ENABLED START*/
    // transition happen if init flag is true in world model.
    return turtlesim::ALICATurtleWorldModel::get()->getInit();
    /*PROTECTED REGION END*/
}
} // namespace alica
