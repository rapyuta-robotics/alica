#include <alica/conditions/conditions.h>

#include <engine/IAlicaWorldModel.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>
#include <iostream>

/*PROTECTED REGION ID(conditionSource) ENABLED START*/
// Add additional options here
#include <alica_ros2_turtlesim/world_model.hpp>
/*PROTECTED REGION END*/

namespace alica
{
bool conditionMove2Init748720375848597116(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition748720375848597116) ENABLED START*/
    return turtlesim::ALICATurtleWorldModel::get()->getInit();
    /*PROTECTED REGION END*/
}
bool conditionInit2Move974606107671315045(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition974606107671315045) ENABLED START*/
    return rp->isAnyChildStatus(PlanStatus::Success);
    /*PROTECTED REGION END*/
}
bool conditionDefaultCondition2190266318562141841(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    /*PROTECTED REGION ID(condition2190266318562141841) ENABLED START*/
    return false;
    /*PROTECTED REGION END*/
}
} /* namespace alica */
