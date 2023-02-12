#include <alica/conditions/conditions.h>

#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>
#include <iostream>

#include <alica_ros2_turtlesim/world_model.hpp>

namespace alica
{
bool conditionMove2Init748720375848597116(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    return turtlesim::ALICATurtleWorldModel::get()->getInit();
}
bool conditionInit2Move974606107671315045(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    return rp->isAnyChildStatus(PlanStatus::Success);
}
bool conditionDefaultCondition2190266318562141841(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    return false;
}
} /* namespace alica */
