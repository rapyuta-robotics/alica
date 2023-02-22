#include "conditions.h"

#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include "world_model.hpp"

namespace turtlesim
{
bool conditionMove2Init(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    return turtlesim::ALICATurtleWorldModel::get()->getInit();
}
bool conditionInit2Move(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    return rp->isAnyChildStatus(alica::PlanStatus::Success);
}
bool conditionDefaultCondition(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    return false;
}
} /* namespace turtlesim */
