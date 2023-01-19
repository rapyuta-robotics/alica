#include "conditions.h"

#include <engine/IAlicaWorldModel.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>
#include <iostream>

#include "world_model.hpp"

namespace alica
{
bool conditionMove2Init(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    return dynamic_cast<const turtlesim::ALICATurtleWorldModel*>(wm)->getInit();
}
bool conditionInit2Move(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    return rp->isAnyChildStatus(PlanStatus::Success);
}
bool conditionDefaultCondition(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    return false;
}
} /* namespace alica */
