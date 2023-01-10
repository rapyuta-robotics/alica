#include "conditions.h"

#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>
#include <iostream>

#include "world_model.hpp"

namespace alica
{
bool conditionMove2Init(const Blackboard* input, const RunningPlan* rp, const Blackboard* globalBlackboard)
{
    std::cerr << "Debug:"
              << "conditionMove2Init" << std::endl;
    std::shared_ptr<turtlesim::ALICATurtleWorldModel> wm =
            LockedBlackboardRO(*globalBlackboard).get<std::shared_ptr<turtlesim::ALICATurtleWorldModel>>("worldmodel");
    return wm->getInit();
}
bool conditionInit2Move(const Blackboard* input, const RunningPlan* rp, const Blackboard* globalBlackboard)
{
    std::cerr << "Debug:"
              << "conditionInit2Move" << std::endl;
    return rp->isAnyChildStatus(PlanStatus::Success);
}
bool conditionDefaultCondition(const Blackboard* input, const RunningPlan* rp, const Blackboard* globalBlackboard)
{
    std::cerr << "Debug:"
              << "conditionDefaultCondition" << std::endl;
    return false;
}
} /* namespace alica */
