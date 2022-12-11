#include "conditions.h"

#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>
#include <iostream>

#include "world_model.hpp"

namespace alica
{
bool conditionMove2Init(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels)
{
    std::cerr << "Debug:"
              << "conditionMove2Init" << std::endl;
    BlackboardImpl& impl = const_cast<BlackboardImpl&>(worldModels->impl()); // todo luca remove cast
    turtlesim::ALICATurtleWorldModel* wm = impl.getWorldModel<turtlesim::ALICATurtleWorldModel>("worldModel");
    return wm->getInit();
}
bool conditionInit2Move(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels)
{
    std::cerr << "Debug:"
              << "conditionInit2Move" << std::endl;
    return rp->isAnyChildStatus(PlanStatus::Success);
}
bool conditionDefaultCondition(const Blackboard* input, const RunningPlan* rp, const Blackboard* worldModels)
{
    std::cerr << "Debug:"
              << "conditionDefaultCondition" << std::endl;
    return false;
}
} /* namespace alica */
