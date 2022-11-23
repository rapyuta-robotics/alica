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
    std::cerr << "Debug:"
              << "conditionMove2Init" << std::endl;
    LockedBlackboardRO bb(*input);
    if (!bb.hasValue("turtlesim::worldmodel")) {
        std::cerr << "Errro:Blackboard for conditionMove2Init not found" << std::endl;
        return true;
    }
    turtlesim::ALICATurtleWorldModel* wmblack = bb.get<turtlesim::ALICATurtleWorldModel*>("turtlesim::worldmodel");
    return wmblack->getInit();
}
bool conditionInit2Move(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    std::cerr << "Debug:"
              << "conditionInit2Move" << std::endl;
    return rp->isAnyChildStatus(PlanStatus::Success);
}
bool conditionDefaultCondition(const Blackboard* input, const RunningPlan* rp, const IAlicaWorldModel* wm)
{
    std::cerr << "Debug:"
              << "conditionDefaultCondition" << std::endl;
    return false;
}
} /* namespace alica */
