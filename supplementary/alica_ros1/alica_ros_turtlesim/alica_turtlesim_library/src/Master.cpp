#include <engine/DefaultUtilityFunction.h>

#include "Master.h"
#include "world_model.hpp"

namespace alica
{

Master::Master(PlanContext& context)
        : BasicPlan(context)
{
}
Master::~Master() {}

void Master::onInit() {}

std::shared_ptr<UtilityFunction> MasterUtilityFunction::getUtilityFunction(Plan* plan)
{
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
}

bool conditionMove2Init(const Blackboard* input, const RunningPlan* rp, const Blackboard* globalBlackboard)
{
    std::shared_ptr<turtlesim::ALICATurtleWorldModel> wm =
            LockedBlackboardRO(*globalBlackboard).get<std::shared_ptr<turtlesim::ALICATurtleWorldModel>>("worldmodel");
    return wm->getInit();
}

bool conditionInit2Move(const Blackboard* input, const RunningPlan* rp, const Blackboard* globalBlackboard)
{
    return rp->isAnyChildStatus(PlanStatus::Success);
}

} // namespace alica
