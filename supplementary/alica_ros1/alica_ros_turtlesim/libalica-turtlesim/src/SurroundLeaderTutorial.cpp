#include "SurroundLeaderTutorial.h"

#include "turtle.hpp"
#include <engine/DefaultUtilityFunction.h>

namespace turtlesim
{

SurroundLeaderTutorial::SurroundLeaderTutorial(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void SurroundLeaderTutorial::onInit()
{
    alica::LockedBlackboardRW bb{*getGlobalBlackboard()};
    if (!bb.hasValue("turtle")) {
        bb.set("turtle", std::make_shared<ALICATurtle>(bb.get<std::string>("agentName")));
    }
}

std::unique_ptr<SurroundLeaderTutorial> SurroundLeaderTutorial::create(alica::PlanContext& context)
{
    return std::make_unique<SurroundLeaderTutorial>(context);
}

std::shared_ptr<alica::UtilityFunction> SurroundLeaderTutorialUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<SurroundLeaderTutorialUtilityFunction> SurroundLeaderTutorialUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<SurroundLeaderTutorialUtilityFunction>();
}

namespace
{

bool isSuccess(const alica::RunningPlan* rp)
{
    if (rp->isBehaviour()) {
        return rp->getStatus() == alica::PlanStatus::Success;
    } else {
        return rp->getActiveState()->isSuccessState();
    }
}

} // namespace

bool TransitionConditions::CustomAnyChildSuccess(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* globalBlackboard)
{
    for (const alica::RunningPlan* child : rp->getChildren()) {
        if (isSuccess(child)) {
            return true;
        }
    }
    return false;
}

} // namespace turtlesim
