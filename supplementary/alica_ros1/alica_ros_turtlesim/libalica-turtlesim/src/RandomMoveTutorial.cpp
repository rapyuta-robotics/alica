#include "RandomMoveTutorial.h"

#include "turtle.hpp"
#include <engine/DefaultUtilityFunction.h>

namespace turtlesim
{

RandomMoveTutorial::RandomMoveTutorial(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void RandomMoveTutorial::onInit()
{
    alica::LockedBlackboardRW bb{*getGlobalBlackboard()};
    if (!bb.hasValue("turtle")) {
        bb.set("turtle", std::make_shared<ALICATurtle>(bb.get<std::string>("agentName")));
    }
}

std::unique_ptr<RandomMoveTutorial> RandomMoveTutorial::create(alica::PlanContext& context)
{
    return std::make_unique<RandomMoveTutorial>(context);
}

} // namespace turtlesim
