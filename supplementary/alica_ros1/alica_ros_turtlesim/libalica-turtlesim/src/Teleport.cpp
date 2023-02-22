
#include "Teleport.h"
#include "turtle.hpp"

#include <memory>
#include <random>

namespace turtlesim
{

Teleport::Teleport(alica::BehaviourContext& context)
        : alica::BasicBehaviour(context)
{
}

void Teleport::initialiseParameters()
{
    // teleport turtle to random place
    auto turtle = alica::LockedBlackboardRW(*getGlobalBlackboard()).get<std::shared_ptr<turtlesim::ALICATurtle>>("turtle");

    alica::LockedBlackboardRO bb(*(getBlackboard()));
    turtle->teleport(bb.get<double>("x"), bb.get<double>("y"));
    setSuccess();
}

std::unique_ptr<Teleport> Teleport::create(alica::BehaviourContext& context)
{
    return std::make_unique<Teleport>(context);
}

} /* namespace turtlesim */
