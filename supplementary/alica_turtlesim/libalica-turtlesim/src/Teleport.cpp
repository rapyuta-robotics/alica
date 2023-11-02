
#include "Teleport.h"

#include <alica_turtlesim/turtle_interfaces.hpp>
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
    _startTime = std::chrono::high_resolution_clock::now();
}

void Teleport::run()
{
    if (isSuccess()) {
        return;
    }

    // Delay so as to avoid rapid success in case of repeated calls
    if (std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - _startTime).count() < 1.0) {
        return;
    }

    // teleport turtle to specified coordinates
    auto turtle = alica::LockedBlackboardRW(*getGlobalBlackboard()).get<std::shared_ptr<turtlesim::TurtleInterfaces>>("turtle");

    alica::LockedBlackboardRO bb(*(getBlackboard()));
    turtle->teleport(bb.get<double>("goal_x"), bb.get<double>("goal_y"));
    setSuccess();
}

std::unique_ptr<Teleport> Teleport::create(alica::BehaviourContext& context)
{
    return std::make_unique<Teleport>(context);
}

} /* namespace turtlesim */
