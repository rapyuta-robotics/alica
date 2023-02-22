#include "TeleportToRandomPosition.h"
#include "turtle.hpp"

#include <memory>
#include <random>

namespace turtlesim
{

TeleportToRandomPosition::TeleportToRandomPosition(alica::BehaviourContext& context)
        : alica::BasicBehaviour(context)
{
}

void TeleportToRandomPosition::initialiseParameters()
{
    alica::LockedBlackboardRW bb(*(getBlackboard()));
    bb.set<double>("xmin", 0.0);
    bb.set<double>("ymin", 0.0);
    bb.set<double>("xmax", 10.0);
    bb.set<double>("ymax", 10.0);
    ROS_INFO_STREAM("initialized location to teleport to");
}

std::unique_ptr<TeleportToRandomPosition> TeleportToRandomPosition::create(alica::BehaviourContext& context)
{
    return std::make_unique<TeleportToRandomPosition>(context);
}

} /* namespace turtlesim */
