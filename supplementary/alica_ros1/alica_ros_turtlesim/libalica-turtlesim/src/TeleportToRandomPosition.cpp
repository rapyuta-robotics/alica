#include "TeleportToRandomPosition.h"
#include "turtle_interfaces.hpp"

#include <memory>
#include <random>

namespace turtlesim
{

TeleportToRandomPosition::TeleportToRandomPosition(alica::PlanContext& context)
        : alica::BasicPlan(context)
{
}

void TeleportToRandomPosition::onInit()
{
    alica::LockedBlackboardRW bb(*(getBlackboard()));
    bb.set<double>("xmin", 0.0);
    bb.set<double>("ymin", 0.0);
    bb.set<double>("xmax", 10.0);
    bb.set<double>("ymax", 10.0);
}

std::unique_ptr<TeleportToRandomPosition> TeleportToRandomPosition::create(alica::PlanContext& context)
{
    return std::make_unique<TeleportToRandomPosition>(context);
}

} /* namespace turtlesim */
