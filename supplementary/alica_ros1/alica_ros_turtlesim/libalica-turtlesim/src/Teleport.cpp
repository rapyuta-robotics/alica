
#include "Teleport.h"

#include <memory>
#include <random>

namespace turtlesim
{

Teleport::Teleport(alica::BehaviourContext& context)
        : alica::BasicBehaviour(context)
{
}

void Teleport::run()
{
    if (isSuccess() || isFailure()) {
        return;
    }
    // wait for turtle to teleport to given location
    if (_turtle->getCurrentPose()) {
        auto pose = _turtle->getCurrentPose();
        auto dx = pose->x - _x;
        auto dy = pose->y - _y;
        if (dx * dx + dy * dy <= 0.01) {
            setSuccess();
        }
    }
}

void Teleport::initialiseParameters()
{
    // teleport turtle to random place
    _turtle = alica::LockedBlackboardRW(*getGlobalBlackboard()).get<std::shared_ptr<turtlesim::TurtleInterfaces>>("turtle");
    alica::LockedBlackboardRO bb(*(getBlackboard()));
    _x = bb.get<double>("x");
    _y = bb.get<double>("y");
    _turtle->teleport(_x, _y);
}

std::unique_ptr<Teleport> Teleport::create(alica::BehaviourContext& context)
{
    return std::make_unique<Teleport>(context);
}

} /* namespace turtlesim */
