
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
    // wait for turtle to have a valid position
    if (_turtle->getCurrentPose()) {
        setSuccess();
    }
}

void Teleport::initialiseParameters()
{
    // teleport turtle to random place
    _turtle = alica::LockedBlackboardRW(*getGlobalBlackboard()).get<std::shared_ptr<turtlesim::TurtleInterfaces>>("turtle");
    alica::LockedBlackboardRO bb(*(getBlackboard()));
    _turtle->teleport(bb.get<double>("x"), bb.get<double>("y"));
}

std::unique_ptr<Teleport> Teleport::create(alica::BehaviourContext& context)
{
    return std::make_unique<Teleport>(context);
}

} /* namespace turtlesim */
