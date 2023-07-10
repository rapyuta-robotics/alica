#include "RotateTurtle.h"
#include <engine/logging/Logging.h>

#include <memory>

namespace turtlesim
{

RotateTurtle::RotateTurtle(alica::BehaviourContext& context)
        : BasicBehaviour(context)
{
}

void RotateTurtle::run()
{
    if (!_turtle) {
        alica::LockedBlackboardRO g_bb(*getGlobalBlackboard());
        if (!g_bb.hasValue("turtle")) {
            return;
        }
        _turtle = g_bb.get<std::shared_ptr<turtlesim::TurtleInterfaces>>("turtle");
    }
    _turtle->rotate(2);
}

void RotateTurtle::initialiseParameters() {}

std::unique_ptr<RotateTurtle> RotateTurtle::create(alica::BehaviourContext& context)
{
    return std::make_unique<RotateTurtle>(context);
}

} // namespace turtlesim
