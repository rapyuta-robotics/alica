#include "GoTo.h"
#include <engine/logging/Logging.h>

#include <memory>

namespace turtlesim
{

GoTo::GoTo(alica::BehaviourContext& context)
        : BasicBehaviour(context)
{
}

void GoTo::run()
{
    if (isSuccess()) {
        return;
    }
    // if (_turtle->moveTowardPosition(_goal_x, _goal_y)) {
    //     alica::Logging::logInfo("GoTo") << "Reached goal";
    //     setSuccess(); // set success if turtle reach goal
    // }
    setSuccess();
}

void GoTo::initialiseParameters()
{
    // _turtle = alica::LockedBlackboardRO(*getGlobalBlackboard()).get<std::shared_ptr<turtlesim::TurtleInterfaces>>("turtle");

    // alica::LockedBlackboardRO bb(*(getBlackboard()));
    // _goal_x = bb.get<double>("goal_x");
    // _goal_y = bb.get<double>("goal_y");
    alica::Logging::logInfo("GoTo") << "Trying to go to (" << _goal_x << "," << _goal_y << ")";
}

GoTo::~GoTo()
{
    alica::Logging::logInfo("GoTo-VEERAJ") << "Getting destroyed now";
}

std::unique_ptr<GoTo> GoTo::create(alica::BehaviourContext& context)
{
    return std::make_unique<GoTo>(context);
}

} // namespace turtlesim
