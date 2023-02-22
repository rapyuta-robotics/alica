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
    if (_turtle->moveTowardGoal(_goal_x, _goal_y)) {
        setSuccess(); // set success if turtle reach goal
    }
}

void GoTo::initialiseParameters()
{
    _turtle = alica::LockedBlackboardRO(*getGlobalBlackboard()).get<std::shared_ptr<turtlesim::ALICATurtle>>("turtle");

    alica::LockedBlackboardRO bb(*(getBlackboard()));
    _goal_x = bb.get<double>("goal_x");
    _goal_y = bb.get<double>("goal_y");
    alica::Logging::logInfo("GoTo") << "Trying to go to (" << _goal_x << "," << _goal_y << ")";
}

std::unique_ptr<GoTo> GoTo::create(alica::BehaviourContext& context)
{
    return std::make_unique<GoTo>(context);
}

} // namespace turtlesim
