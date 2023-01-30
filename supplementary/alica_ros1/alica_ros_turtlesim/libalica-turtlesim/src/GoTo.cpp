#include "GoTo.h"
#include "turtle.hpp"

#include <constraintsolver/CGSolver.h>
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
    // solve constraints and get value
    if (!_query.getSolution<alica::reasoner::CGSolver, double>(getPlanContext(), _results)) {
        alica::Logging::logError(alica::LOGNAME) << "Behaviour: " << getName() << ", solution to query not found";
    }
    // move turtle to goal
    auto turtle = alica::LockedBlackboardRW(*getGlobalBlackboard()).get<std::shared_ptr<turtlesim::ALICATurtle>>("turtle");

    if (turtle->move_toward_goal(_results[0], _results[1])) {
        setSuccess(); // set success if turtle reach goal
    }
}

void GoTo::initialiseParameters()
{
    _query.clearDomainVariables();
    _query.addDomainVariable(getTeamManager().getDomainVariable(getOwnId(), "x"));
    _query.addDomainVariable(getTeamManager().getDomainVariable(getOwnId(), "y"));
}

std::unique_ptr<GoTo> GoTo::create(alica::BehaviourContext& context)
{
    return std::make_unique<GoTo>(context);
}

} // namespace turtlesim
