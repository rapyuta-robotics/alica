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
        alica::Logging::logWarn(alica::LOGNAME) << "Behaviour: " << getName() << " calculating ...";
        return;
    }
    // move turtle to goal
    auto turtle = alica::LockedBlackboardRO(*getGlobalBlackboard()).get<std::shared_ptr<turtlesim::ALICATurtle>>("turtle");

    if (turtle->moveTowardGoal(_results[0], _results[1])) {
        setSuccess(); // set success if turtle reach goal
    }
}

void GoTo::initialiseParameters()
{
    _query.addDomainVariable(getTeamManager().getDomainVariable(getOwnId(), "x"));
    _query.addDomainVariable(getTeamManager().getDomainVariable(getOwnId(), "y"));
}

void GoTo::onTermination()
{
    _query.clearDomainVariables();
}

std::unique_ptr<GoTo> GoTo::create(alica::BehaviourContext& context)
{
    return std::make_unique<GoTo>(context);
}

} // namespace turtlesim
