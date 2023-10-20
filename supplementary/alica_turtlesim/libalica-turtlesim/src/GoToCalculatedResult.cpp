#include "GoToCalculatedResult.h"

#include <constraintsolver/CGSolver.h>
#include <engine/logging/Logging.h>

#include <memory>

namespace turtlesim
{

GoToCalculatedResult::GoToCalculatedResult(alica::BehaviourContext& context)
        : BasicBehaviour(context)
{
}

void GoToCalculatedResult::run()
{
    // solve constraints and get value
    if (!_query.getSolution<alica::reasoner::CGSolver, double>(getPlanContext(), _results)) {
        alica::Logging::logWarn(alica::LOGNAME) << "Behaviour: " << getName() << " calculating ...";
        return;
    }

    if (_turtle->moveTowardPosition(_results[0], _results[1])) {
        setSuccess(); // set success if turtle reach goal
    }
}

void GoToCalculatedResult::initialiseParameters()
{
    _turtle = alica::LockedBlackboardRO(*getGlobalBlackboard()).get<std::shared_ptr<turtlesim::TurtleInterfaces>>("turtle");
    _query.addDomainVariable(getTeamManager().getDomainVariable(getOwnId(), "x"));
    _query.addDomainVariable(getTeamManager().getDomainVariable(getOwnId(), "y"));
}

void GoToCalculatedResult::onTermination()
{
    _query.clearDomainVariables();
}

std::unique_ptr<GoToCalculatedResult> GoToCalculatedResult::create(alica::BehaviourContext& context)
{
    return std::make_unique<GoToCalculatedResult>(context);
}

} // namespace turtlesim
