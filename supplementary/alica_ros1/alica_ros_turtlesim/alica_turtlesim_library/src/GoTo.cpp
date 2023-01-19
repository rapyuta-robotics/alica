#include "GoTo.h"
#include "world_model.hpp"
#include <constraintsolver/CGSolver.h>
#include <engine/logging/Logging.h>
#include <memory>

namespace alica
{

GoTo::GoTo(BehaviourContext& context)
        : BasicBehaviour(context)
{
}

GoTo::~GoTo() {}
void GoTo::run()
{
    // solve constraints and get value
    if (!_query.getSolution<reasoner::CGSolver, double>(getPlanContext(), _results)) {
        Logging::logError(LOGNAME) << "Behaviour: " << getName() << ", solution to query not found";
    }
    // move turtle to goal
    std::shared_ptr<turtlesim::ALICATurtleWorldModel> wm =
            LockedBlackboardRW(*getGlobalBlackboard()).get<std::shared_ptr<turtlesim::ALICATurtleWorldModel>>("worldmodel");

    if (wm->turtle.move_toward_goal(_results[0], _results[1])) {
        setSuccess(); // set success if turtle reach goal
    }
}
void GoTo::initialiseParameters()
{
    _query.clearDomainVariables();
    _query.addDomainVariable(getTeamManager().getDomainVariable(getOwnId(), "x"));
    _query.addDomainVariable(getTeamManager().getDomainVariable(getOwnId(), "y"));
}

} /* namespace alica */
