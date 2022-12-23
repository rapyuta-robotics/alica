#include "GoTo.h"
#include "world_model.hpp"
#include <constraintsolver/CGSolver.h>
#include <memory>

namespace alica
{

GoTo::GoTo(BehaviourContext& context)
        : BasicBehaviour(context)
{
    std::cerr << "Debug:"
              << "GoTo created" << std::endl;
}

GoTo::~GoTo() {}
void GoTo::run()
{
    // solve constraints and get value
    if (!_query.getSolution<reasoner::CGSolver, double>(getPlanContext(), _results)) {
        std::cout << getName() << " - Solution to query not found." << std::endl;
        return;
    }
    // move turtle to goal
    LockedBlackboardRW bbwm(getGlobalBlackboard());
    turtlesim::ALICATurtleWorldModel* wm = bbwm.get<std::shared_ptr<turtlesim::ALICATurtleWorldModel>>("worldmodel").get();

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
