#include "GoTo.h"
#include "engine/RunningPlan.h"
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
void GoTo::run(void* msg)
{
    // solve constraints and get value
    if (!_query.getSolution<reasoner::CGSolver, double>(getPlanContext(), _results)) {
        std::cout << getName() << " - Solution to query not found." << std::endl;
        return;
    }
    // move turtle to goal
    LockedBlackboardRW bb(*(getBlackboard()));
    if (!bb.hasValue("turtlesim::worldmodel")) {
        std::cerr << "Errro:Blackboard for GoTo not found" << std::endl;
        return;
    }
    turtlesim::ALICATurtleWorldModel* wm = bb.get<turtlesim::ALICATurtleWorldModel*>("turtlesim::worldmodel");
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
