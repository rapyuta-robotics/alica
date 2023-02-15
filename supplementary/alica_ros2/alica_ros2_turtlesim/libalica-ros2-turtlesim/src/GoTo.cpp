#include "GoTo.h"
#include <memory>

#include "world_model.hpp"
#include <constraintsolver/CGSolver.h>

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
        std::cout << getName() << " - Solution to query not found." << std::endl;
        return;
    }
    // move turtle to goal
    if (turtlesim::ALICATurtleWorldModel::get()->turtle.move_toward_goal(_results[0], _results[1])) {
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

} /* namespace alica */
