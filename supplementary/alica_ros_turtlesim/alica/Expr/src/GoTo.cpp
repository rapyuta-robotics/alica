#include "GoTo.h"
#include <memory>

/*PROTECTED REGION ID(inccpp4054297592460872311) ENABLED START*/
// Add additional includes here
#include <alica_ros_turtlesim/world_model.hpp>
#include <constraintsolver/CGSolver.h>
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(staticVars4054297592460872311) ENABLED START*/
// initialise static variables here
/*PROTECTED REGION END*/

GoTo::GoTo(IAlicaWorldModel* wm)
        : DomainBehaviour(wm, "GoTo")
{
    /*PROTECTED REGION ID(con4054297592460872311) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
GoTo::~GoTo()
{
    /*PROTECTED REGION ID(dcon4054297592460872311) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
void GoTo::run(void* msg)
{
    /*PROTECTED REGION ID(run4054297592460872311) ENABLED START*/
    // solve constraints and get value
    if (!_query.getSolution<reasoner::CGSolver, double>(getPlanContext(), _results)) {
        std::cout << getName() << " - Solution to query not found." << std::endl;
        return;
    }
    // move turtle to goal
    if (turtlesim::ALICATurtleWorldModel::get()->turtle.move_toward_goal(_results[0], _results[1])) {
        setSuccess(); // set success if turtle reach goal
    }
    /*PROTECTED REGION END*/
}
void GoTo::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters4054297592460872311) ENABLED START*/
    // Add additional options here
    _query.clearDomainVariables();
    _query.addDomainVariable(getEngine()->getTeamManager().getDomainVariable(getOwnId(), "x"));
    _query.addDomainVariable(getEngine()->getTeamManager().getDomainVariable(getOwnId(), "y"));

    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods4054297592460872311) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

} /* namespace alica */
