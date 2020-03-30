#include "Behaviours/GoTo.h"
#include <memory>

/*PROTECTED REGION ID(inccpp1544160969061) ENABLED START*/
// Add additional includes here
#include <alica_ros_turtlesim/world_model.hpp>
#include <constraintsolver/CGSolver.h>
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(staticVars1544160969061) ENABLED START*/
// initialise static variables here
/*PROTECTED REGION END*/

GoTo::GoTo()
        : DomainBehaviour("GoTo")
{
    /*PROTECTED REGION ID(con1544160969061) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
GoTo::~GoTo()
{
    /*PROTECTED REGION ID(dcon1544160969061) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
void GoTo::run(void* msg)
{
    /*PROTECTED REGION ID(run1544160969061) ENABLED START*/
    // Add additional options here

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
    /*PROTECTED REGION ID(initialiseParameters1544160969061) ENABLED START*/
    // Add additional options here
    // add variables to query which is define in editor->Runtime condition->Quantifiers
    _query.clearDomainVariables();
    _query.addDomainVariable(getOwnId(), "x", getEngine());
    _query.addDomainVariable(getOwnId(), "y", getEngine());

    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1544160969061) ENABLED START*/
// Add additional methods here
/*PROTECTED REGION END*/

} /* namespace alica */
