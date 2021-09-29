#include "TestBehaviour.h"
#include <memory>
#include <alica_tests/test_sched_world_model.h>

/*PROTECTED REGION ID(inccpp55178365253414982) ENABLED START*/
// Add additional includes here
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(staticVars55178365253414982) ENABLED START*/
// initialise static variables here
/*PROTECTED REGION END*/

TestBehaviour::TestBehaviour()
        : DomainBehaviour("TestBehaviour")
{
    /*PROTECTED REGION ID(con55178365253414982) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
TestBehaviour::~TestBehaviour()
{
    /*PROTECTED REGION ID(dcon55178365253414982) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
void TestBehaviour::run(void* msg)
{
    /*PROTECTED REGION ID(run55178365253414982) ENABLED START*/
    // Add additional options here
    alica_test::SchedWM::instance().executeBehaviourRunCalled = true;
    /*PROTECTED REGION END*/
}
void TestBehaviour::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters55178365253414982) ENABLED START*/
    // Add additional options here

    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods55178365253414982) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

} /* namespace alica */
