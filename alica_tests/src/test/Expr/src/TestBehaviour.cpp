#include "TestBehaviour.h"
#include <memory>

/*PROTECTED REGION ID(inccpp55178365253414982) ENABLED START*/
// Add additional includes here
#include <alica_tests/test_sched_world_model.h>
/*PROTECTED REGION END*/

namespace alica
{
/*PROTECTED REGION ID(staticVars55178365253414982) ENABLED START*/
// initialise static variables here
/*PROTECTED REGION END*/

TestBehaviour::TestBehaviour(IAlicaWorldModel* wm)
        : DomainBehaviour(wm, "TestBehaviour")
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
    auto* wm = dynamic_cast<alica_test::SchedWM*>(getWorldModel());
    if (wm->executeBehaviourRunCalled) {
        return;
    }
    wm->execOrder += "TestBehaviour::Run\n";
    wm->executeBehaviourRunCalled = true;
    /*PROTECTED REGION END*/
}
void TestBehaviour::initialiseParameters()
{
    /*PROTECTED REGION ID(initialiseParameters55178365253414982) ENABLED START*/
    // Add additional options here
    auto* wm = dynamic_cast<alica_test::SchedWM*>(getWorldModel());
    wm->execOrder += "TestBehaviour::Init\n";
    wm->executeBehaviourRunCalled = false;

    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods55178365253414982) ENABLED START*/
// Add additional options here

void TestBehaviour::onTermination()
{
    auto* wm = dynamic_cast<alica_test::SchedWM*>(getWorldModel());
    wm->execOrder += "TestBehaviour::Term\n";
}
/*PROTECTED REGION END*/

} /* namespace alica */
