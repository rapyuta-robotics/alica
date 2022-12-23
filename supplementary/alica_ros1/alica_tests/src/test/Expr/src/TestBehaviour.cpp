#include <alica_tests/TestBehaviour.h>
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

TestBehaviour::TestBehaviour(BehaviourContext& context)
        : DomainBehaviour(context)
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
void TestBehaviour::run()
{
    /*PROTECTED REGION ID(run55178365253414982) ENABLED START*/
    // Add additional options here
    LockedBlackboardRW bbwm(getGlobalBlackboard());
    auto* wm = bbwm.get<std::shared_ptr<alica_test::SchedWM>>("worldmodel").get();

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
    LockedBlackboardRW bbwm(getGlobalBlackboard());
    auto* wm = bbwm.get<std::shared_ptr<alica_test::SchedWM>>("worldmodel").get();
    wm->execOrder += "TestBehaviour::Init\n";
    wm->executeBehaviourRunCalled = false;

    /*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods55178365253414982) ENABLED START*/
// Add additional options here

void TestBehaviour::onTermination()
{
    LockedBlackboardRW bbwm(getGlobalBlackboard());
    auto* wm = bbwm.get<std::shared_ptr<alica_test::SchedWM>>("worldmodel").get();
    wm->execOrder += "TestBehaviour::Term\n";
}
/*PROTECTED REGION END*/

} /* namespace alica */
