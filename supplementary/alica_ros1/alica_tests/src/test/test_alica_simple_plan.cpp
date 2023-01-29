#include <alica/test/TestContext.h>
#include <alica/test/Util.h>
#include <alica_tests/TestFixture.h>
#include <alica_tests/behaviours/State1Behaviour.h>
#include <alica_tests/behaviours/State2Behaviour.h>
#include <gtest/gtest.h>

namespace alica::test
{
/**
 * Tests whether it is possible to run a behaviour in a primitive plan.
 * State1Behaviour return success after 10 counts
 */
TEST_F(TestFixture, runBehaviourInSimplePlan)
{
    // Transition to the plan corresponding to this test case

    ASSERT_TRUE(_tc->setTransitionCond("TestMasterPlan", "ChooseTestState", "RunBehaviourInSimplePlanState")) << _tc->getLastFailure();
    STEP_UNTIL(_tc, _tc->getActivePlan("RunBehaviourInSimplePlan"));
    ASSERT_TRUE(_tc->isStateActive("TestMasterPlan", "RunBehaviourInSimplePlanState")) << _tc->getLastFailure();
    ASSERT_NE(nullptr, _tc->getActivePlan("RunBehaviourInSimplePlan")) << _tc->getLastFailure();

    // TestState1
    ASSERT_TRUE(_tc->isStateActive("RunBehaviourInSimplePlan", "State1")) << _tc->getLastFailure();
    ASSERT_NE(nullptr, _tc->getActiveBehaviour("State1Behaviour")) << _tc->getLastFailure();
    ASSERT_EQ(nullptr, _tc->getActiveBehaviour("State2Behaviour")) << _tc->getLastFailure();

    // TestState2
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->isStateActive("RunBehaviourInSimplePlan", "State2"));
    ASSERT_EQ(_tc->getActivePlan("RunBehaviourInSimplePlan")->getName(), "RunBehaviourInSimplePlan") << _tc->getLastFailure();
    ASSERT_NE(nullptr, _tc->getActiveBehaviour("State2Behaviour")) << _tc->getLastFailure();
    ASSERT_EQ(nullptr, _tc->getActiveBehaviour("State1Behaviour")) << _tc->getLastFailure();

    // Check callcount
    STEP_UNTIL(_tc, dynamic_cast<alica::State2Behaviour*>(_tc->getActiveBehaviour("State2Behaviour"))->getCallCounter() > 20);
    ASSERT_GT(dynamic_cast<alica::State2Behaviour*>(_tc->getActiveBehaviour("State2Behaviour"))->getCallCounter(), 20) << _tc->getLastFailure();
    ASSERT_NE(nullptr, _tc->getActiveBehaviour("State2Behaviour")) << _tc->getLastFailure();

    return;
}
} // namespace alica::test
