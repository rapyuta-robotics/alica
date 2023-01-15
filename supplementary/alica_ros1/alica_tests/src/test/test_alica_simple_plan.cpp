#include <alica/test/TestContext.h>
#include <alica/test/Util.h>
#include <alica_tests/Behaviour/Attack.h>
#include <alica_tests/TestFixture.h>
#include <gtest/gtest.h>

namespace alica::test
{
/**
 * Tests whether it is possible to run a behaviour in a primitive plan.
 */
TEST_F(TestFixture, runBehaviourInSimplePlan)
{
    // Transition to the plan corresponding to this test case
    ASSERT_TRUE(_tc->setTransitionCond("TestMasterPlan", "ChooseTestState", "SimpleTestPlanState")) << _tc->getLastFailure();
    STEP_UNTIL(_tc, _tc->getActivePlan("SimpleTestPlan"));
    ASSERT_NE(nullptr, _tc->getActivePlan("SimpleTestPlan")) << _tc->getLastFailure();

    // TestState1
    ASSERT_TRUE(_tc->isStateActive("SimpleTestPlan", "TestState1")) << _tc->getLastFailure();
    ASSERT_NE(nullptr, _tc->getActiveBehaviour("MidFieldStandard")) << _tc->getLastFailure();
    ASSERT_EQ(nullptr, _tc->getActiveBehaviour("Attack")) << _tc->getLastFailure();

    // TestState2
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->isStateActive("SimpleTestPlan", "TestState2"));
    ASSERT_EQ(_tc->getActivePlan("SimpleTestPlan")->getName(), "SimpleTestPlan") << _tc->getLastFailure();
    ASSERT_NE(nullptr, _tc->getActiveBehaviour("Attack")) << _tc->getLastFailure();
    ASSERT_EQ(nullptr, _tc->getActiveBehaviour("MidFieldStandard")) << _tc->getLastFailure();

    // Check callcount
    STEP_UNTIL(_tc, dynamic_cast<alica::Attack*>(_tc->getActiveBehaviour("Attack"))->getCallCounter() > 20);
    ASSERT_GT(dynamic_cast<alica::Attack*>(_tc->getActiveBehaviour("Attack"))->getCallCounter(), 20) << _tc->getLastFailure();
    ASSERT_NE(nullptr, _tc->getActiveBehaviour("Attack")) << _tc->getLastFailure();

    return;
}
} // namespace alica::test
