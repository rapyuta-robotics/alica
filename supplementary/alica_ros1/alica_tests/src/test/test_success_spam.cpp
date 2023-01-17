#include <alica/test/TestContext.h>
#include <alica/test/Util.h>
#include <alica_tests/Behaviour/Attack.h>
#include <alica_tests/SimpleTestPlan1412252439925.h>
#include <alica_tests/TestFixture.h>
#include <gtest/gtest.h>

namespace alica::test
{

/**
 * Tests whether it is possible to run a behaviour in a primitive plan.
 */
TEST_F(TestFixture, runBehaviour)
{
    // Transition to the plan corresponding to this test case
    ASSERT_TRUE(_tc->setTransitionCond("TestMasterPlan", "ChooseTestState", "BehaviorSuccessSpamMasterState")) << _tc->getLastFailure();
    STEP_UNTIL(_tc, _tc->getActivePlan("BehaviorSuccessSpamMaster"));
    ASSERT_NE(nullptr, _tc->getActivePlan("BehaviorSuccessSpamMaster")) << _tc->getLastFailure();

    // Normal state
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->getActivePlan("BehaviorSuccessSpamMaster"));
    ASSERT_NE(nullptr, _tc->getActiveBehaviour("SuccessSpam")) << _tc->getLastFailure();
    ASSERT_TRUE(_tc->isStateActive("BehaviorSuccessSpamMaster", "Normal"));

    // Move from state
    ASSERT_TRUE(_tc->setTransitionCond("BehaviorSuccessSpamMaster", "Normal", "Dummy")) << _tc->getLastFailure();
    ASSERT_TRUE(_tc->resetTransitionCond("BehaviorSuccessSpamMaster", "Dummy", "Normal")) << _tc->getLastFailure();
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->isStateActive("BehaviorSuccessSpamMaster", "Dummy"));
    ASSERT_NE(nullptr, _tc->getActiveBehaviour("SuccessSpam")) << _tc->getLastFailure();
    ASSERT_TRUE(_tc->setTransitionCond("BehaviorSuccessSpamMaster", "Dummy", "Normal")) << _tc->getLastFailure();
    ASSERT_TRUE(_tc->resetTransitionCond("BehaviorSuccessSpamMaster", "Normal", "Dummy")) << _tc->getLastFailure();
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->isStateActive("BehaviorSuccessSpamMaster", "Normal"));
    ASSERT_NE(nullptr, _tc->getActiveBehaviour("SuccessSpam")) << _tc->getLastFailure();
}
} // namespace alica::test
