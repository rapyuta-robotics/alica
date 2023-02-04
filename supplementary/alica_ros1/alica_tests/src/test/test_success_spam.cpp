#include <alica/test/TestContext.h>
#include <alica/test/Util.h>
#include <alica_tests/TestFixture.h>
#include <gtest/gtest.h>

namespace alica::test
{

/**
 * Tests whether it is possible to run a behaviour in a primitive plan and check for behaviour success
 * and if PlanIsSuccess is success.
 *
 * Steps:
 * 1) Move to Normal state
 * 2) SuccessNormal behaviour is success
 * 3) Move to Dummy state
 * 4) Check if PlanIsSuccess is success.
 *
 */
TEST_F(TestFixture, planIsSuccess)
{
    // Transition to the plan corresponding to this test case
    ASSERT_TRUE(_tc->setTransitionCond("TestMasterPlan", "ChooseTestState", "PlanIsSuccessState")) << _tc->getLastFailure();
    STEP_UNTIL(_tc, _tc->getActivePlan("PlanIsSuccess"));
    ASSERT_NE(nullptr, _tc->getActivePlan("PlanIsSuccess")) << _tc->getLastFailure();

    // Normal state
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->getActivePlan("PlanIsSuccess"));
    ASSERT_NE(nullptr, _tc->getActiveBehaviour("SuccessNormal")) << _tc->getLastFailure();
    ASSERT_TRUE(_tc->isStateActive("PlanIsSuccess", "Normal"));
    ASSERT_TRUE(_tc->isSuccess(_tc->getActiveBehaviour("SuccessNormal")));
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->isSuccess(_tc->getActiveBehaviour("SuccessNormal")));

    // Move to Dummy state
    ASSERT_TRUE(_tc->setTransitionCond("PlanIsSuccess", "Normal", "Dummy")) << _tc->getLastFailure();
    ASSERT_TRUE(_tc->resetTransitionCond("PlanIsSuccess", "Dummy", "Normal")) << _tc->getLastFailure();

    // Plan is Success
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->isSuccess(_tc->getActivePlan("PlanIsSuccess")));
}
} // namespace alica::test
