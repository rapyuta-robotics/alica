#include <alica/test/TestContext.h>
#include <alica/test/Util.h>
#include <alica_tests/Behaviour/Attack.h>
#include <alica_tests/SimpleTestPlan1412252439925.h>
#include <alica_tests/TestFixture.h>
#include <gtest/gtest.h>

namespace alica::test
{

/**
 * Tests whether it is possible to run a behaviour in a primitive plan
 * and the plan at the end is success.
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
    ASSERT_NE(nullptr, _tc->getActiveBehaviour("SuccessSpam")) << _tc->getLastFailure();
    ASSERT_TRUE(_tc->isStateActive("PlanIsSuccess", "Normal"));
    ASSERT_TRUE(_tc->isSuccess(_tc->getActiveBehaviour("SuccessSpam")));
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->isSuccess(_tc->getActiveBehaviour("SuccessSpam")));

    // Move from state
    ASSERT_TRUE(_tc->setTransitionCond("PlanIsSuccess", "Normal", "Dummy")) << _tc->getLastFailure();
    ASSERT_TRUE(_tc->resetTransitionCond("PlanIsSuccess", "Dummy", "Normal")) << _tc->getLastFailure();   
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->isStateActive("PlanIsSuccess", "Dummy"));


    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->isSuccess(_tc->getActiveBehaviour("SuccessSpam")));
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->isSuccess(_tc->getActivePlan("PlanIsSuccess")));
    
    
    //ASSERT_NE(nullptr, _tc->getActiveBehaviour("SuccessSpam")) << _tc->getLastFailure();
    //ASSERT_TRUE(_tc->setTransitionCond("PlanIsSuccess", "Dummy", "Normal")) << _tc->getLastFailure();
    //ASSERT_TRUE(_tc->resetTransitionCond("PlanIsSuccess", "Normal", "Dummy")) << _tc->getLastFailure();
    //STEP_UNTIL_ASSERT_TRUE(_tc, _tc->isStateActive("PlanIsSuccess", "Normal"));
    //ASSERT_NE(nullptr, _tc->getActiveBehaviour("SuccessSpam")) << _tc->getLastFailure();
}
} // namespace alica::test
