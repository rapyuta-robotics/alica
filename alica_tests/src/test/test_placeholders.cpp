#include <test_alica.h>

#include <gtest/gtest.h>

namespace alica::test
{

TEST_F(SingleAgentTestFixture, placeholdersTest)
{
    // Checks if placeholders are replaced by their corresponding implementations as specified in the mapping json

    // Transition to the plan corresponding to this test case
    ASSERT_TRUE(_tc->setTransitionCond("TestMasterPlan", "ChooseTestState", "PlaceholderTestState")) << _tc->getLastFailure();
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->getActivePlan("PlaceholderTestPlan")) << _tc->getLastFailure();
    auto testPlan = _tc->getActivePlan("PlaceholderTestPlan");

    // Check if the plan placeholder is replaced correctly
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->getActivePlan("PlaceholderPlanImpl")) << _tc->getLastFailure();
    // Check if the blackboard & key mapping is working correctly
    auto plan = _tc->getActivePlan("PlaceholderPlanImpl");
    LockedBlackboardRO planbb(*plan->getBlackboard());
    ASSERT_EQ(3, planbb.get<int64_t>("plan_key"));

    // Check if the nested behaviour placeholder is replaced correctly
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->getActiveBehaviour("PlaceholderBehImpl")) << _tc->getLastFailure();
    // Check if the blackboard & key mapping is working correctly
    auto beh = _tc->getActiveBehaviour("PlaceholderBehImpl");
    LockedBlackboardRO behbb(*beh->getBlackboard());
    ASSERT_EQ(3, behbb.get<int64_t>("beh_key"));

    // Check the plan heirarchy is as expected
    ASSERT_EQ(beh->getPlanContext()->getParent(), plan->getPlanContext());
    ASSERT_EQ(plan->getPlanContext()->getParent(), testPlan->getPlanContext());
}

} // namespace alica::test
