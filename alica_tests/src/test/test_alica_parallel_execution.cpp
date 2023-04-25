#include "test_alica.h"
#include <gtest/gtest.h>

namespace alica::test
{

TEST_F(SingleAgentTestFixture, runSameBehaviourInParallel)
{
    // Check if the same behaviour running multiple times in parallel can use different key mappings

    // Transition to the plan corresponding to this test case
    ASSERT_TRUE(_tc->setTransitionCond("TestMasterPlan", "ChooseTestState", "SameBehaviourInParallelTestState")) << _tc->getLastFailure();
    STEP_UNTIL(_tc, _tc->getActivePlan("SameBehaviourInParallelTestPlan"));
    auto plan = _tc->getActivePlan("SameBehaviourInParallelTestPlan");
    ASSERT_NE(plan, nullptr) << _tc->getLastFailure();

    ASSERT_TRUE(_tc->setTransitionCond("SameBehaviourInParallelTestPlan", "EntryState", "FirstCheckState")) << _tc->getLastFailure();

    // Step until the plan succeeds
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->isSuccess(plan));
}

TEST_F(SingleAgentTestFixture, runSamePlanInParallel)
{
    // Check if the same behaviour running multiple times in parallel can use different key mappings

    // Transition to the plan corresponding to this test case
    ASSERT_TRUE(_tc->setTransitionCond("TestMasterPlan", "ChooseTestState", "SamePlanInParallelTestState")) << _tc->getLastFailure();
    STEP_UNTIL(_tc, _tc->getActivePlan("SamePlanInParallelTestPlan"));
    auto plan = _tc->getActivePlan("SamePlanInParallelTestPlan");
    ASSERT_NE(plan, nullptr) << _tc->getLastFailure();

    ASSERT_TRUE(_tc->setTransitionCond("SamePlanInParallelTestPlan", "EntryState", "FirstCheckState")) << _tc->getLastFailure();

    // Step until the plan succeeds
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->isSuccess(plan));
}

TEST_F(SingleAgentTestFixture, runSamePlanBehInParallel)
{
    // Check if the same behaviour running multiple times in parallel can use different key mappings

    // Transition to the plan corresponding to this test case
    ASSERT_TRUE(_tc->setTransitionCond("TestMasterPlan", "ChooseTestState", "SamePlanBehaviourInParallelTestState")) << _tc->getLastFailure();
    STEP_UNTIL(_tc, _tc->getActivePlan("SamePlanBehInParallelTestPlan"));
    auto plan = _tc->getActivePlan("SamePlanBehInParallelTestPlan");
    ASSERT_NE(plan, nullptr) << _tc->getLastFailure();

    ASSERT_TRUE(_tc->setTransitionCond("SamePlanBehInParallelTestPlan", "EntryState", "FirstCheckState")) << _tc->getLastFailure();

    // Step until the plan succeeds
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->isSuccess(plan));
}
} // namespace alica::test
