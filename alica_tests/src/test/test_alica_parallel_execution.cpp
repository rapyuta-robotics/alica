#include "test_alica.h"
#include <gtest/gtest.h>

namespace alica::test
{

TEST_F(SingleAgentTestFixture, runSameBehaviourInParallel)
{
    // Check if the same behaviour running multiple times in parallel can use different key mappings

    // Transition to the plan corresponding to this test case
    ASSERT_TRUE(_tc->setTransitionCond("TestMasterPlan", "ChooseTestState", "SameInParallelTestState")) << _tc->getLastFailure();
    STEP_UNTIL(_tc, _tc->getActivePlan("SameInParallelTestPlan"));
    ASSERT_TRUE(_tc->setTransitionCond("SameInParallelTestPlan", "ChooseTestState", "SameBehInParallelTestState")) << _tc->getLastFailure();
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->getActivePlan("SameBehInParallelTestPlan")) << _tc->getLastFailure();

    ASSERT_TRUE(_tc->setTransitionCond("SameBehInParallelTestPlan", "EntryState", "FirstCheckState")) << _tc->getLastFailure();

    // The parent retires the plan as soon as it succeeds, so check the parent's success state
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->isStateActive("SameInParallelTestPlan", "SuccessState"));
}

TEST_F(SingleAgentTestFixture, runSamePlanInParallel)
{
    // Check if the same behaviour running multiple times in parallel can use different key mappings

    // Transition to the plan corresponding to this test case
    ASSERT_TRUE(_tc->setTransitionCond("TestMasterPlan", "ChooseTestState", "SameInParallelTestState")) << _tc->getLastFailure();
    STEP_UNTIL(_tc, _tc->getActivePlan("SameInParallelTestPlan"));
    ASSERT_TRUE(_tc->setTransitionCond("SameInParallelTestPlan", "ChooseTestState", "SamePlanInParallelTestState")) << _tc->getLastFailure();
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->getActivePlan("SamePlanInParallelTestPlan")) << _tc->getLastFailure();

    ASSERT_TRUE(_tc->setTransitionCond("SamePlanInParallelTestPlan", "EntryState", "FirstCheckState")) << _tc->getLastFailure();

    // The parent retires the plan as soon as it succeeds, so check the parent's success state
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->isStateActive("SameInParallelTestPlan", "SuccessState"));
}

TEST_F(SingleAgentTestFixture, runSamePlanBehInParallel)
{
    // Check if the same behaviour running multiple times in parallel can use different key mappings

    // Transition to the plan corresponding to this test case
    ASSERT_TRUE(_tc->setTransitionCond("TestMasterPlan", "ChooseTestState", "SameInParallelTestState")) << _tc->getLastFailure();
    STEP_UNTIL(_tc, _tc->getActivePlan("SameInParallelTestPlan"));
    ASSERT_TRUE(_tc->setTransitionCond("SameInParallelTestPlan", "ChooseTestState", "SamePlanBehInParallelTestState")) << _tc->getLastFailure();
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->getActivePlan("SamePlanBehInParallelTestPlan")) << _tc->getLastFailure();

    ASSERT_TRUE(_tc->setTransitionCond("SamePlanBehInParallelTestPlan", "EntryState", "FirstCheckState")) << _tc->getLastFailure();

    // The parent retires the plan as soon as it succeeds, so check the parent's success state
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->isStateActive("SameInParallelTestPlan", "SuccessState"));
}
} // namespace alica::test
