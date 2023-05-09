#include "test_alica.h"
#include <alica/test/CounterClass.h>
#include <alica_tests/behaviours/BehAAA.h>
#include <alica_tests/behaviours/BehBAA.h>

#include <alica/test/Util.h>
#include <alica_tests/test_sched_world_model.h>
#include <gtest/gtest.h>

namespace alica::test
{

TEST_F(SingleAgentTestFixture, scheduling)
{
    // Checks if scheduling is working

    // Transition to the plan corresponding to this test case
    ASSERT_TRUE(_tc->setTransitionCond("TestMasterPlan", "ChooseTestState", "SchedulingTestState")) << _tc->getLastFailure();
    STEP_UNTIL(_tc, _tc->getActivePlan("SchedulingTestPlan"));
    ASSERT_TRUE(_tc->setTransitionCond("SchedulingTestPlan", "ChooseTestState", "SchedulingPlanInitTermTestState")) << _tc->getLastFailure();
    STEP_UNTIL(_tc, _tc->getActivePlan("SchedulingPlanInitTermTestPlan"));
    auto plan = _tc->getActivePlan("SchedulingTestPlan");
    ASSERT_NE(plan, nullptr) << _tc->getLastFailure();

    // Step until the test plan succeeds
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->isSuccess(plan));
}

TEST_F(SingleAgentTestFixture, orderedInitTermCheck)
{
    // Checks if scheduling is working

    // Transition to the plan corresponding to this test case
    ASSERT_TRUE(_tc->setTransitionCond("TestMasterPlan", "ChooseTestState", "SchedulingTestState")) << _tc->getLastFailure();
    STEP_UNTIL(_tc, _tc->getActivePlan("SchedulingTestPlan"));
    ASSERT_TRUE(_tc->setTransitionCond("SchedulingTestPlan", "ChooseTestState", "OrderedSchedulingTestState")) << _tc->getLastFailure();
    STEP_UNTIL(_tc, _tc->getActivePlan("OrderedSchedulingTestPlan"));
    auto plan = _tc->getActivePlan("SchedulingTestPlan");
    ASSERT_NE(plan, nullptr) << _tc->getLastFailure();

    // Step until the test plan succeeds
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->isSuccess(plan));
}

TEST_F(SingleAgentTestFixture, orderedRunCheck)
{
    // Checks if scheduling is working

    // Transition to the plan corresponding to this test case
    ASSERT_TRUE(_tc->setTransitionCond("TestMasterPlan", "ChooseTestState", "SchedulingTestState")) << _tc->getLastFailure();
    STEP_UNTIL(_tc, _tc->getActivePlan("SchedulingTestPlan"));
    ASSERT_TRUE(_tc->setTransitionCond("SchedulingTestPlan", "ChooseTestState", "OrderedRunTestState")) << _tc->getLastFailure();
    STEP_UNTIL(_tc, _tc->getActivePlan("OrderedRunTestPlan"));
    auto plan = _tc->getActivePlan("SchedulingTestPlan");
    ASSERT_NE(plan, nullptr) << _tc->getLastFailure();

    // Step until the test plan succeeds
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->isSuccess(plan));
}

TEST_F(SingleAgentTestFixture, behaviourRunCheck)
{
    // Checks if scheduling is working

    // Transition to the plan corresponding to this test case
    ASSERT_TRUE(_tc->setTransitionCond("TestMasterPlan", "ChooseTestState", "SchedulingTestState")) << _tc->getLastFailure();
    STEP_UNTIL(_tc, _tc->getActivePlan("SchedulingTestPlan"));
    ASSERT_TRUE(_tc->setTransitionCond("SchedulingTestPlan", "ChooseTestState", "BehaviourRunSchedulingTestState")) << _tc->getLastFailure();
    STEP_UNTIL(_tc, _tc->getActivePlan("BehaviourRunSchedulingTestPlan"));
    auto plan = _tc->getActivePlan("SchedulingTestPlan");
    ASSERT_NE(plan, nullptr) << _tc->getLastFailure();

    // Step until the test plan succeeds
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->isSuccess(plan));
}

TEST_F(SingleAgentTestFixture, execBehaviourCheck)
{
    // Checks if scheduling is working

    // Transition to the plan corresponding to this test case
    ASSERT_TRUE(_tc->setTransitionCond("TestMasterPlan", "ChooseTestState", "SchedulingTestState")) << _tc->getLastFailure();
    STEP_UNTIL(_tc, _tc->getActivePlan("SchedulingTestPlan"));
    ASSERT_TRUE(_tc->setTransitionCond("SchedulingTestPlan", "ChooseTestState", "ExecuteBehaviourTestState")) << _tc->getLastFailure();
    STEP_UNTIL(_tc, _tc->getActivePlan("ExecuteBehaviourTestPlan"));
    auto plan = _tc->getActivePlan("SchedulingTestPlan");
    ASSERT_NE(plan, nullptr) << _tc->getLastFailure();

    // Step until the test plan succeeds
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->isSuccess(plan));
}

//     wm->transitionToEndTest = true;
// }

} // namespace alica::test
