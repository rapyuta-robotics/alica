#include "test_alica.h"

#include <alica/test/Util.h>
#include <gtest/gtest.h>

namespace alica::test
{

TEST_F(SingleAgentTestFixture, transitionAfterInitTest)
{
    // Checks if outgoing transitions are evaluated after all plans and behaviours in a state have been initialized

    // Transition to the plan corresponding to this test case
    ASSERT_TRUE(_tc->setTransitionCond("TestMasterPlan", "ChooseTestState", "SchedulingTestState")) << _tc->getLastFailure();
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->getActivePlan("SchedulingTestPlan"));
    ASSERT_TRUE(_tc->setTransitionCond("SchedulingTestPlan", "ChooseTestState", "TransitionAfterInitTestState")) << _tc->getLastFailure();

    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->getActivePlan("TransitionAfterInitTestPlan"));
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->getActivePlan("SchedulingTestPlan"));
}

TEST_F(SingleAgentTestFixture, execOrderSinglePlanTest)
{
    // Checks if execution order of init, run and terminate is correct for a single plan

    // Transition to the plan corresponding to this test case
    ASSERT_TRUE(_tc->setTransitionCond("TestMasterPlan", "ChooseTestState", "SchedulingTestState")) << _tc->getLastFailure();
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->getActivePlan("SchedulingTestPlan"));
    ASSERT_TRUE(_tc->setTransitionCond("SchedulingTestPlan", "ChooseTestState", "SchedulingPlanTestState")) << _tc->getLastFailure();
    auto plan = _tc->getActivePlan("SchedulingTestPlan");
    ASSERT_NE(plan, nullptr) << _tc->getLastFailure();

    // Step until the test plan succeeds
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->isSuccess(plan));

    // evaluate execOrder vector
    LockedBlackboardRO bb(_tc->getGlobalBlackboard());
    std::vector<std::string> execOrder = bb.get<std::vector<std::string>>("execOrder");
    ASSERT_EQ(execOrder.size(), 7);
    ASSERT_EQ(execOrder.at(0), "PlanA::Init");
    ASSERT_EQ(execOrder.at(1), "PlanAA::Init");
    ASSERT_EQ(execOrder.at(2), "BehAAA::Init");
    ASSERT_EQ(execOrder.at(3), "BehAAA::Run");
    ASSERT_EQ(execOrder.at(4), "BehAAA::Term");
    ASSERT_EQ(execOrder.at(5), "PlanAA::Term");
    ASSERT_EQ(execOrder.at(6), "PlanA::Term");
}

TEST_F(SingleAgentTestFixture, testRepeatedRunSchedulingTest)
{
    // Checks if run is called repeatedly

    // Transition to the plan corresponding to this test case
    ASSERT_TRUE(_tc->setTransitionCond("TestMasterPlan", "ChooseTestState", "SchedulingTestState")) << _tc->getLastFailure();
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->getActivePlan("SchedulingTestPlan"));
    ASSERT_TRUE(_tc->setTransitionCond("SchedulingTestPlan", "ChooseTestState", "RepeatedRunCallsTestState")) << _tc->getLastFailure();
    auto plan = _tc->getActivePlan("SchedulingTestPlan");
    ASSERT_NE(plan, nullptr) << _tc->getLastFailure();

    // Step until the test plan succeeds
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->isSuccess(plan));
}

TEST_F(SingleAgentTestFixture, execOrderTransitionBetweenPlansTest)
{
    // Checks if scheduling of init, run and terminate when transitioning between 2 plans is working

    // Transition to the plan corresponding to this test case
    ASSERT_TRUE(_tc->setTransitionCond("TestMasterPlan", "ChooseTestState", "SchedulingTestState")) << _tc->getLastFailure();
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->getActivePlan("SchedulingTestPlan"));
    ASSERT_TRUE(_tc->setTransitionCond("SchedulingTestPlan", "ChooseTestState", "PlanAState")) << _tc->getLastFailure();
    auto plan = _tc->getActivePlan("SchedulingTestPlan");
    ASSERT_NE(plan, nullptr) << _tc->getLastFailure();

    // Step until the test plan succeeds
    // A single STEP_UNTIL_ASSERT_TRUE might time out
    STEP_UNTIL(_tc, _tc->isSuccess(plan));
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->isSuccess(plan));

    LockedBlackboardRO bb(_tc->getGlobalBlackboard());
    std::vector<std::string> execOrder = bb.get<std::vector<std::string>>("execOrder");

    // Enter A
    ASSERT_EQ(execOrder.at(0), "PlanA::Init");
    ASSERT_EQ(execOrder.at(1), "PlanAA::Init");
    ASSERT_EQ(execOrder.at(2), "BehAAA::Init");
    ASSERT_EQ(execOrder.at(3), "BehAAA::Run");
    // Leave A
    ASSERT_EQ(execOrder.at(4), "BehAAA::Term");
    ASSERT_EQ(execOrder.at(5), "PlanAA::Term");
    ASSERT_EQ(execOrder.at(6), "PlanA::Term");
    // Enter B
    ASSERT_EQ(execOrder.at(7), "PlanB::Init");
    ASSERT_EQ(execOrder.at(8), "PlanBA::Init");
    ASSERT_EQ(execOrder.at(9), "BehBAA::Init");
    ASSERT_EQ(execOrder.at(10), "BehBAA::Run");

    // Transition between B and A 5 times
    for (int i = 0; i <= 4; ++i) {
        // Leave B
        ASSERT_EQ(execOrder.at(11 + 14 * i), "BehBAA::Term");
        ASSERT_EQ(execOrder.at(12 + 14 * i), "PlanBA::Term");
        ASSERT_EQ(execOrder.at(13 + 14 * i), "PlanB::Term");
        // Enter A
        ASSERT_EQ(execOrder.at(14 + 14 * i), "PlanA::Init");
        ASSERT_EQ(execOrder.at(15 + 14 * i), "PlanAA::Init");
        ASSERT_EQ(execOrder.at(16 + 14 * i), "BehAAA::Init");
        ASSERT_EQ(execOrder.at(17 + 14 * i), "BehAAA::Run");
        // Leave A
        ASSERT_EQ(execOrder.at(18 + 14 * i), "BehAAA::Term");
        ASSERT_EQ(execOrder.at(19 + 14 * i), "PlanAA::Term");
        ASSERT_EQ(execOrder.at(20 + 14 * i), "PlanA::Term");
        // Enter B
        ASSERT_EQ(execOrder.at(21 + 14 * i), "PlanB::Init");
        ASSERT_EQ(execOrder.at(22 + 14 * i), "PlanBA::Init");
        ASSERT_EQ(execOrder.at(23 + 14 * i), "BehBAA::Init");
        ASSERT_EQ(execOrder.at(24 + 14 * i), "BehBAA::Run");
    }

    // Leave B
    ASSERT_EQ(execOrder.at(81), "BehBAA::Term");
    ASSERT_EQ(execOrder.at(82), "PlanBA::Term");
    ASSERT_EQ(execOrder.at(83), "PlanB::Term");

    ASSERT_EQ(execOrder.size(), 84);
}

} // namespace alica::test
