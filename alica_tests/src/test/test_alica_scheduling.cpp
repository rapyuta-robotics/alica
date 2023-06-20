#include "test_alica.h"

#include <alica/test/Util.h>
#include <gtest/gtest.h>

namespace alica::test
{

TEST_F(SingleAgentTestFixture, transitionAfterInitTest)
{
    /**
     * Checks if outgoing transitions are evaluated after all plans and behaviours in the init state
     * (recursively until the leaf behaviour) have been initialized
     */

    // Transition to the plan corresponding to this test case
    ASSERT_TRUE(_tc->setTransitionCond("TestMasterPlan", "ChooseTestState", "SchedulingTestState")) << _tc->getLastFailure();
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->getActivePlan("SchedulingTestPlan"));
    auto plan = _tc->getActivePlan("SchedulingTestPlan");
    ASSERT_TRUE(_tc->setTransitionCond("SchedulingTestPlan", "ChooseTestState", "TransitionAfterInitTestState")) << _tc->getLastFailure();

    // Step until the test plan succeeds
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->isSuccess(plan));
}

TEST_F(SingleAgentTestFixture, execOrderSinglePlanTest)
{
    // Checks if execution order of init, run and terminate is correct for a single plan

    // Transition to the plan corresponding to this test case
    ASSERT_TRUE(_tc->setTransitionCond("TestMasterPlan", "ChooseTestState", "SchedulingTestState")) << _tc->getLastFailure();
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->getActivePlan("SchedulingTestPlan"));
    ASSERT_TRUE(_tc->setTransitionCond("SchedulingTestPlan", "ChooseTestState", "SchedulingPlanTestState")) << _tc->getLastFailure();
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->getActivePlan("SchedulingPlanTestPlan"));
    ASSERT_TRUE(_tc->setTransitionCond("SchedulingPlanTestPlan", "EntryState", "CheckState")) << _tc->getLastFailure();
    auto plan = _tc->getActivePlan("SchedulingTestPlan");
    ASSERT_NE(plan, nullptr) << _tc->getLastFailure();

    // Step until the test plan succeeds
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->isSuccess(plan));

    // evaluate execOrder vector
    LockedBlackboardRO bb(_tc->getGlobalBlackboard());
    std::vector<std::string> execOrder = bb.get<std::vector<std::string>>("execOrder");
    ASSERT_EQ(execOrder.size(), 6);
    ASSERT_EQ(execOrder.at(0), "PlanA::Init");
    ASSERT_EQ(execOrder.at(1), "PlanAA::Init");
    ASSERT_EQ(execOrder.at(2), "BehAAA::Init");
    ASSERT_EQ(execOrder.at(3), "BehAAA::Term");
    ASSERT_EQ(execOrder.at(4), "PlanAA::Term");
    ASSERT_EQ(execOrder.at(5), "PlanA::Term");

    ASSERT_TRUE(bb.get<bool>("BehAAARunCalled"));
    ASSERT_FALSE(bb.hasValue("BehAAARunOutOfOrder"));
    ASSERT_TRUE(bb.get<bool>("PlanARunCalled"));
    ASSERT_FALSE(bb.hasValue("PlanARunOutOfOrder"));
}

TEST_F(SingleAgentTestFixture, testRepeatedRunSchedulingTest)
{
    // Checks if run is called at the expected frequency

    // Transition to the plan corresponding to this test case
    ASSERT_TRUE(_tc->setTransitionCond("TestMasterPlan", "ChooseTestState", "SchedulingTestState")) << _tc->getLastFailure();
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->getActivePlan("SchedulingTestPlan"));
    ASSERT_TRUE(_tc->setTransitionCond("SchedulingTestPlan", "ChooseTestState", "RepeatedRunCallsTestState")) << _tc->getLastFailure();
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->getActivePlan("TestRepeatedRunsPlan"));
    ASSERT_TRUE(_tc->setTransitionCond("TestRepeatedRunsPlan", "EntryState", "CheckState")) << _tc->getLastFailure();
    auto plan = _tc->getActivePlan("SchedulingTestPlan");
    ASSERT_NE(plan, nullptr) << _tc->getLastFailure();

    // Step until the test plan succeeds
    // A single STEP_UNTIL_ASSERT_TRUE might time out
    for (int i = 0; i < 5; ++i) {
        STEP_UNTIL(_tc, _tc->isSuccess(plan));
    }
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->isSuccess(plan));
}

TEST_F(SingleAgentTestFixture, execOrderTransitionBetweenPlansTest)
{
    // Checks if scheduling of init, run and terminate when transitioning between 2 plans is working

    // Transition to the plan corresponding to this test case
    ASSERT_TRUE(_tc->setTransitionCond("TestMasterPlan", "ChooseTestState", "SchedulingTestState")) << _tc->getLastFailure();
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->getActivePlan("SchedulingTestPlan"));
    ASSERT_TRUE(_tc->setTransitionCond("SchedulingTestPlan", "ChooseTestState", "ExecOrderTestState")) << _tc->getLastFailure();
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->getActivePlan("ExecOrderTestPlan"));
    ASSERT_TRUE(_tc->setTransitionCond("ExecOrderTestPlan", "EntryState", "PlanAState")) << _tc->getLastFailure();
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
    // Leave A
    ASSERT_EQ(execOrder.at(3), "BehAAA::Term");
    ASSERT_EQ(execOrder.at(4), "PlanAA::Term");
    ASSERT_EQ(execOrder.at(5), "PlanA::Term");
    // Enter B
    ASSERT_EQ(execOrder.at(6), "PlanB::Init");
    ASSERT_EQ(execOrder.at(7), "PlanBA::Init");
    ASSERT_EQ(execOrder.at(8), "BehBAA::Init");

    // Transition between B and A 5 times
    for (int i = 0; i <= 5; ++i) {
        // Leave B
        ASSERT_EQ(execOrder.at(9 + 12 * i), "BehBAA::Term");
        ASSERT_EQ(execOrder.at(10 + 12 * i), "PlanBA::Term");
        ASSERT_EQ(execOrder.at(11 + 12 * i), "PlanB::Term");
        // Enter A
        ASSERT_EQ(execOrder.at(12 + 12 * i), "PlanA::Init");
        ASSERT_EQ(execOrder.at(13 + 12 * i), "PlanAA::Init");
        ASSERT_EQ(execOrder.at(14 + 12 * i), "BehAAA::Init");
        // Leave A
        ASSERT_EQ(execOrder.at(15 + 12 * i), "BehAAA::Term");
        ASSERT_EQ(execOrder.at(16 + 12 * i), "PlanAA::Term");
        ASSERT_EQ(execOrder.at(17 + 12 * i), "PlanA::Term");
        // Enter B
        ASSERT_EQ(execOrder.at(18 + 12 * i), "PlanB::Init");
        ASSERT_EQ(execOrder.at(19 + 12 * i), "PlanBA::Init");
        ASSERT_EQ(execOrder.at(20 + 12 * i), "BehBAA::Init");
    }

    // Leave B
    ASSERT_EQ(execOrder.at(81), "BehBAA::Term");
    ASSERT_EQ(execOrder.at(82), "PlanBA::Term");
    ASSERT_EQ(execOrder.at(83), "PlanB::Term");

    ASSERT_EQ(execOrder.size(), 84);

    ASSERT_TRUE(bb.get<bool>("BehAAARunCalled"));
    ASSERT_FALSE(bb.hasValue("BehAAARunOutOfOrder"));
    ASSERT_TRUE(bb.get<bool>("BehBAARunCalled"));
    ASSERT_FALSE(bb.hasValue("BehBAARunOutOfOrder"));
    ASSERT_TRUE(bb.get<bool>("PlanARunCalled"));
    ASSERT_FALSE(bb.hasValue("PlanARunOutOfOrder"));
    ASSERT_TRUE(bb.get<bool>("PlanBRunCalled"));
    ASSERT_FALSE(bb.hasValue("PlanBRunOutOfOrder"));
}

} // namespace alica::test
