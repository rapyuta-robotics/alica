#include <alica/test/TestContext.h>
#include <alica/test/Util.h>
#include <alica_tests/Behaviour/RuntimeConditionCalledPlanState2Behaviour.h>
#include <alica_tests/RuntimeConditionCalledPlan3213121947038933654.h>
#include <alica_tests/TestFixture.h>
#include <gtest/gtest.h>

namespace alica::test
{

/**
 * Tests whether it is possible to run runtimecondition in plan and in behaviur
 *
 * Steps:
 * 1) Move to RuntimeConditionCalledPlanState1 state
 * 2) Check if runtimecondition for RuntimeConditionCalledPlan is called
 * 3) Move to RuntimeConditionCalledPlanState2 state
 * 4) Check if runtimecondition for RuntimeConditionCalledPlanState2Behaviour is called (TODO)
 *
 */

TEST_F(TestFixture, runtimeConditionCalled)
{
    return;
    // Transition to the plan corresponding to this test case
    ASSERT_TRUE(_tc->setTransitionCond("TestMasterPlan", "ChooseTestState", "RuntimeConditionCalledPlanState")) << _tc->getLastFailure();
    STEP_UNTIL(_tc, _tc->getActivePlan("RuntimeConditionCalledPlan"));
    ASSERT_NE(nullptr, _tc->getActivePlan("RuntimeConditionCalledPlan")) << _tc->getLastFailure();

    // RuntimeConditionCalledPlanState1 check runtimecondition for RuntimeConditionCalledPlan
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->getActivePlan("RuntimeConditionCalledPlan"));
    ASSERT_TRUE(_tc->isStateActive("RuntimeConditionCalledPlan", "RuntimeConditionCalledPlanState1")) << _tc->getLastFailure();
    ASSERT_NE(nullptr, _tc->getActiveBehaviour("RuntimeConditionCalledPlanState1Behaviour")) << _tc->getLastFailure();
    ASSERT_EQ(nullptr, _tc->getActiveBehaviour("RuntimeConditionCalledPlanState2Behaviour")) << _tc->getLastFailure();
    auto plan = _tc->getActivePlan("RuntimeConditionCalledPlan");
    int runTimePlanConditionCounter = dynamic_cast<RuntimeConditionCalledPlan3213121947038933654*>(plan)->getRunTimeConditionCounter();
    ASSERT_EQ(1, runTimePlanConditionCounter);

    // RuntimeConditionCalledPlanState2
    ASSERT_TRUE(_tc->setTransitionCond("RuntimeConditionCalledPlan", "RuntimeConditionCalledPlanState1", "RuntimeConditionCalledPlanState2"))
            << _tc->getLastFailure();
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->isStateActive("RuntimeConditionCalledPlan", "RuntimeConditionCalledPlanState2"));
    ASSERT_EQ(_tc->getActivePlan("RuntimeConditionCalledPlan")->getName(), "RuntimeConditionCalledPlan") << _tc->getLastFailure();
    ASSERT_NE(nullptr, _tc->getActiveBehaviour("RuntimeConditionCalledPlanState2Behaviour")) << _tc->getLastFailure();
    ASSERT_EQ(nullptr, _tc->getActiveBehaviour("RuntimeConditionCalledPlanState1Behaviour")) << _tc->getLastFailure();
    return;

    // TODO - Check unTimeConditionCounter for RuntimeConditionCalledPlanState2Behaviour
    /*
    int runTimeConditionCounterEnd = dynamic_cast<RuntimeConditionCalledPlan3213121947038933654*>(plan)->getRunTimeConditionCounter();
    ASSERT_FALSE(0, runTimeConditionCounterEnd);
*/
}

} // namespace alica::test
