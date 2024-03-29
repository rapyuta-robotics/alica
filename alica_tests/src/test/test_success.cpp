#include <DynamicBehaviourCreator.h>
#include <DynamicConditionCreator.h>
#include <DynamicConstraintCreator.h>
#include <DynamicPlanCreator.h>
#include <DynamicTransitionConditionCreator.h>
#include <DynamicUtilityFunctionCreator.h>
#include <alica/test/TestContext.h>
#include <alica/test/Util.h>
#include <alica_tests/TestWorldModel.h>
#include <communication/AlicaDummyCommunication.h>
#include <engine/AlicaTimer.h>
#include <engine/logging/AlicaDefaultLogger.h>
#include <test_alica.h>

#include <gtest/gtest.h>

namespace alica::test
{

TEST_F(SingleAgentTestFixture, behSuccessTest)
{
    // Checks if a behaviour can succeed

    // Transition to the plan corresponding to this test case
    ASSERT_TRUE(_tc->setTransitionCond("TestMasterPlan", "ChooseTestState", "BehSuccessTestState")) << _tc->getLastFailure();
    STEP_UNTIL(_tc, _tc->getActiveBehaviour("SuccessOnInitBeh"));
    auto beh = _tc->getActiveBehaviour("SuccessOnInitBeh");
    ASSERT_NE(beh, nullptr) << _tc->getLastFailure();

    // Step until the behaviour succeeds
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->isSuccess(beh));
}

TEST_F(SingleAgentTestFixture, planSuccessTest)
{
    // Checks if a plan can succeed

    // Transition to the plan corresponding to this test case
    ASSERT_TRUE(_tc->setTransitionCond("TestMasterPlan", "ChooseTestState", "PlanSuccessTestState")) << _tc->getLastFailure();
    STEP_UNTIL(_tc, _tc->getActivePlan("SuccessOnInitPlan"));
    auto plan = _tc->getActivePlan("SuccessOnInitPlan");
    ASSERT_NE(plan, nullptr) << _tc->getLastFailure();

    // Step until the plan succeeds
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->isSuccess(plan));
}

TEST_F(SingleAgentTestFixture, multiPlanInstanceSuccessTest)
{
    // Checks if multiple instances of the same plan can run in parallel & succeed without mixing up runtime states with each other

    // Transition to the plan corresponding to this test case
    ASSERT_TRUE(_tc->setTransitionCond("TestMasterPlan", "ChooseTestState", "MultiPlanInstanceSuccessTestState")) << _tc->getLastFailure();
    STEP_UNTIL(_tc, _tc->getActivePlan("ParallelSuccessOnCondPlan"));
    auto parallelPlan = _tc->getActivePlan("ParallelSuccessOnCondPlan");
    ASSERT_NE(parallelPlan, nullptr) << _tc->getLastFailure();

    // Transition to the state that has 2 plans attached to it i.e. parallelly executes both these attached plans. These attached plans inturn execute the same
    // sub-plan (SuccessOnCondPlan). . This implies that 2 instances of the SuccessOnCondPlan execute parallely at the same time
    ASSERT_TRUE(_tc->setTransitionCond("ParallelSuccessOnCondPlan", "WaitForTriggerState", "ParallelExecState")) << _tc->getLastFailure();

    // Step until the first instance of SuccessOnCondPlan is active. Since 2 instances are expected to be active we need to use the fully qualified name to
    // access each instance
    std::string fqnA = "/MultiPlanInstanceSuccessTestState/ParallelSuccessOnCondState/<ParallelExecState,SuccessOnCondWrapperAPlan>/SuccessOnCondState";
    STEP_UNTIL(_tc, _tc->getActivePlan(fqnA));
    auto planInstanceA = _tc->getActivePlan(fqnA);
    ASSERT_NE(planInstanceA, nullptr) << _tc->getLastFailure();

    // Succeed the first instance, kill some time & then verify that only the first instance is set to true
    ASSERT_TRUE(_tc->setTransitionCond(fqnA, "WaitForCondState", "CondSuccessState"));
    STEP_UNTIL(_tc, false);
    ASSERT_TRUE(_tc->isSuccess(planInstanceA));

    // Verify that we are still executing the second instance
    ASSERT_TRUE(_tc->isStateActive("ParallelSuccessOnCondPlan", "ParallelExecState")) << _tc->getLastFailure();
    std::string fqnB = "/MultiPlanInstanceSuccessTestState/ParallelSuccessOnCondState/<ParallelExecState,SuccessOnCondWrapperBPlan>/SuccessOnCondState";
    STEP_UNTIL(_tc, _tc->getActivePlan(fqnB));
    auto planInstanceB = _tc->getActivePlan(fqnB);
    ASSERT_NE(planInstanceB, nullptr) << _tc->getLastFailure();

    // Verify that the second instance has not succeeded
    ASSERT_FALSE(_tc->isSuccess(planInstanceB));

    // Now set success for the second instance
    ASSERT_TRUE(_tc->setTransitionCond(fqnB, "WaitForCondState", "CondSuccessState")) << _tc->getLastFailure();
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->isSuccess(planInstanceB));

    // Transition to the success states of the wrapper plans
    auto wrapperA = _tc->getActivePlan("SuccessOnCondWrapperAPlan");
    auto wrapperB = _tc->getActivePlan("SuccessOnCondWrapperBPlan");
    ASSERT_NE(wrapperA, nullptr) << _tc->getLastFailure();
    ASSERT_NE(wrapperB, nullptr) << _tc->getLastFailure();
    ASSERT_TRUE(_tc->setTransitionCond("SuccessOnCondWrapperAPlan", "SuccessOnCondState", "WrapperASuccessState")) << _tc->getLastFailure();
    ASSERT_TRUE(_tc->setTransitionCond("SuccessOnCondWrapperBPlan", "SuccessOnCondState", "WrapperBSuccessState")) << _tc->getLastFailure();

    // Step until the test plan succeeds
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->isSuccess(parallelPlan));
}

TEST_F(SingleAgentTestFixture, isChildSuccessTest)
{
    // Checks if isChildSuccess condition is working

    // Transition to the plan corresponding to this test case
    ASSERT_TRUE(_tc->setTransitionCond("TestMasterPlan", "ChooseTestState", "IsChildSuccessTestState")) << _tc->getLastFailure();
    STEP_UNTIL(_tc, _tc->getActivePlan("IsChildSuccessTestPlan"));
    auto isChildSuccessPlan = _tc->getActivePlan("IsChildSuccessTestPlan");
    ASSERT_NE(isChildSuccessPlan, nullptr) << _tc->getLastFailure();

    // Step until the test plan succeeds
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->isSuccess(isChildSuccessPlan));
}

TEST_F(SingleAgentTestFixture, adjacentSuccessTestPlan)
{
    // Checks if plans in adjacent states can succeed without interfering with each other

    // Transition to the plan corresponding to this test case
    ASSERT_TRUE(_tc->setTransitionCond("TestMasterPlan", "ChooseTestState", "AdjacentSuccessTestState")) << _tc->getLastFailure();
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->getActivePlan("AdjacentSuccessTestPlan")) << _tc->getLastFailure();

    // Switch back & forth between the 2 states having the same attached plan
    for (std::size_t i = 0; i < 10; ++i) {
        // Agent should be in the first state i.e. SuccessOnCondStateA
        STEP_UNTIL_ASSERT_TRUE(_tc, _tc->isStateActive("AdjacentSuccessTestPlan", "SuccessOnCondStateA")) << _tc->getLastFailure();
        STEP_UNTIL_ASSERT_TRUE(_tc, _tc->getActivePlan("SuccessOnCondPlan")) << _tc->getLastFailure();
        auto successOnCondPlan = _tc->getActivePlan("SuccessOnCondPlan");
        ASSERT_FALSE(_tc->isSuccess(successOnCondPlan));

        // Transition to the second state i.e. SuccessOnCondStateB by setting success
        ASSERT_TRUE(_tc->setTransitionCond("SuccessOnCondPlan", "WaitForCondState", "CondSuccessState")) << _tc->getLastFailure();
        STEP_UNTIL_ASSERT_TRUE(_tc, _tc->isStateActive("AdjacentSuccessTestPlan", "SuccessOnCondStateB")) << _tc->getLastFailure();
        STEP_UNTIL_ASSERT_TRUE(_tc, _tc->getActivePlan("SuccessOnCondPlan")) << _tc->getLastFailure();
        successOnCondPlan = _tc->getActivePlan("SuccessOnCondPlan");
        ASSERT_FALSE(_tc->isSuccess(successOnCondPlan));

        // Transition back to the first state
        ASSERT_TRUE(_tc->setTransitionCond("SuccessOnCondPlan", "WaitForCondState", "CondSuccessState")) << _tc->getLastFailure();
    }
}

} // namespace alica::test
