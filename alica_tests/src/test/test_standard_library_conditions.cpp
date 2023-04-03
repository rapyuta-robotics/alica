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

TEST_F(SingleAgentTestFixture, equalConditionTest)
{
    // Checks if the EqualCondition works as expected

    // Transition to the plan corresponding to this test case
    ASSERT_TRUE(_tc->setTransitionCond("TestMasterPlan", "ChooseTestState", "StandardLibraryCompareConditionsState")) << _tc->getLastFailure();
    STEP_UNTIL(_tc, _tc->getActivePlan("StandardLibraryCompareConditionsPlan"));
    auto plan = _tc->getActivePlan("StandardLibraryCompareConditionsPlan");
    ASSERT_NE(plan, nullptr) << _tc->getLastFailure();

    ASSERT_TRUE(_tc->setTransitionCond("StandardLibraryCompareConditionsPlan", "CompareCondition", "CompareEqualString")) << _tc->getLastFailure();
    // Step until the plan succeeds
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->isSuccess(plan));
}

TEST_F(SingleAgentTestFixture, lessThanConditionTest)
{
    // Checks if the LessCondition works as expected

    // Transition to the plan corresponding to this test case
    ASSERT_TRUE(_tc->setTransitionCond("TestMasterPlan", "ChooseTestState", "StandardLibraryCompareConditionsState")) << _tc->getLastFailure();
    STEP_UNTIL(_tc, _tc->getActivePlan("StandardLibraryCompareConditionsPlan"));
    auto plan = _tc->getActivePlan("StandardLibraryCompareConditionsPlan");
    ASSERT_NE(plan, nullptr) << _tc->getLastFailure();

    ASSERT_TRUE(_tc->setTransitionCond("StandardLibraryCompareConditionsPlan", "CompareCondition", "CompareLessThanString")) << _tc->getLastFailure();
    // Step until the plan succeeds
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->isSuccess(plan));
}
} // namespace alica::test
