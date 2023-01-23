#include "test_alica.h"
#include <alica/test/Util.h>
#include <gtest/gtest.h>

namespace alica
{
namespace
{

class AlicaFailureHandlingEnabledFixture : public AlicaTestFixture
{
protected:
    const char* getMasterPlanName() const override { return "FailureHandlingMaster"; }
    const char* getHostName() const override { return "hairy"; }
    void SetUp() override
    {
        AlicaTestFixture::SetUp();
        ac->setOption("Alica.AutoFailureHandling", true);
    }
};

class AlicaFailureHandlingDisabledFixture : public AlicaTestFixture
{
protected:
    const char* getMasterPlanName() const override { return "FailureHandlingMaster"; }
    const char* getHostName() const override { return "nase"; }
    void SetUp() override
    {
        AlicaTestFixture::SetUp();
        ac->setOption("Alica.AutoFailureHandling", false);
    }
};

class AlicaFailureHandlingEnabledMultiAgentFixture : public AlicaTestMultiAgentFixture
{
protected:
    const char* getMasterPlanName() const override { return "FailureHandlingMaster"; }
    virtual int getAgentCount() const { return 2; }
    const char* getHostName(int agentNumber) const override { return agentNumber ? "nase" : "hairy"; }
    void SetUp() override
    {
        AlicaTestMultiAgentFixture::SetUp();
        acs[0]->setOption("Alica.AutoFailureHandling", true);
        acs[1]->setOption("Alica.AutoFailureHandling", true);
    }
};

class AlicaFailureHandlingDisabledMultiAgentFixture : public AlicaTestMultiAgentFixture
{
protected:
    const char* getMasterPlanName() const override { return "FailureHandlingMaster"; }
    virtual int getAgentCount() const { return 2; }
    const char* getHostName(int agentNumber) const override { return agentNumber ? "nase" : "hairy"; }
    void SetUp() override
    {
        AlicaTestMultiAgentFixture::SetUp();
        acs[0]->setOption("Alica.AutoFailureHandling", false);
        acs[1]->setOption("Alica.AutoFailureHandling", false);
    }
};

TEST_F(AlicaFailureHandlingEnabledFixture, redoPlanOnFailure)
{
    // Checks if the FailurePlan is restarted on failure i.e. the redo plan rule should be applied if a plan fails once
    std::shared_ptr<alicaTests::TestWorldModel> twm1 =
            LockedBlackboardRW(ac->editGlobalBlackboard()).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");

    const uint64_t FAILURE_PLAN_INIT_STATE = 1171453089016322268;
    const uint64_t FAILURE_PLAN_FAIL_STATE = 3487518754011112127;

    STEP_UNTIL(test::Util::isStateActive(ae, FAILURE_PLAN_INIT_STATE));
    ASSERT_TRUE(test::Util::isStateActive(ae, FAILURE_PLAN_INIT_STATE));

    twm1->setTransitionCondition1446293122737278544(true);
    STEP_UNTIL(test::Util::isStateActive(ae, FAILURE_PLAN_FAIL_STATE));
    ASSERT_TRUE(test::Util::isStateActive(ae, FAILURE_PLAN_FAIL_STATE));

    // Transition to plan failure state, which will redo the plan & we should end up in the init state again
    twm1->setTransitionCondition1023566846009251524(true);
    twm1->setTransitionCondition1446293122737278544(false);
    STEP_UNTIL(test::Util::isStateActive(ae, FAILURE_PLAN_INIT_STATE));
    ASSERT_TRUE(test::Util::isStateActive(ae, FAILURE_PLAN_INIT_STATE));
    ASSERT_EQ(twm1->failurePlanInitCallCount(), 1);
}

TEST_F(AlicaFailureHandlingDisabledFixture, autoFailureHandlingDisabledTest)
{
    // Checks if nothing is done when a plan failure occurs when auto failure handling is disabled
    std::shared_ptr<alicaTests::TestWorldModel> twm1 =
            LockedBlackboardRW(ac->editGlobalBlackboard()).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    const uint64_t FAILURE_PLAN_INIT_STATE = 1171453089016322268;
    const uint64_t FAILURE_PLAN_FAIL_STATE = 3487518754011112127;
    const uint64_t FAILURE_PLAN_FAILED_STATE = 3748960977005112327;
    const uint64_t FAILURE_HANDLING_MASTER_FAILURE_HANDLED_STATE = 4449850763179483831;

    STEP_UNTIL(test::Util::isStateActive(ae, FAILURE_PLAN_INIT_STATE));
    ASSERT_TRUE(test::Util::isStateActive(ae, FAILURE_PLAN_INIT_STATE));

    twm1->setTransitionCondition1446293122737278544(true);
    STEP_UNTIL(test::Util::isStateActive(ae, FAILURE_PLAN_FAIL_STATE));
    ASSERT_TRUE(test::Util::isStateActive(ae, FAILURE_PLAN_FAIL_STATE));

    twm1->setTransitionCondition1446293122737278544(false);
    twm1->setTransitionCondition1023566846009251524(true);
    STEP_UNTIL(test::Util::isStateActive(ae, FAILURE_PLAN_FAILED_STATE));
    ASSERT_TRUE(test::Util::isStateActive(ae, FAILURE_PLAN_FAILED_STATE));

    // Check if we remain in the failed state
    STEP_UNTIL(test::Util::isStateActive(ae, FAILURE_PLAN_FAILED_STATE));

    ASSERT_TRUE(test::Util::isStateActive(ae, FAILURE_PLAN_FAILED_STATE));

    // Check if higher level plan can check for child failure
    twm1->enableTransitionCondition3194919312481305139();
    STEP_UNTIL(test::Util::isStateActive(ae, FAILURE_HANDLING_MASTER_FAILURE_HANDLED_STATE));
    ASSERT_TRUE(test::Util::isStateActive(ae, FAILURE_HANDLING_MASTER_FAILURE_HANDLED_STATE));
    ASSERT_EQ(twm1->failurePlanInitCallCount(), 1);
}

TEST_F(AlicaFailureHandlingEnabledMultiAgentFixture, redoPlanOnFailureMultiAgent)
{
    // Checks if the FailurePlan is restarted on failure i.e. the redo plan rule should be applied if a plan fails once
    // The rule should only be applied for the robot that had the failure

    std::shared_ptr<alicaTests::TestWorldModel> twm1 =
            LockedBlackboardRW(acs[0]->editGlobalBlackboard()).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    std::shared_ptr<alicaTests::TestWorldModel> twm2 =
            LockedBlackboardRW(acs[1]->editGlobalBlackboard()).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");

    const uint64_t FAILURE_PLAN_INIT_STATE = 1171453089016322268;
    const uint64_t FAILURE_PLAN_FAIL_STATE = 3487518754011112127;

    aes[0]->start();
    aes[1]->start();
    STEP_UNTIL(acs[0], test::Util::isStateActive(aes[0], FAILURE_PLAN_INIT_STATE));
    ASSERT_TRUE(test::Util::isStateActive(aes[0], FAILURE_PLAN_INIT_STATE));
    STEP_UNTIL(acs[1], test::Util::isStateActive(aes[1], FAILURE_PLAN_INIT_STATE));
    ASSERT_TRUE(test::Util::isStateActive(aes[1], FAILURE_PLAN_INIT_STATE));

    twm1->setTransitionCondition1446293122737278544(true);
    twm2->setTransitionCondition1446293122737278544(true);
    STEP_UNTIL(acs[0], test::Util::isStateActive(aes[0], FAILURE_PLAN_FAIL_STATE));
    ASSERT_TRUE(test::Util::isStateActive(aes[0], FAILURE_PLAN_FAIL_STATE));
    STEP_UNTIL(acs[1], test::Util::isStateActive(aes[1], FAILURE_PLAN_FAIL_STATE));
    ASSERT_TRUE(test::Util::isStateActive(aes[1], FAILURE_PLAN_FAIL_STATE));

    // Transition to plan failure state, which will redo the plan for agent 0 & we should end up in the init state again
    // For agent 1 we should remain in the same state i.e. the plan redo rule should not be applied
    twm1->setTransitionCondition1023566846009251524(true);
    twm1->setTransitionCondition1446293122737278544(false);
    twm2->setTransitionCondition1446293122737278544(false);
    STEP_UNTIL(acs[0], test::Util::isStateActive(aes[0], FAILURE_PLAN_INIT_STATE));
    ASSERT_TRUE(test::Util::isStateActive(aes[0], FAILURE_PLAN_INIT_STATE));
    STEP_UNTIL(acs[1], test::Util::isStateActive(aes[1], FAILURE_PLAN_FAIL_STATE));
    ASSERT_TRUE(test::Util::isStateActive(aes[1], FAILURE_PLAN_FAIL_STATE));
    ASSERT_EQ(twm1->failurePlanInitCallCount(), 1);
    ASSERT_EQ(twm2->failurePlanInitCallCount(), 1);
}

TEST_F(AlicaFailureHandlingDisabledMultiAgentFixture, autoFailureHandlingDisabledMultiAgentTest)
{
    // Checks if nothing is done when a plan failure occurs when auto failure handling is disabled for both agents
    std::shared_ptr<alicaTests::TestWorldModel> twm1 =
            LockedBlackboardRW(acs[0]->editGlobalBlackboard()).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    std::shared_ptr<alicaTests::TestWorldModel> twm2 =
            LockedBlackboardRW(acs[1]->editGlobalBlackboard()).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");

    const uint64_t FAILURE_PLAN_INIT_STATE = 1171453089016322268;
    const uint64_t FAILURE_PLAN_FAIL_STATE = 3487518754011112127;
    const uint64_t FAILURE_PLAN_FAILED_STATE = 3748960977005112327;
    const uint64_t FAILURE_HANDLING_MASTER_FAILURE_HANDLED_STATE = 4449850763179483831;

    aes[0]->start();
    aes[1]->start();
    STEP_UNTIL(acs[0], test::Util::isStateActive(aes[0], FAILURE_PLAN_INIT_STATE));
    ASSERT_TRUE(test::Util::isStateActive(aes[0], FAILURE_PLAN_INIT_STATE));
    STEP_UNTIL(acs[1], test::Util::isStateActive(aes[1], FAILURE_PLAN_INIT_STATE));
    ASSERT_TRUE(test::Util::isStateActive(aes[1], FAILURE_PLAN_INIT_STATE));

    twm1->setTransitionCondition1446293122737278544(true);
    twm2->setTransitionCondition1446293122737278544(true);
    STEP_UNTIL(acs[0], test::Util::isStateActive(aes[0], FAILURE_PLAN_FAIL_STATE));
    ASSERT_TRUE(test::Util::isStateActive(aes[0], FAILURE_PLAN_FAIL_STATE));
    STEP_UNTIL(acs[1], test::Util::isStateActive(aes[1], FAILURE_PLAN_FAIL_STATE));
    ASSERT_TRUE(test::Util::isStateActive(aes[1], FAILURE_PLAN_FAIL_STATE));

    twm1->setTransitionCondition1446293122737278544(false);
    twm1->setTransitionCondition1023566846009251524(true);
    twm2->setTransitionCondition1446293122737278544(false);
    twm2->setTransitionCondition1023566846009251524(true);
    STEP_UNTIL(acs[0], test::Util::isStateActive(aes[0], FAILURE_PLAN_FAILED_STATE));
    ASSERT_TRUE(test::Util::isStateActive(aes[0], FAILURE_PLAN_FAILED_STATE));
    STEP_UNTIL(acs[1], test::Util::isStateActive(aes[1], FAILURE_PLAN_FAILED_STATE));
    ASSERT_TRUE(test::Util::isStateActive(aes[1], FAILURE_PLAN_FAILED_STATE));

    // Check if we remain in the failed state
    STEP_ALL_UNTIL(acs, test::Util::isStateActive(aes[0], FAILURE_PLAN_FAILED_STATE) && test::Util::isStateActive(aes[1], FAILURE_PLAN_FAILED_STATE));

    ASSERT_TRUE(test::Util::isStateActive(aes[0], FAILURE_PLAN_FAILED_STATE));
    ASSERT_TRUE(test::Util::isStateActive(aes[1], FAILURE_PLAN_FAILED_STATE));

    // Check if higher level plan can check for child failure
    twm1->enableTransitionCondition3194919312481305139();
    twm2->enableTransitionCondition3194919312481305139();
    STEP_UNTIL(acs[0], test::Util::isStateActive(aes[0], FAILURE_HANDLING_MASTER_FAILURE_HANDLED_STATE));
    ASSERT_TRUE(test::Util::isStateActive(aes[0], FAILURE_HANDLING_MASTER_FAILURE_HANDLED_STATE));
    STEP_UNTIL(acs[1], test::Util::isStateActive(aes[1], FAILURE_HANDLING_MASTER_FAILURE_HANDLED_STATE));
    ASSERT_TRUE(test::Util::isStateActive(aes[1], FAILURE_HANDLING_MASTER_FAILURE_HANDLED_STATE));
    ASSERT_EQ(twm1->failurePlanInitCallCount(), 1);
    ASSERT_EQ(twm2->failurePlanInitCallCount(), 1);
}

} // namespace
} // namespace alica
