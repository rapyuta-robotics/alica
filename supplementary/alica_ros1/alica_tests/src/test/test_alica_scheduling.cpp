#include "test_alica.h"
#include <alica/test/CounterClass.h>
#include <alica_tests/Behaviour/BehAAA.h>
#include <alica_tests/Behaviour/BehBAA.h>

#include <alica/test/Util.h>
#include <alica_tests/test_sched_world_model.h>
#include <gtest/gtest.h>

namespace alica
{
namespace
{

class AlicaSchedulingPlan : public AlicaSchedulingTestFixture
{
protected:
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "SchedulingTestMasterPlan"; }
    bool stepEngine() const override { return true; }
    virtual void SetUp() override { AlicaSchedulingTestFixture::SetUp(); }
    virtual void TearDown() override { AlicaSchedulingTestFixture::TearDown(); }
};

TEST_F(AlicaSchedulingPlan, scheduling)
{
    ae->start();
    CounterClass::called = 0;

    STEP_UNTIL(CounterClass::called == 1);
    // init of scheduling plan 1
    ASSERT_EQ(1, CounterClass::called);
    CounterClass::called += 1; // allow transition

    STEP_UNTIL(CounterClass::called == 4);
    // init of scheduling plan 2 and 3
    ASSERT_EQ(4, CounterClass::called);
    CounterClass::called += 1; // allow transition

    STEP_UNTIL(CounterClass::called == 7);
    // onTermination of scheduling plan 2 and 3 called
    ASSERT_EQ(7, CounterClass::called);
    CounterClass::called += 1; // allow transition

    STEP_UNTIL(CounterClass::called == 9);
    // onTermination of scheduling plan 1 called
    ASSERT_EQ(9, CounterClass::called);
}

TEST_F(AlicaSchedulingPlan, orderedInitTermCheck)
{
    CounterClass::called = -1;
    ae->start();

    alica_test::SchedWM* wm = LockedBlackboardRW(ac->editGlobalBlackboard()).get<std::shared_ptr<alica_test::SchedWM>>("worldmodel").get();

    std::string planAInitOrder = "PlanA::Init\nPlanAA::Init\nBehAAA::Init\n";
    std::string planATermOrder = "BehAAA::Term\nPlanAA::Term\nPlanA::Term\n";
    std::string planBInitOrder = "PlanB::Init\nPlanBA::Init\nBehBAA::Init\n";
    std::string planBTermOrder = "BehBAA::Term\nPlanBA::Term\nPlanB::Term\n";

    std::string expectedExecOrder = planAInitOrder;
    wm->execOrderTest = true;
    STEP_UNTIL(expectedExecOrder == wm->execOrder);
    ASSERT_EQ(expectedExecOrder, wm->execOrder);

    for (int i = 0; i < 10; ++i) {
        expectedExecOrder += planATermOrder + planBInitOrder;
        wm->planA2PlanB = true;
        wm->planB2PlanA = false;
        STEP_UNTIL(expectedExecOrder == wm->execOrder);
        ASSERT_EQ(expectedExecOrder, wm->execOrder);

        expectedExecOrder += planBTermOrder + planAInitOrder;
        wm->planA2PlanB = false;
        wm->planB2PlanA = true;
        STEP_UNTIL(expectedExecOrder == wm->execOrder);
        ASSERT_EQ(expectedExecOrder, wm->execOrder);
    }

    wm->execOrder.clear();
    wm->execOrder = planAInitOrder;
    for (int i = 0; i < 10; ++i) {
        wm->planA2PlanB = true;
        wm->planB2PlanA = false;
        ac->stepEngine();
        wm->planA2PlanB = false;
        wm->planB2PlanA = true;
        ac->stepEngine();
    }
    ASSERT_EQ(expectedExecOrder, wm->execOrder);
}

TEST_F(AlicaSchedulingPlan, orderedRunCheck)
{
    CounterClass::called = -1;
    ae->start();

    alica_test::SchedWM* wm = LockedBlackboardRW(ac->editGlobalBlackboard()).get<std::shared_ptr<alica_test::SchedWM>>("worldmodel").get();
    wm->execOrderTest = true;
    ac->stepEngine();

    for (int i = 0; i < 10; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        wm->planA2PlanB = true;
        wm->planB2PlanA = false;
        ac->stepEngine();

        wm->planA2PlanB = false;
        wm->planB2PlanA = true;
        ac->stepEngine();
    }
    ASSERT_TRUE(wm->planARunCalled);
    ASSERT_FALSE(wm->planARunOutOfOrder);
    ASSERT_TRUE(wm->behAAARunCalled);
    ASSERT_FALSE(wm->behAAARunOutOfOrder);
}

TEST_F(AlicaSchedulingPlan, behaviourSuccessFailureCheck)
{
    CounterClass::called = -1;
    ae->start();

    alica_test::SchedWM* wm = LockedBlackboardRW(ac->editGlobalBlackboard()).get<std::shared_ptr<alica_test::SchedWM>>("worldmodel").get();
    wm->execOrderTest = true;
    ac->stepEngine();

    // std::this_thread::sleep_for(std::chrono::milliseconds(50));
    SLEEP_UNTIL(alica::test::Util::getBasicBehaviour(ae, 1629895901559, 0) != nullptr);
    auto behAAA = alica::test::Util::getBasicBehaviour(ae, 1629895901559, 0);
    ASSERT_FALSE(wm->behAAASuccessInInit);
    ASSERT_FALSE(wm->behAAAFailureInInit);
    ASSERT_FALSE(behAAA->isSuccess());
    ASSERT_FALSE(behAAA->isFailure());

    wm->behAAASetSuccess = true;
    ac->stepEngine();
    SLEEP_UNTIL(behAAA->isSuccess());
    ASSERT_TRUE(behAAA->isSuccess());
    ASSERT_FALSE(behAAA->isFailure());
    ASSERT_FALSE(wm->behAAASetSuccessFailed);

    wm->behAAASetSuccess = false;
    wm->behAAASetFailure = true;
    ac->stepEngine();
    SLEEP_UNTIL(behAAA->isFailure());
    ASSERT_FALSE(behAAA->isSuccess());
    ASSERT_TRUE(behAAA->isFailure());
    ASSERT_FALSE(wm->behAAASetFailureFailed);

    wm->planA2PlanB = true;
    wm->planB2PlanA = false;
    ac->stepEngine();
    wm->behAAASetSuccess = false;
    wm->behAAASetFailure = false;

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ASSERT_TRUE(wm->behAAASuccessInTerminate);
    ASSERT_TRUE(wm->behAAAFailureInTerminate);

    wm->planA2PlanB = false;
    wm->planB2PlanA = true;
    ac->stepEngine();

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    behAAA = alica::test::Util::getBasicBehaviour(ae, 1629895901559, 0);
    ASSERT_FALSE(wm->behAAASuccessInInit);
    ASSERT_FALSE(wm->behAAAFailureInInit);
    ASSERT_FALSE(behAAA->isSuccess());
    ASSERT_FALSE(behAAA->isFailure());

    wm->behAAASetSuccess = true;
    SLEEP_UNTIL(behAAA->isSuccess());
    ASSERT_TRUE(behAAA->isSuccess());
    ASSERT_FALSE(behAAA->isFailure());
    ASSERT_FALSE(wm->behAAASetSuccessFailed);
    wm->behAAABlockRun = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    wm->planA2PlanB = true;
    wm->planB2PlanA = false;
    wm->behAAABlockRun = false;
    ac->stepEngine();
    ASSERT_TRUE(wm->behAAASuccessInTerminate);
    ASSERT_TRUE(wm->behAAAFailureInTerminate);
}

TEST_F(AlicaSchedulingPlan, behaviourRunCheck)
{
    CounterClass::called = -1;
    ae->start();

    alica_test::SchedWM* wm = LockedBlackboardRW(ac->editGlobalBlackboard()).get<std::shared_ptr<alica_test::SchedWM>>("worldmodel").get();
    wm->execOrderTest = true;
    ac->stepEngine();

    for (int i = 0; i < 10; ++i) {

        STEP_UNTIL(alica::BehAAA::runCount >= 1);
        SLEEP_UNTIL(alica::BehAAA::runCount >= 10);
        ASSERT_GE(alica::BehAAA::runCount, 10);

        wm->planA2PlanB = true;
        wm->planB2PlanA = false;
        STEP_UNTIL(alica::BehAAA::runCount == 0);

        ASSERT_EQ(alica::BehAAA::runCount, 0);

        STEP_UNTIL(alica::BehBAA::runCount >= 1);
        SLEEP_UNTIL(alica::BehBAA::runCount >= 10);
        ASSERT_GE(alica::BehBAA::runCount, 10);

        wm->planA2PlanB = false;
        wm->planB2PlanA = true;
        STEP_UNTIL(alica::BehBAA::runCount == 0);
        ASSERT_EQ(alica::BehBAA::runCount, 0);
    }
}

TEST_F(AlicaSchedulingPlan, execBehaviourCheck)
{
    CounterClass::called = -1;
    ae->start();

    alica_test::SchedWM* wm = LockedBlackboardRW(ac->editGlobalBlackboard()).get<std::shared_ptr<alica_test::SchedWM>>("worldmodel").get();

    wm->execBehaviourTest = true;
    std::string orderString = "TestBehaviour::Init\nTestBehaviour::Run\n";
    STEP_UNTIL(wm->execOrder == orderString);
    EXPECT_EQ(wm->execOrder, orderString);

    wm->transitionToExecuteBehaviour = false;
    wm->transitionToExecuteBehaviourInSubPlan = true;
    orderString += "TestBehaviour::Term\nTestBehaviour::Init\nTestBehaviour::Run\n";
    STEP_UNTIL(wm->execOrder == orderString);
    EXPECT_EQ(wm->execOrder, orderString);

    std::string execOrderBeforeTransition = "TestBehaviour::Term\nTestBehaviour::Init\nTestBehaviour::Run\n";
    std::string execOrderAfterTransition = execOrderBeforeTransition + "TestBehaviour::Term\nTestBehaviour::Init\nTestBehaviour::Run\n";

    for (int i = 0; i < 10; i++) {
        wm->execOrder = "";

        wm->transitionToExecuteBehaviourInSubPlan = false;
        wm->transitionToExecuteBehaviour = true;
        STEP_UNTIL(wm->execOrder == execOrderBeforeTransition);
        EXPECT_EQ(wm->execOrder, execOrderBeforeTransition);

        wm->transitionToExecuteBehaviour = false;
        wm->transitionToExecuteBehaviourInSubPlan = true;
        STEP_UNTIL(wm->execOrder == execOrderAfterTransition);
        EXPECT_EQ(wm->execOrder, execOrderAfterTransition);
    }

    wm->transitionToEndTest = true;
}

} // namespace
} // namespace alica
