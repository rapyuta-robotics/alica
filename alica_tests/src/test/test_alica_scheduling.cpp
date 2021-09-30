#include <alica_tests/CounterClass.h>
#include "test_alica.h"
#include "Behaviour/BehAAA.h"
#include "Behaviour/BehBAA.h"

#include <alica/test/Util.h>
#include <gtest/gtest.h>
#include <alica_tests/test_sched_world_model.h>

namespace alica
{
namespace
{

class AlicaSchedulingPlan : public AlicaTestFixture
{
protected:
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "SchedulingTestMasterPlan"; }
    bool stepEngine() const override { return true; }
    virtual void SetUp() override
    {
        alica_test::SchedWM::instance().reset();
        AlicaTestFixture::SetUp();
    }
    virtual void TearDown() override
    {
        AlicaTestFixture::TearDown();
        alica_test::SchedWM::instance().reset();
    }
};

TEST_F(AlicaSchedulingPlan, scheduling)
{
    ae->start();
    CounterClass::called = 0;

    ac->stepEngine();
    // init of scheduling plan 1
    ASSERT_EQ(1, CounterClass::called);
    CounterClass::called += 1; // allow transition

    ac->stepEngine();
    // init of scheduling plan 2 and 3
    ASSERT_EQ(4, CounterClass::called);
    CounterClass::called += 1; // allow transition

    ac->stepEngine();
    // onTermination of scheduling plan 2 and 3 called
    ASSERT_EQ(7, CounterClass::called);
    CounterClass::called += 1; // allow transition

    ac->stepEngine();
    // onTermination of scheduling plan 1 called
    ASSERT_EQ(9, CounterClass::called);
}

TEST_F(AlicaSchedulingPlan, orderedInitTermCheck)
{
    CounterClass::called = -1;
    ae->start();

    auto& wm = alica_test::SchedWM::instance();

    std::string planAInitOrder = "PlanA::Init\nPlanAA::Init\nBehAAA::Init\n";
    std::string planATermOrder = "BehAAA::Term\nPlanAA::Term\nPlanA::Term\n";
    std::string planBInitOrder = "PlanB::Init\nPlanBA::Init\nBehBAA::Init\n";
    std::string planBTermOrder = "BehBAA::Term\nPlanBA::Term\nPlanB::Term\n";

    std::string expectedExecOrder = planAInitOrder;
    wm.execOrderTest = true;
    ac->stepEngine();
    ASSERT_EQ(expectedExecOrder, wm.execOrder);

    for (int i = 0; i < 10; ++i) {
        expectedExecOrder += planATermOrder + planBInitOrder;
        wm.planA2PlanB = true;
        wm.planB2PlanA = false;
        ac->stepEngine();
        ASSERT_EQ(expectedExecOrder, wm.execOrder);

        expectedExecOrder += planBTermOrder + planAInitOrder;
        wm.planA2PlanB = false;
        wm.planB2PlanA = true;
        ac->stepEngine();
        ASSERT_EQ(expectedExecOrder, wm.execOrder);
    }

    wm.execOrder.clear();
    wm.execOrder = planAInitOrder;
    for (int i = 0; i < 10; ++i) {
        wm.planA2PlanB = true;
        wm.planB2PlanA = false;
        ac->stepEngine();
        wm.planA2PlanB = false;
        wm.planB2PlanA = true;
        ac->stepEngine();
    }
    ASSERT_EQ(expectedExecOrder, wm.execOrder);
}

TEST_F(AlicaSchedulingPlan, orderedRunCheck)
{
    CounterClass::called = -1;
    ae->start();

    auto& wm = alica_test::SchedWM::instance();
    wm.execOrderTest = true;
    ac->stepEngine();

    for (int i = 0; i < 10; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        wm.planA2PlanB = true;
        wm.planB2PlanA = false;
        ac->stepEngine();

        wm.planA2PlanB = false;
        wm.planB2PlanA = true;
        ac->stepEngine();
    }
    ASSERT_TRUE(wm.planARunCalled);
    ASSERT_FALSE(wm.planARunOutOfOrder);
    ASSERT_TRUE(wm.behAAARunCalled);
    ASSERT_FALSE(wm.behAAARunOutOfOrder);
}

TEST_F(AlicaSchedulingPlan, behaviourSuccessFailureCheck)
{
    CounterClass::called = -1;
    ae->start();

    auto& wm = alica_test::SchedWM::instance();
    wm.execOrderTest = true;
    ac->stepEngine();

    auto behAAA = alica::test::Util::getBasicBehaviour(ae, 1629895901559, 0);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ASSERT_FALSE(wm.behAAASuccessInInit);
    ASSERT_FALSE(wm.behAAAFailureInInit);
    ASSERT_FALSE(behAAA->isSuccess());
    ASSERT_FALSE(behAAA->isFailure());

    wm.behAAASetSuccess = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ASSERT_TRUE(behAAA->isSuccess());
    ASSERT_FALSE(behAAA->isFailure());
    ASSERT_FALSE(wm.behAAASetSuccessFailed);

    wm.behAAASetSuccess = false;
    wm.behAAASetFailure = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ASSERT_FALSE(behAAA->isSuccess());
    ASSERT_TRUE(behAAA->isFailure());
    ASSERT_FALSE(wm.behAAASetFailureFailed);

    wm.planA2PlanB = true;
    wm.planB2PlanA = false;
    ac->stepEngine();
    wm.behAAASetSuccess = false;
    wm.behAAASetFailure = false;

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ASSERT_FALSE(wm.behAAASuccessInTerminate);
    ASSERT_FALSE(wm.behAAAFailureInTerminate);
    ASSERT_FALSE(behAAA->isSuccess());
    ASSERT_FALSE(behAAA->isFailure());

    wm.planA2PlanB = false;
    wm.planB2PlanA = true;
    ac->stepEngine();

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ASSERT_FALSE(wm.behAAASuccessInInit);
    ASSERT_FALSE(wm.behAAAFailureInInit);
    ASSERT_FALSE(behAAA->isSuccess());
    ASSERT_FALSE(behAAA->isFailure());

    wm.behAAASetSuccess = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ASSERT_TRUE(behAAA->isSuccess());
    ASSERT_FALSE(behAAA->isFailure());
    ASSERT_FALSE(wm.behAAASetSuccessFailed);
    wm.behAAABlockRun = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    wm.planA2PlanB = true;
    wm.planB2PlanA = false;
    ac->stepEngine();
    wm.behAAABlockRun = false;
    ASSERT_FALSE(behAAA->isSuccess());
    ASSERT_FALSE(behAAA->isFailure());
}

TEST_F(AlicaSchedulingPlan, behaviourRunCheck)
{
    CounterClass::called = -1;
    ae->start();
    auto& wm = alica_test::SchedWM::instance();
    wm.execOrderTest = true;
    ac->stepEngine();

    auto behAAA = std::dynamic_pointer_cast<alica::BehAAA>(alica::test::Util::getBasicBehaviour(ae, 1629895901559, 0));
    auto behBAA = std::dynamic_pointer_cast<alica::BehBAA>(alica::test::Util::getBasicBehaviour(ae, 1629895911592, 0));

    for (int i = 0; i < 10; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        ASSERT_GT(behAAA->runCount, 10);

        wm.planA2PlanB = true;
        wm.planB2PlanA = false;
        ac->stepEngine();

        ASSERT_EQ(behAAA->runCount, 0);

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        ASSERT_GT(behBAA->runCount, 10);

        wm.planA2PlanB = false;
        wm.planB2PlanA = true;
        ac->stepEngine();

        ASSERT_EQ(behBAA->runCount, 0);
    }
}

TEST_F(AlicaSchedulingPlan, execBehaviourCheck)
{
    CounterClass::called = -1;
    ae->start();

    auto& wm = alica_test::SchedWM::instance();
    wm.execBehaviourTest = true;
    ac->stepEngine();

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    ASSERT_TRUE(wm.executeBehaviourRunCalled);

    alica_test::SchedWM::instance().transitionToExecuteBehaviourInSubPlan = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    ASSERT_TRUE(wm.executeBehaviourRunCalled);

    for (int i = 0; i < 10; i++) {
        alica_test::SchedWM::instance().transitionToExecuteBehaviourInSubPlan = false;
        alica_test::SchedWM::instance().transitionToExecuteBehaviour = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        ASSERT_TRUE(wm.executeBehaviourRunCalled);

        alica_test::SchedWM::instance().transitionToExecuteBehaviourInSubPlan = true;
        alica_test::SchedWM::instance().transitionToExecuteBehaviour = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        ASSERT_TRUE(wm.executeBehaviourRunCalled);
    }

    alica_test::SchedWM::instance().transitionToEndTest = true;
}

} // namespace
} // namespace alica
