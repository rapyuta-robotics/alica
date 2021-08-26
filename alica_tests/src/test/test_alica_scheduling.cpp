#include "CounterClass.h"
#include "test_alica.h"

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

} // namespace
} // namespace alica
