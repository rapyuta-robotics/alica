#include "test_alica.h"

#include <alica/test/TestContext.h>
#include <alica/test/Util.h>
#include <alica_tests/TestFixture.h>
#include <gtest/gtest.h>

/*
TODO Luca
uncomment when it will be possible to add transition TestMasterPlan->AdjacentSuccessMasterPlan

namespace alica::test
{
TEST_F(TestFixture, adjacentPlansPlanSuccess)
{
    // Transition to the plan corresponding to this test case
    // ASSERT_TRUE(_tc->setTransitionCond("TestMasterPlan", "ChooseTestState", "AdjacentSuccessSubPlanState")) << _tc->getLastFailure();
    // STEP_UNTIL(_tc, _tc->getActivePlan("AdjacentSuccessSubPlan"));
    // ASSERT_NE(nullptr, _tc->getActivePlan("AdjacentSuccessSubPlan")) << _tc->getLastFailure();

    EXPECT_TRUE(_tc->isStateActive("AdjacentSuccessSubPlan", "EntryState"));

    // Go to SecondState
    ASSERT_TRUE(_tc->resetTransitionCond("AdjacentSuccessMasterPlan", "SecondState", "EntryState")) << _tc->getLastFailure();
    ASSERT_TRUE(_tc->setTransitionCond("AdjacentSuccessMasterPlan", "EntryState", "SecondState")) << _tc->getLastFailure();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_TRUE(_tc->isStateActive("AdjacentSuccessMasterPlan", "SecondState"));

    // Go to EntryState
    ASSERT_TRUE(_tc->resetTransitionCond("AdjacentSuccessMasterPlan", "EntryState", "SecondState")) << _tc->getLastFailure();
    ASSERT_TRUE(_tc->setTransitionCond("AdjacentSuccessMasterPlan", "SecondState", "EntryState")) << _tc->getLastFailure();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_TRUE(_tc->isStateActive("AdjacentSuccessMasterPlan", "EntryState"));

    // Go to SecondState
    ASSERT_TRUE(_tc->resetTransitionCond("AdjacentSuccessMasterPlan", "SecondState", "EntryState")) << _tc->getLastFailure();
    ASSERT_TRUE(_tc->setTransitionCond("AdjacentSuccessMasterPlan", "EntryState", "SecondState")) << _tc->getLastFailure();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_TRUE(_tc->isStateActive("AdjacentSuccessMasterPlan", "SecondState"));

    // Go to EntryState
    ASSERT_TRUE(_tc->resetTransitionCond("AdjacentSuccessMasterPlan", "EntryState", "SecondState")) << _tc->getLastFailure();
    ASSERT_TRUE(_tc->setTransitionCond("AdjacentSuccessMasterPlan", "SecondState", "EntryState")) << _tc->getLastFailure();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_TRUE(_tc->isStateActive("AdjacentSuccessMasterPlan", "EntryState"));

    for (int i = 0; i < 10; i++) {
        // go into WaitState of subPlan before evaluating

        // worldModel1->setTransitionCondition1067314038887345208(false); // do not transition from WaitState to SucState in subPlan
        // worldModel1->setTransitionCondition1747408236004727286(true);  // transition from EntryState to WaitState in subPlan
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        if (i % 2 == 0) {
            // EXPECT_TRUE(alica::test::Util::isStateActive(ae, 338845808462999166)); // is in EntryState
            EXPECT_TRUE(_tc->isStateActive("AdjacentSuccessMasterPlan", "EntryState"));
        } else {
            // EXPECT_TRUE(alica::test::Util::isStateActive(ae, 1114306208475690481)); // is in SecondState
            EXPECT_TRUE(_tc->isStateActive("AdjacentSuccessMasterPlan", "SecondState"));
        }

        // EXPECT_FALSE(alica::test::Util::hasPlanSucceeded(ae, 1682631238618360548)); // subPlan has not succeeded yet

        // go into SucState of subPlan, triggers transition in MasterPlan
        worldModel1->setTransitionCondition1747408236004727286(false); // do not transition from EntryState to WaitState in subPlan
        worldModel1->setTransitionCondition1067314038887345208(true);  // transition from WaitState to SucState in subPlan

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}
} // namespace alica::test

*/
namespace alica
{
namespace
{

class AlicaAdjacentPlansSuccess : public AlicaTestFixture
{
protected:
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "AdjacentSuccessMasterPlan"; }
    bool stepEngine() const override { return false; }
    virtual void SetUp() override { AlicaTestFixture::SetUp(); }
    virtual void TearDown() override { AlicaTestFixture::TearDown(); }
};

TEST_F(AlicaAdjacentPlansSuccess, adjacentPlansPlanSuccess)
{
    std::shared_ptr<alicaTests::TestWorldModel> worldModel =
            LockedBlackboardRW(ac->editGlobalBlackboard()).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    ae->start();

    for (int i = 0; i < 10; i++) {
        // go into WaitState of subPlan before evaluating
        worldModel->setTransitionCondition1067314038887345208(false); // do not transition from WaitState to SucState in subPlan
        worldModel->setTransitionCondition1747408236004727286(true);  // transition from EntryState to WaitState in subPlan
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        if (i % 2 == 0) {
            EXPECT_TRUE(alica::test::Util::isStateActive(ae, 338845808462999166)); // is in EntryState
        } else {
            EXPECT_TRUE(alica::test::Util::isStateActive(ae, 1114306208475690481)); // is in SecondState
        }

        EXPECT_FALSE(alica::test::Util::hasPlanSucceeded(ae, 1682631238618360548)); // subPlan has not succeeded yet

        // go into SucState of subPlan, triggers transition in MasterPlan
        worldModel->setTransitionCondition1747408236004727286(false); // do not transition from EntryState to WaitState in subPlan
        worldModel->setTransitionCondition1067314038887345208(true);  // transition from WaitState to SucState in subPlan

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

} // namespace
} // namespace alica
