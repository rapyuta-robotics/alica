#include "test_alica.h"

#include <alica/test/Util.h>
#include <alica_tests/TestWorldModel.h>

#include <gtest/gtest.h>

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
    alicaTests::TestWorldModel* worldModel = ac->getWorldModel<alicaTests::TestWorldModel>();
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
