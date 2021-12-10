#include "test_alica.h"

#include <alica_tests/TestWorldModel.h>
#include <alica/test/Util.h>

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
    virtual void SetUp() override
    {
        AlicaTestFixture::SetUp();
    }
    virtual void TearDown() override
    {
        AlicaTestFixture::TearDown();
    }
};

TEST_F(AlicaAdjacentPlansSuccess, adjacentPlansPlanSuccess)
{
    auto worldModel = dynamic_cast<alicaTests::TestWorldModel*>(ac->getWorldModel());
    worldModel->setTransitionCondition3143778092687974738(false);
    worldModel->setTransitionCondition3345031375302716643(false);
    worldModel->setTransitionCondition1914245867924544479(false);

    ae->start();
    auto successSpamBehaviour = alica::test::Util::getBasicBehaviour(ae, 1522377401286, 0);

    worldModel->setTransitionCondition3143778092687974738(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(50)); // wait for setSuccessCall
    EXPECT_FALSE(successSpamBehaviour->isSuccess() && alica::test::Util::isPlanActive(ae, 656998006978148289));

    for(int i = 0; i < 10; i++) {
        if (i % 2 == 0) {
            worldModel->setTransitionCondition1914245867924544479(false);
            worldModel->setTransitionCondition3345031375302716643(true);
        } else {
            worldModel->setTransitionCondition3345031375302716643(false);
            worldModel->setTransitionCondition1914245867924544479(true);
        }

        // allow transition to state with SuccessSpam behaviour in AdjacentSuccessSubPlan
        worldModel->setTransitionCondition3143778092687974738(true);

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        // make sure the plan never succeeds after a transition, success from behaviour does not carry
        // over to sibling plan.
        EXPECT_FALSE(successSpamBehaviour->isSuccess() && alica::test::Util::isPlanActive(ae, 656998006978148289));
    }
    
}

} // namespace
} // namespace alica
