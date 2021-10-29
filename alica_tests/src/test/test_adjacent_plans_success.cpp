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
    std::cerr << "plansTest" << std::endl;
    alicaTests::TestWorldModel::getOne()->setTransitionCondition3143778092687974738(false);

    alicaTests::TestWorldModel::getOne()->setTransitionCondition3345031375302716643(false);
    alicaTests::TestWorldModel::getOne()->setTransitionCondition1914245867924544479(false);

    ae->start();
    auto successSpamBehaviour = alica::test::Util::getBasicBehaviour(ae, 1522377401286, 0);

    alicaTests::TestWorldModel::getOne()->setTransitionCondition3143778092687974738(true);
    std::cerr << "allow subplan transition" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(50)); // wait for setSuccessCall
    EXPECT_TRUE(successSpamBehaviour->isSuccess());

    std::cerr << "allow mp transitions" << std::endl;
    alicaTests::TestWorldModel::getOne()->setTransitionCondition3345031375302716643(true);
    alicaTests::TestWorldModel::getOne()->setTransitionCondition1914245867924544479(true);
    alicaTests::TestWorldModel::getOne()->setTransitionCondition3143778092687974738(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    EXPECT_TRUE(successSpamBehaviour->isSuccess());
}

TEST_F(AlicaAdjacentPlansSuccess, adjacentPlansBehaviourSuccess)
{
    std::cerr << "behTest" << std::endl;
    ae->start();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    auto successSpamBehaviour = alica::test::Util::getBasicBehaviour(ae, 1522377401286, 0);

    for(int i = 0; i < 10; i++) {
        // allow transition to state with SuccessSpam behaviour in AdjacentSuccessSubPlan
        alicaTests::TestWorldModel::getOne()->setTransitionCondition3143778092687974738(true);
        // allow transition between EntryState and SecondState of AdjacentSuccessMasterPlan
        alicaTests::TestWorldModel::getOne()->setTransitionCondition3345031375302716643(true);
        alicaTests::TestWorldModel::getOne()->setTransitionCondition1914245867924544479(true);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        EXPECT_FALSE(successSpamBehaviour->isSuccess());
    }   
}

} // namespace
} // namespace alica
