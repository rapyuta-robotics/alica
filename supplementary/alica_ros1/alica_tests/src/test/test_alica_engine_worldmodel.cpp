#include "test_alica.h"

#include <alica/test/TestContext.h>
#include <alica/test/Util.h>
#include <alica_tests/TestFixture.h>
#include <gtest/gtest.h>

/*
TODO Luca
uncomment when it will be possible to add transition TestMasterPlan->ConstraintTestPlan
namespace alica::test
{

TEST_F(TestFixture, simpleGetWM)
{
    // Transition to the plan corresponding to this test case
    ASSERT_TRUE(_tc->setTransitionCond("TestMasterPlan", "ChooseTestState", "ConstraintTestPlanState")) << _tc->getLastFailure();
    STEP_UNTIL(_tc, _tc->getActivePlan("ConstraintTestPlan"));
    ASSERT_NE(nullptr, _tc->getActivePlan("ConstraintTestPlan")) << _tc->getLastFailure();

    LockedBlackboardRW(_tc->editGlobalBlackboard()).set("worldmodel1", std::make_shared<alicaTests::TestWorldModelNew>(_tc.get()));
    std::shared_ptr<alicaTests::TestWorldModelNew> wm1 =
            LockedBlackboardRO(_tc->editGlobalBlackboard()).get<std::shared_ptr<alicaTests::TestWorldModelNew>>("worldmodel1");
    EXPECT_NE(nullptr, wm1);

    LockedBlackboardRW(_tc->editGlobalBlackboard()).set("worldmodel2", std::make_shared<alicaTests::TestWorldModelNew>(_tc.get()));
    std::shared_ptr<alicaTests::TestWorldModelNew> wm2 =
            LockedBlackboardRO(_tc->editGlobalBlackboard()).get<std::shared_ptr<alicaTests::TestWorldModelNew>>("worldmodel2");
    EXPECT_NE(nullptr, wm2);
}
} // namespace alica::test
*/

namespace alica
{
namespace
{
class AlicaTestFixtureWM : public AlicaTestFixture
{
protected:
    const char* getMasterPlanName() const override { return "ConstraintTestPlan"; }
};

TEST_F(AlicaTestFixtureWM, simpleGetWM)
{
    std::shared_ptr<alicaTests::TestWorldModel> wm1 =
            LockedBlackboardRW(ac->editGlobalBlackboard()).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    EXPECT_NE(nullptr, wm1);

    LockedBlackboardRW(ac->editGlobalBlackboard()).set("worldmodel2", std::make_shared<alicaTests::TestWorldModel>());
    std::shared_ptr<alicaTests::TestWorldModel> wm2 =
            LockedBlackboardRW(ac->editGlobalBlackboard()).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel2");
    EXPECT_NE(nullptr, wm2);
}

} // namespace
} // namespace alica
