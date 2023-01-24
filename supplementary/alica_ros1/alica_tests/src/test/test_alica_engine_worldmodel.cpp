#include "test_alica.h"

#include <alica/test/TestContext.h>
#include <alica/test/Util.h>
#include <alica_tests/TestFixture.h>
#include <gtest/gtest.h>

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
