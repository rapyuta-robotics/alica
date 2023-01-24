#include "test_alica.h"

#include <alica/test/TestContext.h>
#include <alica/test/Util.h>
#include <alica_tests/TestFixture.h>
#include <gtest/gtest.h>

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
