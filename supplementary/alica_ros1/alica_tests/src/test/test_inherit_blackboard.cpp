#include "alica_tests/TestWorldModel.h"
#include "test_alica.h"
#include <alica/test/Util.h>

namespace alica
{
namespace
{

class TestInheritBlackboard : public AlicaTestFixture
{
protected:
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "TestInheritBlackboardMaster"; }
    bool stepEngine() const override { return false; }
};

TEST_F(TestInheritBlackboard, testInheritBlackboard)
{
    // Use inherited blackboards and check if keys are accessible
    ae->start();
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(100));
    std::shared_ptr<alicaTests::TestWorldModel> wm =
            LockedBlackboardRW(ae->editGlobalBlackboard()).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    EXPECT_EQ(wm->passedParameters["masterKeyInBehavior"], 3); // Read a key defined in the master plan inside the behavior
    EXPECT_EQ(wm->passedParameters["hasBehaviorKey"], 4);      // Check that a key defined in the behavior is not available in the master plan
}

TEST_F(TestInheritBlackboard, testInheritBlackboardFlag)
{
    ae->start();
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(100));
    // Behaviour has inheritBlackboard set to false
    EXPECT_TRUE(alica::test::Util::getBasicBehaviour(ae, 831400441334251600, 0)->getInheritBlackboard());
    // Plan has inheritBlackboard set to false
    EXPECT_TRUE(alica::test::Util::getBasicPlan(ae, 1692837668719979400, 0)->getInheritBlackboard());
}

} // namespace
} // namespace alica
