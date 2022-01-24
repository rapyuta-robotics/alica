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
    // Todo check if modified values are available
    ae->start();
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(100));
    auto wm = dynamic_cast<alicaTests::TestWorldModel*>(ae->getWorldModel());
    EXPECT_EQ(wm->passedParameters["masterKey"], 3);
    EXPECT_EQ(wm->passedParameters["hasBehaviorKey"], false);
}

TEST_F(TestInheritBlackboard, testInheritBlackboardFlag)
{
    ae->start();
    // Behaviour has inheritBlackboard set to false
    EXPECT_TRUE(alica::test::Util::getBasicBehaviour(ae, 831400441334251600, 0)->getInheritBlackboard());
    // Plan has inheritBlackboard set to false
    EXPECT_TRUE(alica::test::Util::getBasicPlan(ae, 1692837668719979400, 0)->getInheritBlackboard());
}

} // namespace
} // namespace alica
