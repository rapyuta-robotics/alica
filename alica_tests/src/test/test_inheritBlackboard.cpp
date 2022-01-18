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
    ae->start();
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(100));
    auto wm = dynamic_cast<alicaTests::TestWorldModel*>(ae->getWorldModel());
    EXPECT_EQ(wm->passedParameters["masterKey"], 3);
    EXPECT_EQ(wm->passedParameters["hasBehaviorKey"], false);
}

} // namespace
} // namespace alica
