#include "test_alica.h"
#include "alica_tests/TestWorldModel.h"
#include <alica/test/Util.h>

namespace alica
{
namespace
{

class TestBlackBoard : public AlicaTestFixture
{
protected:
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "TestParameterPassingMaster"; }
    bool stepEngine() const override { return false; }
};

TEST_F(TestBlackBoard, testParameterPassing)
{
    ae->start();
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(100));
    auto wm = dynamic_cast<alicaTests::TestWorldModel*>(ae->getWorldModel());
    EXPECT_EQ(wm->passedParameters["behaviourParameter"], 1);
    EXPECT_EQ(wm->passedParameters["planParameter"], 2);
}

TEST_F(TestBlackBoard, testRequiresParameters) 
{
    ae->start();
    auto wm = dynamic_cast<alicaTests::TestWorldModel*>(ae->getWorldModel());
    // Behaviour has requiresParameters set to true
    EXPECT_TRUE(alica::test::Util::getBasicBehaviour(ae, 831400441334251602, 0)->getRequiresParameters());
    // SubPlan has requiresParameters set to true
    EXPECT_TRUE(alica::test::Util::getBasicPlan(ae, 1692837668719979457, 0)->getRequiresParameters());
}
}
} // namespace alica

