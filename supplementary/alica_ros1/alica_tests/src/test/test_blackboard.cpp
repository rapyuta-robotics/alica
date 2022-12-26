#include "alica_tests/TestWorldModel.h"
#include "test_alica.h"
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

TEST_F(TestBlackBoard, testJsonTwoBehaviorKeyMapping)
{
    // Two parent values mapped to same behavior input value to differentiate call contexts
    ae->start();
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(200));

    alicaTests::TestWorldModel* wm = LockedBlackboardRW(ae->editGlobalBlackboard()).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel").get();
    EXPECT_EQ(wm->passedParameters["behaviorInputKey"], 5);       // Value set in first behavior call
    EXPECT_EQ(wm->passedParameters["behaviorSecondInputKey"], 7); // Value set in second behavior call
}

TEST_F(TestBlackBoard, testJsonPlanKeyMapping)
{
    // Pass values of mapped keys form a plan into another plan
    ae->start();
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(200));
    alicaTests::TestWorldModel* wm = LockedBlackboardRW(ae->editGlobalBlackboard()).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel").get();
    EXPECT_EQ(wm->passedParameters["planInputFromMaster"], 8);
}

TEST_F(TestBlackBoard, testJsonBehaviorKeyMapping)
{
    // Pass values of mapped keys form a plan into a behavior and out of a bahavior into a plan
    ae->start();
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(200));
    alicaTests::TestWorldModel* wm = LockedBlackboardRW(ae->editGlobalBlackboard()).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel").get();
    EXPECT_EQ(wm->passedParameters["behaviorInputKey"], 5); // Value set in plan init -> read in behavior
    EXPECT_EQ(wm->passedParameters["planInputKey"], 6);     // Value set in behavior -> read in plan termination
}

TEST_F(TestBlackBoard, testJsonBlackboardPlan)
{
    // Check if a key defined in json of a plan is accessible
    ae->start();
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(100));
    alicaTests::TestWorldModel* wm = LockedBlackboardRW(ae->editGlobalBlackboard()).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel").get();
    EXPECT_EQ(wm->passedParameters["planKey"], 1);
}

TEST_F(TestBlackBoard, testJsonBlackboardBehavior)
{
    // Check if a key defined in json of a behavior is accessible
    ae->start();
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(100));
    alicaTests::TestWorldModel* wm = LockedBlackboardRW(ae->editGlobalBlackboard()).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel").get();
    EXPECT_EQ(wm->passedParameters["behaviorKey"], 2);
}

TEST_F(TestBlackBoard, testNotInheritBlackboardFlag)
{
    ae->start();
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(100));
    // Behaviour has inheritBlackboard set to false
    EXPECT_FALSE(alica::test::Util::getBasicBehaviour(ae, 831400441334251602, 0)->getInheritBlackboard());
    // Plan has inheritBlackboard set to false
    EXPECT_FALSE(alica::test::Util::getBasicPlan(ae, 1692837668719979457, 0)->getInheritBlackboard());
}

} // namespace
} // namespace alica
