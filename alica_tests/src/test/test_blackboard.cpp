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

TEST_F(TestBlackBoard, testRegisterRemoveValue)
{
    Blackboard blackboard;
    LockedBlackboardRW lockedBlackboard = LockedBlackboardRW(blackboard);

    EXPECT_FALSE(lockedBlackboard.hasValue("param"));
    EXPECT_TRUE(lockedBlackboard.empty());
    EXPECT_EQ(lockedBlackboard.size(), 0u);

    lockedBlackboard.registerValue("param", 24);

    EXPECT_FALSE(lockedBlackboard.empty());
    EXPECT_EQ(lockedBlackboard.size(), 1u);
    EXPECT_EQ(lockedBlackboard.get<int>("param"), 24);

    lockedBlackboard.removeValue("param");
    EXPECT_FALSE(lockedBlackboard.hasValue("param"));
    EXPECT_TRUE(lockedBlackboard.empty());
    EXPECT_EQ(lockedBlackboard.size(), 0u);
}

TEST_F(TestBlackBoard, testClearBlackboard)
{
    std::string param3 = "24";
    Blackboard blackboard;
    LockedBlackboardRW lockedBlackboard = LockedBlackboardRW(blackboard);

    EXPECT_FALSE(lockedBlackboard.hasValue("param"));
    EXPECT_TRUE(lockedBlackboard.empty());
    EXPECT_EQ(lockedBlackboard.size(), 0u);

    lockedBlackboard.registerValue("param1", 24);
    lockedBlackboard.registerValue("param2", 1243154151);
    lockedBlackboard.registerValue("param3", param3);
    lockedBlackboard.registerValue("param4", true);

    EXPECT_FALSE(lockedBlackboard.empty());
    EXPECT_EQ(lockedBlackboard.size(), 4u);
    EXPECT_EQ(lockedBlackboard.get<int>("param1"), 24);
    EXPECT_EQ(lockedBlackboard.get<int>("param2"), 1243154151);
    EXPECT_EQ(lockedBlackboard.get<std::string>("param3"), "24");
    EXPECT_TRUE(lockedBlackboard.get<bool>("param4"));

    lockedBlackboard.clear();

    EXPECT_FALSE(lockedBlackboard.hasValue("param1"));
    EXPECT_FALSE(lockedBlackboard.hasValue("param2"));
    EXPECT_FALSE(lockedBlackboard.hasValue("param3"));
    EXPECT_FALSE(lockedBlackboard.hasValue("param4"));
    EXPECT_TRUE(lockedBlackboard.empty());
    EXPECT_EQ(lockedBlackboard.size(), 0u);
}

TEST_F(TestBlackBoard, testReadOnlyAccess)
{
    std::string param3 = "24";
    Blackboard blackboard;
    {
        LockedBlackboardRW lockedBlackboard = LockedBlackboardRW(blackboard);
        lockedBlackboard.registerValue("param1", 24);
        lockedBlackboard.registerValue("param2", 1243154151);
        lockedBlackboard.registerValue("param3", param3);
        lockedBlackboard.registerValue("param4", true);
    }

    LockedBlackboardRO readOnlyBlackboard = LockedBlackboardRO(blackboard);

    EXPECT_FALSE(readOnlyBlackboard.empty());
    EXPECT_EQ(readOnlyBlackboard.size(), 4u);

    EXPECT_TRUE(readOnlyBlackboard.hasValue("param1"));
    EXPECT_TRUE(readOnlyBlackboard.hasValue("param2"));
    EXPECT_TRUE(readOnlyBlackboard.hasValue("param3"));
    EXPECT_TRUE(readOnlyBlackboard.hasValue("param4"));

    EXPECT_EQ(readOnlyBlackboard.get<int>("param1"), 24);
    EXPECT_EQ(readOnlyBlackboard.get<int>("param2"), 1243154151);
    EXPECT_EQ(readOnlyBlackboard.get<std::string>("param3"), "24");
    EXPECT_TRUE(readOnlyBlackboard.get<bool>("param4"));
}
}
} // namespace alica
