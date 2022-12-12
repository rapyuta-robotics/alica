#include "alica_tests/TestWorldModel.h"
#include "test_alica.h"
#include <alica/test/Util.h>
#include <engine/PlanStatus.h>
#include <engine/Types.h>
#include <variant>

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
    auto wm = dynamic_cast<alicaTests::TestWorldModel*>(ae->getWorldModel());
    EXPECT_EQ(wm->passedParameters["behaviorInputKey"], 5);       // Value set in first behavior call
    EXPECT_EQ(wm->passedParameters["behaviorSecondInputKey"], 7); // Value set in second behavior call
}

TEST_F(TestBlackBoard, testJsonPlanKeyMapping)
{
    // Pass values of mapped keys form a plan into another plan
    ae->start();
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(200));
    auto wm = dynamic_cast<alicaTests::TestWorldModel*>(ae->getWorldModel());
    EXPECT_EQ(wm->passedParameters["planInputFromMaster"], 8);
}

TEST_F(TestBlackBoard, testJsonBehaviorKeyMapping)
{
    // Pass values of mapped keys form a plan into a behavior and out of a bahavior into a plan
    ae->start();
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(200));
    auto wm = dynamic_cast<alicaTests::TestWorldModel*>(ae->getWorldModel());
    EXPECT_EQ(wm->passedParameters["behaviorInputKey"], 5); // Value set in plan init -> read in behavior
    EXPECT_EQ(wm->passedParameters["planInputKey"], 6);     // Value set in behavior -> read in plan termination
}

TEST_F(TestBlackBoard, testJsonBlackboardPlan)
{
    // Check if a key defined in json of a plan is accessible
    ae->start();
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(100));
    auto wm = dynamic_cast<alicaTests::TestWorldModel*>(ae->getWorldModel());
    EXPECT_EQ(wm->passedParameters["planKey"], 1);
}

TEST_F(TestBlackBoard, testJsonBlackboardBehavior)
{
    // Check if a key defined in json of a behavior is accessible
    ae->start();
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(100));
    auto wm = dynamic_cast<alicaTests::TestWorldModel*>(ae->getWorldModel());
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

TEST_F(TestBlackBoard, testWithoutPlan)
{
    Blackboard bb;
    LockedBlackboardRW bbrw = LockedBlackboardRW(bb);

    // set default types
    bbrw.set<int64_t>("val1", 14l);
    bbrw.set<double>("val2", 15.3);
    bbrw.set<std::string>("val3", "test");
    bbrw.set<bool>("val4", true);

    // get default types
    EXPECT_EQ(bbrw.get<int64_t>("val1"), 14l);
    EXPECT_EQ(bbrw.get<double>("val2"), 15.3);
    EXPECT_EQ(bbrw.get<std::string>("val3"), "test");
    EXPECT_EQ(bbrw.get<bool>("val4"), true);

    // unknown types
    bbrw.set<PlanStatus>("val5", PlanStatus::Success);
    EXPECT_EQ(bbrw.get<PlanStatus>("val5"), PlanStatus::Success);
}

TEST_F(TestBlackBoard, testMapping)
{
    Blackboard srcBb, targetBb;
    auto srcLocked = LockedBlackboardRW(srcBb);
    auto targetLocked = LockedBlackboardRW(targetBb);

    srcLocked.set<int64_t>("valueInt", 13);
    srcLocked.set<bool>("valueBool", true);

    targetLocked.set<double>("valueDouble", 3.7);
    targetLocked.set<int64_t>("valueInt", 92);

    targetBb.impl().map("valueInt", "valueDouble", srcBb.impl());
    targetBb.impl().map("valueBool", "valueInt", srcBb.impl());

    EXPECT_EQ(targetLocked.get<double>("valueDouble"), 13.0);
    EXPECT_EQ(targetLocked.get<int64_t>("valueInt"), 1);
}

TEST_F(TestBlackBoard, testBoolToBool)
{
    Blackboard srcBb, targetBb;
    auto srcLocked = LockedBlackboardRW(srcBb);
    auto targetLocked = LockedBlackboardRW(targetBb);

    srcLocked.set<bool>("valueSrc", true);
    targetLocked.set<bool>("valueTarget", false);
    targetBb.impl().map("valueSrc", "valueTarget", srcBb.impl());

    EXPECT_EQ(targetLocked.get<bool>("valueTarget"), true);

    srcLocked.set<bool>("valueSrc", false);
    targetBb.impl().map("valueSrc", "valueTarget", srcBb.impl());

    EXPECT_EQ(targetLocked.get<bool>("valueTarget"), false);
}

TEST_F(TestBlackBoard, testBoolToInt64)
{
    Blackboard srcBb, targetBb;
    auto srcLocked = LockedBlackboardRW(srcBb);
    auto targetLocked = LockedBlackboardRW(targetBb);

    srcLocked.set<bool>("valueSrc", true);
    targetLocked.set<int64_t>("valueTarget", -1);
    targetBb.impl().map("valueSrc", "valueTarget", srcBb.impl());

    EXPECT_EQ(targetLocked.get<int64_t>("valueTarget"), 1);

    srcLocked.set<bool>("valueSrc", false);
    targetBb.impl().map("valueSrc", "valueTarget", srcBb.impl());

    EXPECT_EQ(targetLocked.get<int64_t>("valueTarget"), 0);
}

TEST_F(TestBlackBoard, testBoolToDouble)
{
    Blackboard srcBb, targetBb;
    auto srcLocked = LockedBlackboardRW(srcBb);
    auto targetLocked = LockedBlackboardRW(targetBb);

    srcLocked.set<bool>("valueSrc", true);
    targetLocked.set<double>("valueTarget", -1.0);
    targetBb.impl().map("valueSrc", "valueTarget", srcBb.impl());

    EXPECT_EQ(targetLocked.get<double>("valueTarget"), 1.0);

    srcLocked.set<bool>("valueSrc", false);
    targetBb.impl().map("valueSrc", "valueTarget", srcBb.impl());

    EXPECT_EQ(targetLocked.get<double>("valueTarget"), 0.0);
}

TEST_F(TestBlackBoard, testBoolToString)
{
    Blackboard srcBb, targetBb;
    auto srcLocked = LockedBlackboardRW(srcBb);
    auto targetLocked = LockedBlackboardRW(targetBb);

    srcLocked.set<bool>("valueSrc", true);
    targetLocked.set<std::string>("valueTarget", "");

    bool exceptionThrown = false;
    try {
        targetBb.impl().map("valueSrc", "valueTarget", srcBb.impl());
    } catch (const alica::BlackboardException& e) {
        exceptionThrown = true;
        EXPECT_STREQ(e.what(), "Blackboard exception: variant construction failed");
    }

    EXPECT_TRUE(exceptionThrown);
}

TEST_F(TestBlackBoard, testInt64ToBool)
{
    Blackboard srcBb, targetBb;
    auto srcLocked = LockedBlackboardRW(srcBb);
    auto targetLocked = LockedBlackboardRW(targetBb);

    srcLocked.set<int64_t>("valueSrc", 1);
    targetLocked.set<bool>("valueTarget", false);
    targetBb.impl().map("valueSrc", "valueTarget", srcBb.impl());

    EXPECT_EQ(targetLocked.get<bool>("valueTarget"), true);

    srcLocked.set<int64_t>("valueSrc", 0);
    targetBb.impl().map("valueSrc", "valueTarget", srcBb.impl());

    EXPECT_EQ(targetLocked.get<bool>("valueTarget"), false);

    srcLocked.set<int64_t>("valueSrc", -2);
    targetBb.impl().map("valueSrc", "valueTarget", srcBb.impl());

    EXPECT_EQ(targetLocked.get<bool>("valueTarget"), true);
}

TEST_F(TestBlackBoard, testInt64ToInt64)
{
    Blackboard srcBb, targetBb;
    auto srcLocked = LockedBlackboardRW(srcBb);
    auto targetLocked = LockedBlackboardRW(targetBb);

    srcLocked.set<int64_t>("valueSrc", 1);
    targetLocked.set<int64_t>("valueTarget", 0);
    targetBb.impl().map("valueSrc", "valueTarget", srcBb.impl());

    EXPECT_EQ(targetLocked.get<int64_t>("valueTarget"), 1);
}

TEST_F(TestBlackBoard, testInt64ToDouble)
{
    Blackboard srcBb, targetBb;
    auto srcLocked = LockedBlackboardRW(srcBb);
    auto targetLocked = LockedBlackboardRW(targetBb);

    srcLocked.set<int64_t>("valueSrc", 1);
    targetLocked.set<double>("valueTarget", 0.0);
    targetBb.impl().map("valueSrc", "valueTarget", srcBb.impl());

    EXPECT_EQ(targetLocked.get<double>("valueTarget"), 1.0);

    srcLocked.set<int64_t>("valueSrc", -2);
    targetBb.impl().map("valueSrc", "valueTarget", srcBb.impl());

    EXPECT_EQ(targetLocked.get<double>("valueTarget"), -2.0);
}

TEST_F(TestBlackBoard, testInt64ToString)
{
    Blackboard srcBb, targetBb;
    auto srcLocked = LockedBlackboardRW(srcBb);
    auto targetLocked = LockedBlackboardRW(targetBb);

    srcLocked.set<int64_t>("valueSrc", 1);
    targetLocked.set<std::string>("valueTarget", "");

    bool exceptionThrown = false;
    try {
        targetBb.impl().map("valueSrc", "valueTarget", srcBb.impl());
    } catch (const alica::BlackboardException& e) {
        exceptionThrown = true;
        EXPECT_STREQ(e.what(), "Blackboard exception: variant construction failed");
    }

    EXPECT_TRUE(exceptionThrown);
}

TEST_F(TestBlackBoard, testDoubleToBool)
{
    Blackboard srcBb, targetBb;
    auto srcLocked = LockedBlackboardRW(srcBb);
    auto targetLocked = LockedBlackboardRW(targetBb);

    srcLocked.set<double>("valueSrc", 1.0);
    targetLocked.set<bool>("valueTarget", false);
    targetBb.impl().map("valueSrc", "valueTarget", srcBb.impl());

    EXPECT_EQ(targetLocked.get<bool>("valueTarget"), true);

    srcLocked.set<double>("valueSrc", 0.0);
    targetBb.impl().map("valueSrc", "valueTarget", srcBb.impl());

    EXPECT_EQ(targetLocked.get<bool>("valueTarget"), false);

    srcLocked.set<double>("valueSrc", -0.001);
    targetBb.impl().map("valueSrc", "valueTarget", srcBb.impl());

    EXPECT_EQ(targetLocked.get<bool>("valueTarget"), true);
}

TEST_F(TestBlackBoard, testDoubleToInt64)
{
    Blackboard srcBb, targetBb;
    auto srcLocked = LockedBlackboardRW(srcBb);
    auto targetLocked = LockedBlackboardRW(targetBb);

    srcLocked.set<double>("valueSrc", 1.7);
    targetLocked.set<int64_t>("valueTarget", 0);
    targetBb.impl().map("valueSrc", "valueTarget", srcBb.impl());

    EXPECT_EQ(targetLocked.get<int64_t>("valueTarget"), 1);

    srcLocked.set<double>("valueSrc", -5.4);
    targetBb.impl().map("valueSrc", "valueTarget", srcBb.impl());

    EXPECT_EQ(targetLocked.get<int64_t>("valueTarget"), -5);
}

TEST_F(TestBlackBoard, testDoubleToDouble)
{
    Blackboard srcBb, targetBb;
    auto srcLocked = LockedBlackboardRW(srcBb);
    auto targetLocked = LockedBlackboardRW(targetBb);

    srcLocked.set<double>("valueSrc", 1.3);
    targetLocked.set<double>("valueTarget", 0.5);
    targetBb.impl().map("valueSrc", "valueTarget", srcBb.impl());

    EXPECT_EQ(targetLocked.get<double>("valueTarget"), 1.3);
}

TEST_F(TestBlackBoard, testDoubleToString)
{
    Blackboard srcBb, targetBb;
    auto srcLocked = LockedBlackboardRW(srcBb);
    auto targetLocked = LockedBlackboardRW(targetBb);

    srcLocked.set<double>("valueSrc", 1.0);
    targetLocked.set<std::string>("valueTarget", "");

    bool exceptionThrown = false;
    try {
        targetBb.impl().map("valueSrc", "valueTarget", srcBb.impl());
    } catch (const alica::BlackboardException& e) {
        exceptionThrown = true;
        EXPECT_STREQ(e.what(), "Blackboard exception: variant construction failed");
    }

    EXPECT_TRUE(exceptionThrown);
}

TEST_F(TestBlackBoard, testStringToBool)
{
    Blackboard srcBb, targetBb;
    auto srcLocked = LockedBlackboardRW(srcBb);
    auto targetLocked = LockedBlackboardRW(targetBb);

    srcLocked.set<std::string>("valueSrc", "123");
    targetLocked.set<bool>("valueTarget", false);

    bool exceptionThrown = false;
    try {
        targetBb.impl().map("valueSrc", "valueTarget", srcBb.impl());
    } catch (const alica::BlackboardException& e) {
        exceptionThrown = true;
        EXPECT_STREQ(e.what(), "Blackboard exception: variant construction failed");
    }

    EXPECT_TRUE(exceptionThrown);
}

TEST_F(TestBlackBoard, testStringToInt64)
{
    Blackboard srcBb, targetBb;
    auto srcLocked = LockedBlackboardRW(srcBb);
    auto targetLocked = LockedBlackboardRW(targetBb);

    srcLocked.set<std::string>("valueSrc", "123");
    targetLocked.set<int64_t>("valueTarget", 0);

    bool exceptionThrown = false;
    try {
        targetBb.impl().map("valueSrc", "valueTarget", srcBb.impl());
    } catch (const alica::BlackboardException& e) {
        exceptionThrown = true;
        EXPECT_STREQ(e.what(), "Blackboard exception: variant construction failed");
    }

    EXPECT_TRUE(exceptionThrown);
}

TEST_F(TestBlackBoard, testStringToDouble)
{
    Blackboard srcBb, targetBb;
    auto srcLocked = LockedBlackboardRW(srcBb);
    auto targetLocked = LockedBlackboardRW(targetBb);

    srcLocked.set<std::string>("valueSrc", "123");
    targetLocked.set<double>("valueTarget", 0);

    bool exceptionThrown = false;
    try {
        targetBb.impl().map("valueSrc", "valueTarget", srcBb.impl());
    } catch (const alica::BlackboardException& e) {
        exceptionThrown = true;
        EXPECT_STREQ(e.what(), "Blackboard exception: variant construction failed");
    }

    EXPECT_TRUE(exceptionThrown);
}

TEST_F(TestBlackBoard, testStringToString)
{
    Blackboard srcBb, targetBb;
    auto srcLocked = LockedBlackboardRW(srcBb);
    auto targetLocked = LockedBlackboardRW(targetBb);

    srcLocked.set<std::string>("valueSrc", "123");
    targetLocked.set<std::string>("valueTarget", "abc");
    targetBb.impl().map("valueSrc", "valueTarget", srcBb.impl());

    EXPECT_EQ(targetLocked.get<std::string>("valueTarget"), "123");
}

} // namespace
} // namespace alica
