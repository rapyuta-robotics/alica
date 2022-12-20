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

// used for accessing bb values with an unknown type
class UnknownType
{
public:
    UnknownType() = default;
};

class NonConstructable
{
public:
    NonConstructable() = delete;
};

class TestBlackboard : public AlicaTestFixture
{
protected:
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "TestParameterPassingMaster"; }
    bool stepEngine() const override { return false; }
};

TEST_F(TestBlackboard, testJsonTwoBehaviorKeyMapping)
{
    // Two parent values mapped to same behavior input value to differentiate call contexts
    ae->start();
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(200));
    auto wm = dynamic_cast<alicaTests::TestWorldModel*>(ae->getWorldModel());
    EXPECT_EQ(wm->passedParameters["behaviorInputKey"], 5);       // Value set in first behavior call
    EXPECT_EQ(wm->passedParameters["behaviorSecondInputKey"], 7); // Value set in second behavior call
}

TEST_F(TestBlackboard, testJsonPlanKeyMapping)
{
    // Pass values of mapped keys form a plan into another plan
    ae->start();
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(200));
    auto wm = dynamic_cast<alicaTests::TestWorldModel*>(ae->getWorldModel());
    EXPECT_EQ(wm->passedParameters["planInputFromMaster"], 8);
}

TEST_F(TestBlackboard, testJsonBehaviorKeyMapping)
{
    // Pass values of mapped keys form a plan into a behavior and out of a bahavior into a plan
    ae->start();
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(200));
    auto wm = dynamic_cast<alicaTests::TestWorldModel*>(ae->getWorldModel());
    EXPECT_EQ(wm->passedParameters["behaviorInputKey"], 5); // Value set in plan init -> read in behavior
    EXPECT_EQ(wm->passedParameters["planInputKey"], 6);     // Value set in behavior -> read in plan termination
}

TEST_F(TestBlackboard, testJsonBlackboardPlan)
{
    // Check if a key defined in json of a plan is accessible
    ae->start();
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(100));
    auto wm = dynamic_cast<alicaTests::TestWorldModel*>(ae->getWorldModel());
    EXPECT_EQ(wm->passedParameters["planKey"], 1);
}

TEST_F(TestBlackboard, testJsonBlackboardBehavior)
{
    // Check if a key defined in json of a behavior is accessible
    ae->start();
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(100));
    auto wm = dynamic_cast<alicaTests::TestWorldModel*>(ae->getWorldModel());
    EXPECT_EQ(wm->passedParameters["behaviorKey"], 2);
}

TEST_F(TestBlackboard, testNotInheritBlackboardFlag)
{
    ae->start();
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(100));
    // Behaviour has inheritBlackboard set to false
    EXPECT_FALSE(alica::test::Util::getBasicBehaviour(ae, 831400441334251602, 0)->getInheritBlackboard());
    // Plan has inheritBlackboard set to false
    EXPECT_FALSE(alica::test::Util::getBasicPlan(ae, 1692837668719979457, 0)->getInheritBlackboard());
}

TEST_F(TestBlackboard, testWithoutPlan)
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

TEST_F(TestBlackboard, testMapping)
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

TEST_F(TestBlackboard, testBoolToBool)
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

TEST_F(TestBlackboard, testBoolToInt64)
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

TEST_F(TestBlackboard, testBoolToDouble)
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

TEST_F(TestBlackboard, testBoolToString)
{
    Blackboard srcBb, targetBb;
    auto srcLocked = LockedBlackboardRW(srcBb);
    auto targetLocked = LockedBlackboardRW(targetBb);

    srcLocked.set<bool>("valueSrc", true);
    targetLocked.set<std::string>("valueTarget", "");

    EXPECT_THROW(
            {
                try {
                    targetBb.impl().map("valueSrc", "valueTarget", srcBb.impl());
                } catch (const BlackboardException& e) {
                    EXPECT_STREQ(e.what(), "Blackboard exception: variant construction failed");
                    throw;
                }
            },
            BlackboardException);
}

TEST_F(TestBlackboard, testInt64ToBool)
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

TEST_F(TestBlackboard, testInt64ToInt64)
{
    Blackboard srcBb, targetBb;
    auto srcLocked = LockedBlackboardRW(srcBb);
    auto targetLocked = LockedBlackboardRW(targetBb);

    srcLocked.set<int64_t>("valueSrc", 1);
    targetLocked.set<int64_t>("valueTarget", 0);
    targetBb.impl().map("valueSrc", "valueTarget", srcBb.impl());

    EXPECT_EQ(targetLocked.get<int64_t>("valueTarget"), 1);
}

TEST_F(TestBlackboard, testInt64ToDouble)
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

TEST_F(TestBlackboard, testInt64ToString)
{
    Blackboard srcBb, targetBb;
    auto srcLocked = LockedBlackboardRW(srcBb);
    auto targetLocked = LockedBlackboardRW(targetBb);

    srcLocked.set<int64_t>("valueSrc", 1);
    targetLocked.set<std::string>("valueTarget", "");

    EXPECT_THROW(
            {
                try {
                    targetBb.impl().map("valueSrc", "valueTarget", srcBb.impl());
                } catch (const BlackboardException& e) {
                    EXPECT_STREQ(e.what(), "Blackboard exception: variant construction failed");
                    throw;
                }
            },
            BlackboardException);
}

TEST_F(TestBlackboard, testDoubleToBool)
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

TEST_F(TestBlackboard, testDoubleToInt64)
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

TEST_F(TestBlackboard, testDoubleToDouble)
{
    Blackboard srcBb, targetBb;
    auto srcLocked = LockedBlackboardRW(srcBb);
    auto targetLocked = LockedBlackboardRW(targetBb);

    srcLocked.set<double>("valueSrc", 1.3);
    targetLocked.set<double>("valueTarget", 0.5);
    targetBb.impl().map("valueSrc", "valueTarget", srcBb.impl());

    EXPECT_EQ(targetLocked.get<double>("valueTarget"), 1.3);
}

TEST_F(TestBlackboard, testDoubleToString)
{
    Blackboard srcBb, targetBb;
    auto srcLocked = LockedBlackboardRW(srcBb);
    auto targetLocked = LockedBlackboardRW(targetBb);

    srcLocked.set<double>("valueSrc", 1.0);
    targetLocked.set<std::string>("valueTarget", "");

    EXPECT_THROW(
            {
                try {
                    targetBb.impl().map("valueSrc", "valueTarget", srcBb.impl());
                } catch (const BlackboardException& e) {
                    EXPECT_STREQ(e.what(), "Blackboard exception: variant construction failed");
                    throw;
                }
            },
            BlackboardException);
}

TEST_F(TestBlackboard, testStringToBool)
{
    Blackboard srcBb, targetBb;
    auto srcLocked = LockedBlackboardRW(srcBb);
    auto targetLocked = LockedBlackboardRW(targetBb);

    srcLocked.set<std::string>("valueSrc", "123");
    targetLocked.set<bool>("valueTarget", false);

    EXPECT_THROW(
            {
                try {
                    targetBb.impl().map("valueSrc", "valueTarget", srcBb.impl());
                } catch (const BlackboardException& e) {
                    EXPECT_STREQ(e.what(), "Blackboard exception: variant construction failed");
                    throw;
                }
            },
            BlackboardException);
}

TEST_F(TestBlackboard, testStringToInt64)
{
    Blackboard srcBb, targetBb;
    auto srcLocked = LockedBlackboardRW(srcBb);
    auto targetLocked = LockedBlackboardRW(targetBb);

    srcLocked.set<std::string>("valueSrc", "123");
    targetLocked.set<int64_t>("valueTarget", 0);

    EXPECT_THROW(
            {
                try {
                    targetBb.impl().map("valueSrc", "valueTarget", srcBb.impl());
                } catch (const BlackboardException& e) {
                    EXPECT_STREQ(e.what(), "Blackboard exception: variant construction failed");
                    throw;
                }
            },
            BlackboardException);
}

TEST_F(TestBlackboard, testStringToDouble)
{
    Blackboard srcBb, targetBb;
    auto srcLocked = LockedBlackboardRW(srcBb);
    auto targetLocked = LockedBlackboardRW(targetBb);

    srcLocked.set<std::string>("valueSrc", "123");
    targetLocked.set<double>("valueTarget", 0);

    EXPECT_THROW(
            {
                try {
                    targetBb.impl().map("valueSrc", "valueTarget", srcBb.impl());
                } catch (const BlackboardException& e) {
                    EXPECT_STREQ(e.what(), "Blackboard exception: variant construction failed");
                    throw;
                }
            },
            BlackboardException);
}

TEST_F(TestBlackboard, testStringToString)
{
    Blackboard srcBb, targetBb;
    auto srcLocked = LockedBlackboardRW(srcBb);
    auto targetLocked = LockedBlackboardRW(targetBb);

    srcLocked.set<std::string>("valueSrc", "123");
    targetLocked.set<std::string>("valueTarget", "abc");
    targetBb.impl().map("valueSrc", "valueTarget", srcBb.impl());

    EXPECT_EQ(targetLocked.get<std::string>("valueTarget"), "123");
}

TEST_F(TestBlackboard, testAccessingWithWrongType)
{
    Blackboard bb;
    auto bb_locked = LockedBlackboardRW(bb);
    bb_locked.set<int64_t>("value", 1);

    EXPECT_THROW(
            {
                try {
                    bb_locked.get<double>("value"); // try to access with a type we can cast to
                } catch (const BlackboardException& e) {
                    EXPECT_STREQ(e.what(), "Blackboard exception: get() type mismatch, key: value, setType: int64, getType: double");
                    throw;
                }
            },
            BlackboardException);

    EXPECT_THROW(
            {
                try {
                    bb_locked.get<std::string>("value"); // try to access with a type we cant cast to
                } catch (const BlackboardException& e) {
                    EXPECT_STREQ(e.what(), "Blackboard exception: get() type mismatch, key: value, setType: int64, getType: std::string");
                    throw;
                }
            },
            BlackboardException);
}

TEST_F(TestBlackboard, testAccessingWithNonExistingKey)
{
    Blackboard bb;
    auto bb_locked = LockedBlackboardRW(bb);

    EXPECT_THROW(
            {
                try {
                    bb_locked.get<double>("value");
                } catch (const BlackboardException& e) {
                    EXPECT_STREQ(e.what(), "Blackboard exception: get() failure, key: value is not yet set, so cannot get it");
                    throw;
                }
            },
            BlackboardException);
}

TEST_F(TestBlackboard, testAccessUnknownTypeWithKnownWrongType)
{
    Blackboard bb;
    auto bb_locked = LockedBlackboardRW(bb);
    bb_locked.set<PlanStatus>("value", PlanStatus::Success);

    EXPECT_THROW(
            {
                try {
                    bb_locked.get<double>("value");
                } catch (const BlackboardException& e) {
                    EXPECT_STREQ(e.what(), "Blackboard exception: get() type mismatch, key: value, setType: std::any, getType: double");
                    throw;
                }
            },
            BlackboardException);
}

TEST_F(TestBlackboard, testAccessUnknownTypeWithUnknownWrongType)
{
    Blackboard bb;
    auto bb_locked = LockedBlackboardRW(bb);
    bb_locked.set<PlanStatus>("value", PlanStatus::Success);

    EXPECT_THROW(
            {
                try {
                    bb_locked.get<UnknownType>("value");
                } catch (const BlackboardException& e) {
                    EXPECT_STREQ(e.what(), "Blackboard exception: get() type mismatch, key: value, setType: unknown, getType: unknown");
                    throw;
                }
            },
            BlackboardException);
}

TEST_F(TestBlackboard, testSetNotMatchingKnownType)
{
    std::unique_ptr<alica::BlackboardBlueprint> blueprint = std::make_unique<alica::BlackboardBlueprint>();
    blueprint->addKey("intVal", "int64", "14");
    blueprint->addKey("doubleVal", "double", "2.7");
    blueprint->addKey("boolVal", "bool", "true");
    blueprint->addKey("stringVal", "std::string", "test string");
    alica::Blackboard blackboard = alica::Blackboard(blueprint.get());
    alica::LockedBlackboardRW bb = LockedBlackboardRW(blackboard);

    EXPECT_EQ(bb.get<int64_t>("intVal"), 14);
    EXPECT_EQ(bb.get<double>("doubleVal"), 2.7);
    EXPECT_EQ(bb.get<bool>("boolVal"), true);
    EXPECT_EQ(bb.get<std::string>("stringVal"), "test string");

    EXPECT_THROW(
            {
                try {
                    bb.set<double>("intVal", 2.7);
                } catch (const BlackboardException& e) {
                    EXPECT_STREQ(e.what(), "Blackboard exception: set() type mismatch, key: intVal, set type: double, expected type (from yaml file): int64");
                    throw;
                }
            },
            BlackboardException);

    EXPECT_THROW(
            {
                try {
                    bb.set<std::string>("doubleVal", "test");
                } catch (const BlackboardException& e) {
                    EXPECT_STREQ(e.what(),
                            "Blackboard exception: set() type mismatch, key: doubleVal, set type: std::string, expected type (from yaml file): double");
                    throw;
                }
            },
            BlackboardException);

    EXPECT_THROW(
            {
                try {
                    bb.set<std::string>("boolVal", "test");
                } catch (const BlackboardException& e) {
                    EXPECT_STREQ(
                            e.what(), "Blackboard exception: set() type mismatch, key: boolVal, set type: std::string, expected type (from yaml file): bool");
                    throw;
                }
            },
            BlackboardException);

    EXPECT_THROW(
            {
                try {
                    bb.set<int64_t>("stringVal", 0);
                } catch (const BlackboardException& e) {
                    EXPECT_STREQ(e.what(),
                            "Blackboard exception: set() type mismatch, key: stringVal, set type: int64, expected type (from yaml file): std::string");
                    throw;
                }
            },
            BlackboardException);
}

TEST_F(TestBlackboard, testSetNotMatchingUnknownType)
{
    Blackboard bb;
    auto bb_locked = LockedBlackboardRW(bb);
    bb_locked.set<PlanStatus>("value", PlanStatus::Success);
    EXPECT_EQ(bb_locked.get<PlanStatus>("value"), PlanStatus::Success);

    bb_locked.set<UnknownType>("value", UnknownType());
    EXPECT_EQ(typeid(UnknownType), typeid(bb_locked.get<UnknownType>("value")));
}

TEST_F(TestBlackboard, testSetWithDifferentTypeNotInPML)
{
    Blackboard bb;
    auto bb_locked = LockedBlackboardRW(bb);
    bb_locked.set<int64_t>("value", 1);
    EXPECT_EQ(bb_locked.get<int64_t>("value"), 1);

    bb_locked.set<double>("value", 2.0);
    EXPECT_EQ(bb_locked.get<double>("value"), 2.0);
}

TEST_F(TestBlackboard, testSetAnyWithDifferentType)
{
    Blackboard bb;
    auto bb_locked = LockedBlackboardRW(bb);
    bb_locked.set<alica::PlanStatus>("value", alica::PlanStatus::Success);
    EXPECT_EQ(bb_locked.get<alica::PlanStatus>("value"), alica::PlanStatus::Success);

    bb_locked.set<UnknownType>("value", UnknownType());
    EXPECT_EQ(typeid(UnknownType), typeid(bb_locked.get<UnknownType>("value")));

    bb_locked.set<int64_t>("value", 2);
    EXPECT_EQ(bb_locked.get<int64_t>("value"), 2);
}

TEST_F(TestBlackboard, testInitWithUnsupportedType)
{
    std::unique_ptr<alica::BlackboardBlueprint> blueprint = std::make_unique<alica::BlackboardBlueprint>();
    blueprint->addKey("unknown", "UnknownType", std::nullopt);
    EXPECT_THROW(
            {
                try {
                    alica::Blackboard(blueprint.get());
                } catch (const BlackboardException& e) {
                    EXPECT_STREQ(e.what(), "Blackboard exception: key: unknown has an unsupported type");
                    throw;
                }
            },
            BlackboardException);
}

TEST_F(TestBlackboard, testInitWithDefaultValueForAny)
{
    std::unique_ptr<alica::BlackboardBlueprint> blueprint = std::make_unique<alica::BlackboardBlueprint>();
    blueprint->addKey("anyWithDefault", "std::any", "defaultValue");
    EXPECT_THROW(
            {
                try {
                    alica::Blackboard(blueprint.get());
                } catch (const BlackboardException& e) {
                    EXPECT_STREQ(e.what(), "Blackboard exception: key: anyWithDefault cannot have a default value because it is of type std::any");
                    throw;
                }
            },
            BlackboardException);
}

TEST_F(TestBlackboard, testInitWithWrongDefaultValueType)
{
    std::unique_ptr<alica::BlackboardBlueprint> blueprint = std::make_unique<alica::BlackboardBlueprint>();
    blueprint->addKey("anyWithDefault", "int64", "defaultValue");
    EXPECT_THROW(
            {
                try {
                    alica::Blackboard(blueprint.get());
                } catch (const BlackboardException& e) {
                    EXPECT_STREQ(e.what(), "Blackboard exception: Could not set key: anyWithDefault, from default value: defaultValue, type: int64, details: "
                                           "yaml-cpp: error at line 1, column 1: bad conversion");
                    throw;
                }
            },
            BlackboardException);
}

TEST_F(TestBlackboard, testInitWithNonConstructable)
{
    std::unique_ptr<alica::BlackboardBlueprint> blueprint = std::make_unique<alica::BlackboardBlueprint>();
    blueprint->addKey("anyWithDefault", "NonConstructable", std::nullopt);
    EXPECT_THROW(
            {
                try {
                    alica::Blackboard(blueprint.get());
                } catch (const BlackboardException& e) {
                    EXPECT_STREQ(e.what(), "Blackboard exception: key: anyWithDefault has an unsupported type");
                    throw;
                }
            },
            BlackboardException);
}

TEST_F(TestBlackboard, testInitWithDefaultValue)
{
    std::unique_ptr<alica::BlackboardBlueprint> blueprint = std::make_unique<alica::BlackboardBlueprint>();
    blueprint->addKey("intVal", "int64", "14");
    blueprint->addKey("doubleVal", "double", "2.7");
    blueprint->addKey("boolVal", "bool", "true");
    blueprint->addKey("stringVal", "std::string", "test string");
    alica::Blackboard blackboard = alica::Blackboard(blueprint.get());
    alica::LockedBlackboardRO bb = LockedBlackboardRO(blackboard);

    EXPECT_EQ(bb.get<int64_t>("intVal"), 14);
    EXPECT_EQ(bb.get<double>("doubleVal"), 2.7);
    EXPECT_EQ(bb.get<bool>("boolVal"), true);
    EXPECT_EQ(bb.get<std::string>("stringVal"), "test string");
}

TEST_F(TestBlackboard, testInitWithoutDefaultValue)
{
    std::unique_ptr<alica::BlackboardBlueprint> blueprint = std::make_unique<alica::BlackboardBlueprint>();
    blueprint->addKey("intVal", "int64", std::nullopt);
    blueprint->addKey("doubleVal", "double", std::nullopt);
    blueprint->addKey("boolVal", "bool", std::nullopt);
    blueprint->addKey("stringVal", "std::string", std::nullopt);
    alica::Blackboard blackboard = alica::Blackboard(blueprint.get());
    alica::LockedBlackboardRO bb = LockedBlackboardRO(blackboard);

    EXPECT_EQ(bb.get<int64_t>("intVal"), 0);
    EXPECT_EQ(bb.get<double>("doubleVal"), 0.0);
    EXPECT_EQ(bb.get<bool>("boolVal"), false);
    EXPECT_EQ(bb.get<std::string>("stringVal"), "");
}

TEST_F(TestBlackboard, testInitWithConvertibleIntDefaultValue)
{
    std::unique_ptr<alica::BlackboardBlueprint> blueprint = std::make_unique<alica::BlackboardBlueprint>();
    blueprint->addKey("intVal", "int64", "14.7");

    EXPECT_THROW(
            {
                try {
                    alica::Blackboard(blueprint.get());
                } catch (const BlackboardException& e) {
                    EXPECT_STREQ(e.what(), "Blackboard exception: Could not set key: intVal, from default value: 14.7, type: int64, details: yaml-cpp: error "
                                           "at line 1, column 1: bad conversion");
                    throw;
                }
            },
            BlackboardException);
}

TEST_F(TestBlackboard, testInitWithConvertibleBoolDefaultValue)
{
    std::unique_ptr<alica::BlackboardBlueprint> blueprint = std::make_unique<alica::BlackboardBlueprint>();
    blueprint->addKey("boolVal", "bool", "1");

    EXPECT_THROW(
            {
                try {
                    alica::Blackboard(blueprint.get());
                } catch (const BlackboardException& e) {
                    EXPECT_STREQ(e.what(), "Blackboard exception: Could not set key: boolVal, from default value: 1, type: bool, details: yaml-cpp: error at "
                                           "line 1, column 1: bad conversion");
                    throw;
                }
            },
            BlackboardException);
}

TEST_F(TestBlackboard, testInitWithConvertibleDoubleDefaultValue)
{
    std::unique_ptr<alica::BlackboardBlueprint> blueprint = std::make_unique<alica::BlackboardBlueprint>();
    blueprint->addKey("doubleVal", "double", "false");

    EXPECT_THROW(
            {
                try {
                    alica::Blackboard(blueprint.get());
                } catch (const BlackboardException& e) {
                    EXPECT_STREQ(e.what(), "Blackboard exception: Could not set key: doubleVal, from default value: false, type: double, details: yaml-cpp: "
                                           "error at line 1, column 1: bad conversion");
                    throw;
                }
            },
            BlackboardException);
}

} // namespace
} // namespace alica
