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

constexpr const char* BOOL = "bool";
constexpr const char* INT64 = "int64";
constexpr const char* DOUBLE = "double";
constexpr const char* STRING = "std::string";
constexpr const char* ANY = "std::any";

// used for accessing bb values with an unknown type
class UnknownType
{
public:
    UnknownType() = default;
    UnknownType(int64_t input): value(input) {};
    int64_t value;
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

    template <class targetType>
    bool checkMapping(BlackboardImpl& srcBB, BlackboardImpl& targetBB, const std::string& srcKey, const std::string& targetKey, targetType targetValue)
    {
        targetBB.map(srcKey, targetKey, srcBB);
        return targetBB.get<targetType>(targetKey) == targetValue;
    }
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

TEST_F(TestBlackboard, testMappingFromBool)
{
    Blackboard srcBb, targetBb;
    auto srcLocked = LockedBlackboardRW(srcBb);
    auto targetLocked = LockedBlackboardRW(targetBb);

    srcLocked.set<bool>("valueSrc", true);
    targetLocked.set<bool>("valueTarget", false);
    EXPECT_TRUE((checkMapping<bool>(srcBb.impl(), targetBb.impl(), "valueSrc", "valueTarget", true)));

    targetLocked.set<int64_t>("valueTarget", -1);
    EXPECT_TRUE((checkMapping<int64_t>(srcBb.impl(), targetBb.impl(), "valueSrc", "valueTarget", 1)));

    targetLocked.set<double>("valueTarget", -1.0);
    EXPECT_TRUE((checkMapping<double>(srcBb.impl(), targetBb.impl(), "valueSrc", "valueTarget", 1.0)));

    targetLocked.set<std::string>("valueTarget", "");
    EXPECT_THROW(({ checkMapping<std::string>(srcBb.impl(), targetBb.impl(), "valueSrc", "valueTarget", ""); }), BlackboardException);

    srcLocked.set<bool>("valueSrc", false);
    targetLocked.set<bool>("valueTarget", true);
    EXPECT_TRUE((checkMapping<bool>(srcBb.impl(), targetBb.impl(), "valueSrc", "valueTarget", false)));
    
    targetLocked.set<int64_t>("valueTarget", 1);
    EXPECT_TRUE((checkMapping<int64_t>(srcBb.impl(), targetBb.impl(), "valueSrc", "valueTarget", 0)));
    
    targetLocked.set<double>("valueTarget", 1.0);
    EXPECT_TRUE((checkMapping<double>(srcBb.impl(), targetBb.impl(), "valueSrc", "valueTarget", 0)));
    
    targetLocked.set<std::string>("valueTarget", "");
    EXPECT_THROW(({ checkMapping<std::string>(srcBb.impl(), targetBb.impl(), "valueSrc", "valueTarget", ""); }), BlackboardException);
}

TEST_F(TestBlackboard, testMappingFromInt64)
{
    Blackboard srcBb, targetBb;
    auto srcLocked = LockedBlackboardRW(srcBb);
    auto targetLocked = LockedBlackboardRW(targetBb);

    srcLocked.set<int64_t>("valueSrc", 1);
    targetLocked.set<bool>("valueTarget", false);
    EXPECT_TRUE((checkMapping<bool>(srcBb.impl(), targetBb.impl(), "valueSrc", "valueTarget", true)));

    targetLocked.set<int64_t>("valueTarget", 0);
    EXPECT_TRUE((checkMapping<int64_t>(srcBb.impl(), targetBb.impl(), "valueSrc", "valueTarget", 1)));

    targetLocked.set<double>("valueTarget", 0.0);
    EXPECT_TRUE((checkMapping<double>(srcBb.impl(), targetBb.impl(), "valueSrc", "valueTarget", 1.0)));

    targetLocked.set<std::string>("valueTarget", "");
    EXPECT_THROW(({ checkMapping<std::string>(srcBb.impl(), targetBb.impl(), "valueSrc", "valueTarget", ""); }), BlackboardException);

    srcLocked.set<int64_t>("valueSrc", -1);
    targetLocked.set<bool>("valueTarget", false);
    EXPECT_TRUE((checkMapping<bool>(srcBb.impl(), targetBb.impl(), "valueSrc", "valueTarget", true)));

    targetLocked.set<int64_t>("valueTarget", 0);
    EXPECT_TRUE((checkMapping<int64_t>(srcBb.impl(), targetBb.impl(), "valueSrc", "valueTarget", -1)));

    targetLocked.set<double>("valueTarget", 0.0);
    EXPECT_TRUE((checkMapping<double>(srcBb.impl(), targetBb.impl(), "valueSrc", "valueTarget", -1.0)));

    targetLocked.set<std::string>("valueTarget", "");
    EXPECT_THROW(({ checkMapping<std::string>(srcBb.impl(), targetBb.impl(), "valueSrc", "valueTarget", ""); }), BlackboardException);

    srcLocked.set<int64_t>("valueSrc", 0);
    targetLocked.set<bool>("valueTarget", true);
    EXPECT_TRUE((checkMapping<bool>(srcBb.impl(), targetBb.impl(), "valueSrc", "valueTarget", false)));

    targetLocked.set<int64_t>("valueTarget", -1);
    EXPECT_TRUE((checkMapping<int64_t>(srcBb.impl(), targetBb.impl(), "valueSrc", "valueTarget", 0)));

    targetLocked.set<double>("valueTarget", -1.0);
    EXPECT_TRUE((checkMapping<double>(srcBb.impl(), targetBb.impl(), "valueSrc", "valueTarget", 0.0)));

    targetLocked.set<std::string>("valueTarget", "");
    EXPECT_THROW(({ checkMapping<std::string>(srcBb.impl(), targetBb.impl(), "valueSrc", "valueTarget", ""); }), BlackboardException);
}

TEST_F(TestBlackboard, testMappingFromDouble)
{
    Blackboard srcBb, targetBb;
    auto srcLocked = LockedBlackboardRW(srcBb);
    auto targetLocked = LockedBlackboardRW(targetBb);

    srcLocked.set<double>("valueSrc", 1.0);
    targetLocked.set<bool>("valueTarget", false);
    EXPECT_TRUE((checkMapping<bool>(srcBb.impl(), targetBb.impl(), "valueSrc", "valueTarget", true)));

    targetLocked.set<int64_t>("valueTarget", int64_t());
    EXPECT_TRUE((checkMapping<int64_t>(srcBb.impl(), targetBb.impl(), "valueSrc", "valueTarget", 1)));

    targetLocked.set<double>("valueTarget", double());
    EXPECT_TRUE((checkMapping<double>(srcBb.impl(), targetBb.impl(), "valueSrc", "valueTarget", 1.0)));

    targetLocked.set<std::string>("valueTarget", "");
    EXPECT_THROW(({ checkMapping<std::string>(srcBb.impl(), targetBb.impl(), "valueSrc", "valueTarget", ""); }), BlackboardException);

    srcLocked.set<double>("valueSrc", double());
    targetLocked.set<bool>("valueTarget", true);
    EXPECT_TRUE((checkMapping<bool>(srcBb.impl(), targetBb.impl(), "valueSrc", "valueTarget", false)));

    targetLocked.set<int64_t>("valueTarget", int64_t());
    EXPECT_TRUE((checkMapping<int64_t>(srcBb.impl(), targetBb.impl(), "valueSrc", "valueTarget", 0)));

    targetLocked.set<double>("valueTarget", double());
    EXPECT_TRUE((checkMapping<double>(srcBb.impl(), targetBb.impl(), "valueSrc", "valueTarget", 0.0)));

    targetLocked.set<std::string>("valueTarget", "");
    EXPECT_THROW(({ checkMapping<std::string>(srcBb.impl(), targetBb.impl(), "valueSrc", "valueTarget", ""); }), BlackboardException);

    srcLocked.set<double>("valueSrc", -0.001);
    targetLocked.set<bool>("valueTarget", false);
    EXPECT_TRUE((checkMapping<bool>(srcBb.impl(), targetBb.impl(), "valueSrc", "valueTarget", true)));

    targetLocked.set<int64_t>("valueTarget", -1);
    EXPECT_TRUE((checkMapping<int64_t>(srcBb.impl(), targetBb.impl(), "valueSrc", "valueTarget", 0)));

    targetLocked.set<double>("valueTarget", 1);
    EXPECT_TRUE((checkMapping<double>(srcBb.impl(), targetBb.impl(), "valueSrc", "valueTarget", -0.001)));

    targetLocked.set<std::string>("valueTarget", "");
    EXPECT_THROW(({ checkMapping<std::string>(srcBb.impl(), targetBb.impl(), "valueSrc", "valueTarget", ""); }), BlackboardException);
}

TEST_F(TestBlackboard, testMappingFromString)
{
    Blackboard srcBb, targetBb;
    auto srcLocked = LockedBlackboardRW(srcBb);
    auto targetLocked = LockedBlackboardRW(targetBb);

    srcLocked.set<std::string>("valueSrc", "123");

    targetLocked.set<bool>("valueTarget", false);
    // throw exception because string can not be mapped to bool
    EXPECT_THROW(({ checkMapping<bool>(srcBb.impl(), targetBb.impl(), "valueSrc", "valueTarget", true); }), BlackboardException);

    targetLocked.set<int64_t>("valueTarget", int64_t());
    // throw exception because string can not be mapped to int
    EXPECT_THROW(({ checkMapping<int64_t>(srcBb.impl(), targetBb.impl(), "valueSrc", "valueTarget", 1); }), BlackboardException);

    targetLocked.set<double>("valueTarget", double());
    // throw exception because string can not be mapped to double
    EXPECT_THROW(({ checkMapping<double>(srcBb.impl(), targetBb.impl(), "valueSrc", "valueTarget", 1.0); }), BlackboardException);

    srcLocked.set<std::string>("valueSrc", "123");
    targetLocked.set<std::string>("valueTarget", "abc");
    EXPECT_TRUE((checkMapping<std::string>(srcBb.impl(), targetBb.impl(), "valueSrc", "valueTarget", "123")));
}

TEST_F(TestBlackboard, testAccessingWithWrongType)
{
    Blackboard bb;
    auto bb_locked = LockedBlackboardRW(bb);
    bb_locked.set<int64_t>("value", 1);

    // throw exception because double does not match int64, even though we can cast int to double
    EXPECT_THROW({ bb_locked.get<double>("value"); }, BlackboardException);

    // throw exception because string does not match int64
    EXPECT_THROW({ bb_locked.get<std::string>("value"); }, BlackboardException);
}

TEST_F(TestBlackboard, testAccessingWithNonExistingKey)
{
    Blackboard bb;
    auto bb_locked = LockedBlackboardRW(bb);

    // throw exception because key does not exist
    EXPECT_THROW({ bb_locked.get<double>("value"); }, BlackboardException);
}

TEST_F(TestBlackboard, testAccessUnknownTypeWithKnownWrongType)
{
    Blackboard bb;
    auto bb_locked = LockedBlackboardRW(bb);
    bb_locked.set<PlanStatus>("value", PlanStatus::Success);

    // throw exception because type double doesnt match PlanStatus
    EXPECT_THROW({ bb_locked.get<double>("value"); }, BlackboardException);
}

TEST_F(TestBlackboard, testAccessUnknownTypeWithUnknownWrongType)
{
    Blackboard bb;
    auto bb_locked = LockedBlackboardRW(bb);
    bb_locked.set<PlanStatus>("value", PlanStatus::Success);

    // throw exception because UnknownType does not match PlanStatus
    EXPECT_THROW({ bb_locked.get<UnknownType>("value"); }, BlackboardException);
}

TEST_F(TestBlackboard, testSetNotMatchingKnownType)
{
    std::unique_ptr<alica::BlackboardBlueprint> blueprint = std::make_unique<alica::BlackboardBlueprint>();
    blueprint->addKey("intVal", INT64, "14");
    blueprint->addKey("doubleVal", DOUBLE, "2.7");
    blueprint->addKey("boolVal", BOOL, "true");
    blueprint->addKey("stringVal", STRING, "test string");
    alica::Blackboard blackboard = alica::Blackboard(blueprint.get());
    alica::LockedBlackboardRW bb = LockedBlackboardRW(blackboard);

    EXPECT_EQ(bb.get<int64_t>("intVal"), 14);
    EXPECT_EQ(bb.get<double>("doubleVal"), 2.7);
    EXPECT_EQ(bb.get<bool>("boolVal"), true);
    EXPECT_EQ(bb.get<std::string>("stringVal"), "test string");

    // throw exception because value is not of type double, but int
    EXPECT_THROW({ bb.set<double>("intVal", 2.7); }, BlackboardException);

    // throw exception because value is not of type string, but double
    EXPECT_THROW({ bb.set<std::string>("doubleVal", "test"); }, BlackboardException);

    // throw exception because value is not of type string, but bool
    EXPECT_THROW({ bb.set<std::string>("boolVal", "test"); }, BlackboardException);

    // throw exception because value is not of type int64, but string
    EXPECT_THROW({ bb.set<int64_t>("stringVal", 0); }, BlackboardException);
}

TEST_F(TestBlackboard, testSetNotMatchingUnknownType)
{
    Blackboard bb;
    auto bb_locked = LockedBlackboardRW(bb);
    bb_locked.set<PlanStatus>("value", PlanStatus::Success);
    EXPECT_EQ(bb_locked.get<PlanStatus>("value"), PlanStatus::Success);

    bb_locked.set<UnknownType>("value", UnknownType(185));
    EXPECT_EQ(185, bb_locked.get<UnknownType>("value").value);
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

    bb_locked.set<UnknownType>("value", UnknownType(512));
    EXPECT_EQ(bb_locked.get<UnknownType>("value").value, 512);

    bb_locked.set<int64_t>("value", 2);
    EXPECT_EQ(bb_locked.get<int64_t>("value"), 2);
}

TEST_F(TestBlackboard, testInitWithUnsupportedType)
{
    std::unique_ptr<alica::BlackboardBlueprint> blueprint = std::make_unique<alica::BlackboardBlueprint>();
    blueprint->addKey("unknown", "UnknownType", std::nullopt);
    // throw exception because UnknownType is not supported and can not be used as a default value type
    EXPECT_THROW({ alica::Blackboard(blueprint.get()); }, BlackboardException);
}

TEST_F(TestBlackboard, testInitWithDefaultValueForAny)
{
    std::unique_ptr<alica::BlackboardBlueprint> blueprint = std::make_unique<alica::BlackboardBlueprint>();
    blueprint->addKey("anyWithDefault", ANY, "defaultValue");
    // throw exception because std::any type can not have a default value
    EXPECT_THROW({ alica::Blackboard(blueprint.get()); }, BlackboardException);
}

TEST_F(TestBlackboard, testInitWithWrongDefaultValueType)
{
    std::unique_ptr<alica::BlackboardBlueprint> blueprint = std::make_unique<alica::BlackboardBlueprint>();
    blueprint->addKey("anyWithDefault", INT64, "defaultValue");
    // throw exception because "defaultValue" is not of type int64
    EXPECT_THROW({ alica::Blackboard(blueprint.get()); }, BlackboardException);
}

TEST_F(TestBlackboard, testInitWithNonConstructable)
{
    std::unique_ptr<alica::BlackboardBlueprint> blueprint = std::make_unique<alica::BlackboardBlueprint>();
    blueprint->addKey("anyWithDefault", "NonConstructable", std::nullopt);
    // throw exception because NonConstructable is not constructable with a default value
    EXPECT_THROW({ alica::Blackboard(blueprint.get()); }, BlackboardException);
}

TEST_F(TestBlackboard, testInitWithDefaultValue)
{
    std::unique_ptr<alica::BlackboardBlueprint> blueprint = std::make_unique<alica::BlackboardBlueprint>();
    blueprint->addKey("intVal", INT64, "14");
    blueprint->addKey("doubleVal", DOUBLE, "2.7");
    blueprint->addKey("boolVal", BOOL, "true");
    blueprint->addKey("stringVal", STRING, "test string");
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
    blueprint->addKey("intVal", INT64, std::nullopt);
    blueprint->addKey("doubleVal", DOUBLE, std::nullopt);
    blueprint->addKey("boolVal", BOOL, std::nullopt);
    blueprint->addKey("stringVal", STRING, std::nullopt);
    alica::Blackboard blackboard = alica::Blackboard(blueprint.get());
    alica::LockedBlackboardRO bb = LockedBlackboardRO(blackboard);

    EXPECT_EQ(bb.get<int64_t>("intVal"), int64_t());
    EXPECT_EQ(bb.get<double>("doubleVal"), double());
    EXPECT_EQ(bb.get<bool>("boolVal"), false);
    EXPECT_EQ(bb.get<std::string>("stringVal"), "");
}

TEST_F(TestBlackboard, testInitWithConvertibleIntDefaultValue)
{
    std::unique_ptr<alica::BlackboardBlueprint> blueprint = std::make_unique<alica::BlackboardBlueprint>();
    blueprint->addKey("intVal", INT64, "14.7");
    // throw exception because "14.7" is not of type int64
    EXPECT_THROW({ alica::Blackboard(blueprint.get()); }, BlackboardException);
}

TEST_F(TestBlackboard, testInitWithConvertibleBoolDefaultValue)
{
    std::unique_ptr<alica::BlackboardBlueprint> blueprint = std::make_unique<alica::BlackboardBlueprint>();
    blueprint->addKey("boolVal", BOOL, "1");
    // throw exception because "1" is not of type bool
    EXPECT_THROW({ alica::Blackboard(blueprint.get()); }, BlackboardException);
}

TEST_F(TestBlackboard, testInitWithConvertibleDoubleDefaultValue)
{
    std::unique_ptr<alica::BlackboardBlueprint> blueprint = std::make_unique<alica::BlackboardBlueprint>();
    blueprint->addKey("doubleVal", DOUBLE, "false");
    // throw exception, because of "false" is not of type double
    EXPECT_THROW({ alica::Blackboard(blueprint.get()); }, BlackboardException);
}

TEST_F(TestBlackboard, testAnyWithDifferentTypes)
{
    std::unique_ptr<alica::BlackboardBlueprint> blueprint = std::make_unique<alica::BlackboardBlueprint>();
    blueprint->addKey("anyType", ANY, std::nullopt);
    blueprint->addKey("knownType", ANY, std::nullopt);
    blueprint->addKey("unknownType", ANY, std::nullopt);
    alica::Blackboard blackboard = alica::Blackboard(blueprint.get());
    LockedBlackboardRW bb_locked = LockedBlackboardRW(blackboard);

    bb_locked.set<double>("knownType", 17.3);
    bb_locked.set<UnknownType>("unknownType", UnknownType(7198));
    bb_locked.set<std::any>("anyType", std::any{true});

    // use get with correct types
    EXPECT_EQ(bb_locked.get<double>("knownType"), 17.3);
    EXPECT_EQ(bb_locked.get<UnknownType>("unknownType").value, 7198);
    // EXPECT_EQ(std::any_cast<bool>(bb_locked.get<std::any>("anyType")), true);
    EXPECT_EQ(bb_locked.get<bool>("anyType"), true);

    // use get with wrong type
    EXPECT_THROW({ bb_locked.get<std::string>("knownType"); }, BlackboardException);
    EXPECT_THROW({ bb_locked.get<std::string>("unknownType"); }, BlackboardException);
    EXPECT_THROW({ bb_locked.get<std::string>("anyType"); }, BlackboardException);

    // cant access any using std::any
    EXPECT_THROW({ bb_locked.get<std::any>("anyType"); }, BlackboardException);
}

TEST_F(TestBlackboard, testMappingWithDifferentPMLTypes)
{
    std::unique_ptr<alica::BlackboardBlueprint> blueprintSrc = std::make_unique<alica::BlackboardBlueprint>();
    blueprintSrc->addKey("anyTypeSrc", ANY, std::nullopt);
    blueprintSrc->addKey("knownTypeSrc", INT64, std::nullopt);
    blueprintSrc->addKey("unknownTypeSrc", ANY, std::nullopt);

    std::unique_ptr<alica::BlackboardBlueprint> blueprintTarget = std::make_unique<alica::BlackboardBlueprint>();
    blueprintTarget->addKey("anyTypeTarget", ANY, std::nullopt);
    blueprintTarget->addKey("knownTypeTarget", INT64, std::nullopt);
    blueprintTarget->addKey("unknownTypeTarget", ANY, std::nullopt);

    alica::Blackboard srcBb = alica::Blackboard(blueprintSrc.get());
    alica::Blackboard targetBb = alica::Blackboard(blueprintTarget.get());

    srcBb.impl().set<int64_t>("knownTypeSrc", 1);
    targetBb.impl().map("knownTypeSrc", "knownTypeTarget", srcBb.impl());
    EXPECT_EQ(targetBb.impl().get<int64_t>("knownTypeTarget"), 1.0);

    srcBb.impl().set<alica::PlanStatus>("unknownTypeSrc", alica::PlanStatus::Success);
    targetBb.impl().set<alica::PlanStatus>("unknownTypeTarget", alica::PlanStatus::Failed);
    targetBb.impl().map("unknownTypeSrc", "unknownTypeTarget", srcBb.impl());
    EXPECT_EQ(targetBb.impl().get<alica::PlanStatus>("unknownTypeTarget"), alica::PlanStatus::Success);

    srcBb.impl().set<std::any>("anyTypeSrc", std::any{19L});
    targetBb.impl().map("anyTypeSrc", "anyTypeTarget", srcBb.impl());
    EXPECT_EQ(targetBb.impl().get<int64_t>("anyTypeTarget"), 19L);
}

TEST_F(TestBlackboard, setWithConvertibleType)
{
    std::unique_ptr<alica::BlackboardBlueprint> blueprint = std::make_unique<alica::BlackboardBlueprint>();
    blueprint->addKey("intVal", INT64, "14");
    alica::Blackboard blackboard = alica::Blackboard(blueprint.get());
    alica::LockedBlackboardRW bb = LockedBlackboardRW(blackboard);

    bb.set<int8_t>("intVal", 1);
    EXPECT_EQ(bb.get<int64_t>("intVal"), 1);
    bb.set<int32_t>("intVal", 1);
    EXPECT_EQ(bb.get<int64_t>("intVal"), 1);

    bb.set<uint8_t>("intVal", 1);
    EXPECT_EQ(bb.get<int64_t>("intVal"), 1);
    bb.set<uint32_t>("intVal", 1);
    EXPECT_EQ(bb.get<int64_t>("intVal"), 1);

    EXPECT_THROW({bb.set<uint64_t>("intVal", 1);}, BlackboardException);
}

TEST_F(TestBlackboard, setWithoutSpecifyingType)
{
    std::unique_ptr<alica::BlackboardBlueprint> blueprint = std::make_unique<alica::BlackboardBlueprint>();
    alica::Blackboard blackboard = alica::Blackboard();
    alica::LockedBlackboardRW bb = LockedBlackboardRW(blackboard);

    bb.set("intVal", 19);
    bb.set("doubleVal", 3.3);
    bb.set("boolVal", true);
    bb.set("longIntVal", 1L);
    bb.set("usignedLongIntVal", 7uL);
    bb.set("stringVal", std::string("abc")); // has to be wrapped into std::string, otherwise is bool
    bb.set("unknownType", UnknownType(123));

    EXPECT_EQ(bb.get<int64_t>("intVal"), 19);
    EXPECT_EQ(bb.get<double>("doubleVal"), 3.3);
    EXPECT_EQ(bb.get<bool>("boolVal"), true);
    EXPECT_EQ(bb.get<int64_t>("longIntVal"), true);
    EXPECT_EQ(bb.get<uint64_t>("usignedLongIntVal"), 7uL);
    EXPECT_EQ(bb.get<std::string>("stringVal"), "abc"); // will be interpreted as a bool (true)
    EXPECT_EQ(bb.get<UnknownType>("unknownType").value, 123);

    bb.set("usignedIntVal", 5u);
    EXPECT_THROW({bb.get<uint64_t>("usignedIntVal");}, BlackboardException);
    EXPECT_EQ(bb.get<int64_t>("usignedIntVal"), 5u);
}

} // namespace
} // namespace alica
