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

struct BBType
{
    static constexpr const char* BOOL = "bool";
    static constexpr const char* INT64 = "int64";
    static constexpr const char* UINT64 = "uint64";
    static constexpr const char* DOUBLE = "double";
    static constexpr const char* STRING = "std::string";
    static constexpr const char* ANY = "std::any";
};

// used for accessing bb values with an unknown type
class UnknownType
{
public:
    UnknownType() = default;
    UnknownType(int64_t input)
            : value(input){};
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

    template <class SrcType, class TargetType>
    bool checkMapping(const std::string& srcTypeName, const SrcType& srcValue, const std::string& targetTypeName, const TargetType& targetValue)
    {
        BlackboardBlueprint srcBlueprint;
        srcBlueprint.addKey("srcKey", srcTypeName, {});
        BlackboardBlueprint targetBlueprint;
        targetBlueprint.addKey("targetKey", targetTypeName, {});
        Blackboard srcBB(&srcBlueprint);
        Blackboard targetBB(&targetBlueprint);
        srcBB.impl().set("srcKey", srcValue);
        targetBB.impl().map("srcKey", "targetKey", srcBB.impl());
        if constexpr (std::is_same_v<TargetType, std::any>) {
            return std::any_cast<SrcType>(targetBB.impl().get<TargetType>("targetKey")) == std::any_cast<SrcType>(targetValue);
        } else {
            return targetBB.impl().get<TargetType>("targetKey") == targetValue;
        }
    }
};

TEST_F(TestBlackboard, testJsonTwoBehaviorKeyMapping)
{
    // Two parent values mapped to same behavior input value to differentiate call contexts
    ae->start();
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(200));

    std::shared_ptr<alicaTests::TestWorldModel> wm = LockedBlackboardRW(ae->editGlobalBlackboard()).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    EXPECT_EQ(wm->passedParameters["behaviorInputKey"], 5);       // Value set in first behavior call
    EXPECT_EQ(wm->passedParameters["behaviorSecondInputKey"], 7); // Value set in second behavior call
}

TEST_F(TestBlackboard, testJsonPlanKeyMapping)
{
    // Pass values of mapped keys form a plan into another plan
    ae->start();
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(200));
    std::shared_ptr<alicaTests::TestWorldModel> wm = LockedBlackboardRW(ae->editGlobalBlackboard()).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    EXPECT_EQ(wm->passedParameters["planInputFromMaster"], 8);
}

TEST_F(TestBlackboard, testJsonBehaviorKeyMapping)
{
    // Pass values of mapped keys form a plan into a behavior and out of a bahavior into a plan
    ae->start();
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(200));
    std::shared_ptr<alicaTests::TestWorldModel> wm = LockedBlackboardRW(ae->editGlobalBlackboard()).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    EXPECT_EQ(wm->passedParameters["behaviorInputKey"], 5); // Value set in plan init -> read in behavior
    EXPECT_EQ(wm->passedParameters["planInputKey"], 6);     // Value set in behavior -> read in plan termination
}

TEST_F(TestBlackboard, testJsonBlackboardPlan)
{
    // Check if a key defined in json of a plan is accessible
    ae->start();
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(100));
    std::shared_ptr<alicaTests::TestWorldModel> wm = LockedBlackboardRW(ae->editGlobalBlackboard()).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    EXPECT_EQ(wm->passedParameters["planKey"], 1);
}

TEST_F(TestBlackboard, testJsonBlackboardBehavior)
{
    // Check if a key defined in json of a behavior is accessible
    ae->start();
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(100));
    std::shared_ptr<alicaTests::TestWorldModel> wm = LockedBlackboardRW(ae->editGlobalBlackboard()).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
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

TEST_F(TestBlackboard, testMappingFromBool)
{
    EXPECT_TRUE((checkMapping<bool, bool>(BBType::BOOL, true, BBType::BOOL, true)));
    EXPECT_THROW((checkMapping<bool, int64_t>(BBType::BOOL, true, BBType::INT64, 1)), BlackboardException);
    EXPECT_THROW((checkMapping<bool, uint64_t>(BBType::BOOL, true, BBType::UINT64, 1u)), BlackboardException);
    EXPECT_THROW((checkMapping<bool, double>(BBType::BOOL, true, BBType::DOUBLE, 1.0)), BlackboardException);
    EXPECT_THROW((checkMapping<bool, std::string>(BBType::BOOL, true, BBType::STRING, "true")), BlackboardException);
    EXPECT_TRUE((checkMapping<bool, std::any>(BBType::BOOL, true, BBType::ANY, std::any{true})));
}

TEST_F(TestBlackboard, testMappingFromInt64)
{
    EXPECT_THROW((checkMapping<int64_t, bool>(BBType::INT64, 1, BBType::BOOL, true)), BlackboardException);
    EXPECT_TRUE((checkMapping<int64_t, int64_t>(BBType::INT64, 1, BBType::INT64, 1)));
    EXPECT_THROW((checkMapping<int64_t, uint64_t>(BBType::INT64, 1, BBType::UINT64, 1u)), BlackboardException);
    EXPECT_THROW((checkMapping<int64_t, double>(BBType::INT64, 1, BBType::DOUBLE, 1.0)), BlackboardException);
    EXPECT_THROW((checkMapping<int64_t, std::string>(BBType::INT64, 1, BBType::STRING, "1")), BlackboardException);
    EXPECT_TRUE((checkMapping<int64_t, std::any>(BBType::INT64, 1, BBType::ANY, std::any{(int64_t) 1})));
}

TEST_F(TestBlackboard, testMappingFromUnsignedInt64)
{
    EXPECT_THROW((checkMapping<uint64_t, bool>(BBType::UINT64, 1, BBType::BOOL, true)), BlackboardException);
    EXPECT_THROW((checkMapping<uint64_t, int64_t>(BBType::UINT64, 1, BBType::INT64, 1)), BlackboardException);
    EXPECT_TRUE((checkMapping<uint64_t, uint64_t>(BBType::UINT64, 1, BBType::UINT64, 1u)));
    EXPECT_THROW((checkMapping<uint64_t, double>(BBType::UINT64, 1, BBType::DOUBLE, 1.0)), BlackboardException);
    EXPECT_THROW((checkMapping<uint64_t, std::string>(BBType::UINT64, 1, BBType::STRING, "1")), BlackboardException);
    EXPECT_TRUE((checkMapping<uint64_t, std::any>(BBType::UINT64, 1, BBType::ANY, std::any{(uint64_t) 1u})));
}

TEST_F(TestBlackboard, testMappingFromDouble)
{
    EXPECT_THROW((checkMapping<double, bool>(BBType::DOUBLE, 1.0, BBType::BOOL, true)), BlackboardException);
    EXPECT_THROW((checkMapping<double, int64_t>(BBType::DOUBLE, 1.0, BBType::INT64, 1)), BlackboardException);
    EXPECT_THROW((checkMapping<double, uint64_t>(BBType::DOUBLE, 1.0, BBType::UINT64, 1u)), BlackboardException);
    EXPECT_TRUE((checkMapping<double, double>(BBType::DOUBLE, 1.0, BBType::DOUBLE, 1.0)));
    EXPECT_THROW((checkMapping<double, std::string>(BBType::DOUBLE, 1.0, BBType::STRING, "1.0")), BlackboardException);
    EXPECT_TRUE((checkMapping<double, std::any>(BBType::DOUBLE, 1.0, BBType::ANY, std::any{1.0})));
}

TEST_F(TestBlackboard, testMappingFromString)
{
    EXPECT_THROW((checkMapping<std::string, bool>(BBType::STRING, "1", BBType::BOOL, true)), BlackboardException);
    EXPECT_THROW((checkMapping<std::string, int64_t>(BBType::STRING, "1", BBType::INT64, 1)), BlackboardException);
    EXPECT_THROW((checkMapping<std::string, uint64_t>(BBType::STRING, "1", BBType::UINT64, 1u)), BlackboardException);
    EXPECT_THROW((checkMapping<std::string, double>(BBType::STRING, "1", BBType::DOUBLE, 1.0)), BlackboardException);
    EXPECT_TRUE((checkMapping<std::string, std::string>(BBType::STRING, "1", BBType::STRING, "1")));
    EXPECT_TRUE((checkMapping<std::string, std::any>(BBType::STRING, "1", BBType::ANY, std::any{std::string{"1"}})));
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

    // throw exception because uint64 does not match int64, because signed -> unsigned conversions are not supported
    EXPECT_THROW({ bb_locked.get<uint64_t>("value"); }, BlackboardException);
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
    blueprint->addKey("intVal", BBType::INT64, "14");
    blueprint->addKey("uintVal", BBType::UINT64, "15");
    blueprint->addKey("doubleVal", BBType::DOUBLE, "2.7");
    blueprint->addKey("boolVal", BBType::BOOL, "true");
    blueprint->addKey("stringVal", BBType::STRING, "test string");
    alica::Blackboard blackboard = alica::Blackboard(blueprint.get());
    alica::LockedBlackboardRW bb = LockedBlackboardRW(blackboard);

    EXPECT_EQ(bb.get<int64_t>("intVal"), 14);
    EXPECT_EQ(bb.get<uint64_t>("uintVal"), 15u);
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

    // throw exception because value is not of type int64, but uint64
    EXPECT_THROW({ bb.set<int64_t>("uintVal", 0); }, BlackboardException);

    // throw exception because value is not of type unt64, but int64
    EXPECT_THROW({ bb.set<uint64_t>("intVal", 0); }, BlackboardException);
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
    blueprint->addKey("anyWithDefault", BBType::ANY, "defaultValue");
    // throw exception because std::any type can not have a default value
    EXPECT_THROW({ alica::Blackboard(blueprint.get()); }, BlackboardException);
}

TEST_F(TestBlackboard, testInitWithWrongDefaultValueType)
{
    std::unique_ptr<alica::BlackboardBlueprint> blueprint = std::make_unique<alica::BlackboardBlueprint>();
    blueprint->addKey("anyWithDefault", BBType::INT64, "defaultValue");
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
    blueprint->addKey("intVal", BBType::INT64, "14");
    blueprint->addKey("uintVal", BBType::UINT64, "15");
    blueprint->addKey("doubleVal", BBType::DOUBLE, "2.7");
    blueprint->addKey("boolVal", BBType::BOOL, "true");
    blueprint->addKey("stringVal", BBType::STRING, "test string");
    alica::Blackboard blackboard = alica::Blackboard(blueprint.get());
    alica::LockedBlackboardRO bb = LockedBlackboardRO(blackboard);

    EXPECT_EQ(bb.get<int64_t>("intVal"), 14);
    EXPECT_EQ(bb.get<uint64_t>("uintVal"), 15u);
    EXPECT_EQ(bb.get<double>("doubleVal"), 2.7);
    EXPECT_EQ(bb.get<bool>("boolVal"), true);
    EXPECT_EQ(bb.get<std::string>("stringVal"), "test string");
}

TEST_F(TestBlackboard, testInitWithoutDefaultValue)
{
    std::unique_ptr<alica::BlackboardBlueprint> blueprint = std::make_unique<alica::BlackboardBlueprint>();
    blueprint->addKey("intVal", BBType::INT64, std::nullopt);
    blueprint->addKey("uintVal", BBType::UINT64, std::nullopt);
    blueprint->addKey("doubleVal", BBType::DOUBLE, std::nullopt);
    blueprint->addKey("boolVal", BBType::BOOL, std::nullopt);
    blueprint->addKey("stringVal", BBType::STRING, std::nullopt);
    alica::Blackboard blackboard = alica::Blackboard(blueprint.get());
    alica::LockedBlackboardRO bb = LockedBlackboardRO(blackboard);

    EXPECT_EQ(bb.get<int64_t>("intVal"), int64_t());
    EXPECT_EQ(bb.get<uint64_t>("uintVal"), uint64_t());
    EXPECT_EQ(bb.get<double>("doubleVal"), double());
    EXPECT_EQ(bb.get<bool>("boolVal"), false);
    EXPECT_EQ(bb.get<std::string>("stringVal"), "");
}

TEST_F(TestBlackboard, testInitWithConvertibleIntDefaultValue)
{
    std::unique_ptr<alica::BlackboardBlueprint> blueprint = std::make_unique<alica::BlackboardBlueprint>();
    blueprint->addKey("intVal", BBType::INT64, "14.7");
    // throw exception because "14.7" is not of type int64
    EXPECT_THROW({ alica::Blackboard(blueprint.get()); }, BlackboardException);
}

TEST_F(TestBlackboard, testInitWithConvertibleBoolDefaultValue)
{
    std::unique_ptr<alica::BlackboardBlueprint> blueprint = std::make_unique<alica::BlackboardBlueprint>();
    blueprint->addKey("boolVal", BBType::BOOL, "1");
    // throw exception because "1" is not of type bool
    EXPECT_THROW({ alica::Blackboard(blueprint.get()); }, BlackboardException);
}

TEST_F(TestBlackboard, testInitWithConvertibleDoubleDefaultValue)
{
    std::unique_ptr<alica::BlackboardBlueprint> blueprint = std::make_unique<alica::BlackboardBlueprint>();
    blueprint->addKey("doubleVal", BBType::DOUBLE, "false");
    // throw exception, because of "false" is not of type double
    EXPECT_THROW({ alica::Blackboard(blueprint.get()); }, BlackboardException);
}

TEST_F(TestBlackboard, testAnyWithDifferentTypes)
{
    std::unique_ptr<alica::BlackboardBlueprint> blueprint = std::make_unique<alica::BlackboardBlueprint>();
    blueprint->addKey("anyType", BBType::ANY, std::nullopt);
    blueprint->addKey("knownType", BBType::ANY, std::nullopt);
    blueprint->addKey("unknownType", BBType::ANY, std::nullopt);
    alica::Blackboard blackboard = alica::Blackboard(blueprint.get());
    LockedBlackboardRW bb_locked = LockedBlackboardRW(blackboard);

    bb_locked.set<double>("knownType", 17.3);
    bb_locked.set<UnknownType>("unknownType", UnknownType(7198));
    bb_locked.set<std::any>("anyType", std::any{true});

    // use get with correct types
    EXPECT_EQ(bb_locked.get<double>("knownType"), 17.3);
    EXPECT_EQ(bb_locked.get<UnknownType>("unknownType").value, 7198);
    EXPECT_EQ(bb_locked.get<bool>("anyType"), true);

    // use get with wrong type
    EXPECT_THROW({ bb_locked.get<std::string>("knownType"); }, BlackboardException);
    EXPECT_THROW({ bb_locked.get<std::string>("unknownType"); }, BlackboardException);
    EXPECT_THROW({ bb_locked.get<std::string>("anyType"); }, BlackboardException);

    EXPECT_EQ(std::any_cast<bool>(bb_locked.get<std::any>("anyType")), true);
}

TEST_F(TestBlackboard, testMappingWithDifferentPMLTypes)
{
    std::unique_ptr<alica::BlackboardBlueprint> blueprintSrc = std::make_unique<alica::BlackboardBlueprint>();
    blueprintSrc->addKey("anyTypeSrc", BBType::ANY, std::nullopt);
    blueprintSrc->addKey("knownTypeSrc", BBType::INT64, std::nullopt);
    blueprintSrc->addKey("unknownTypeSrc", BBType::ANY, std::nullopt);

    std::unique_ptr<alica::BlackboardBlueprint> blueprintTarget = std::make_unique<alica::BlackboardBlueprint>();
    blueprintTarget->addKey("anyTypeTarget", BBType::ANY, std::nullopt);
    blueprintTarget->addKey("knownTypeTarget", BBType::INT64, std::nullopt);
    blueprintTarget->addKey("unknownTypeTarget", BBType::ANY, std::nullopt);

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
    blueprint->addKey("intVal", BBType::INT64, "14");
    blueprint->addKey("uintVal", BBType::UINT64, "15");
    blueprint->addKey("decimal", BBType::DOUBLE, "1.0");
    blueprint->addKey("string", BBType::STRING, "str");
    alica::Blackboard blackboard = alica::Blackboard(blueprint.get());
    alica::LockedBlackboardRW bb = LockedBlackboardRW(blackboard);

    bb.set<int8_t>("intVal", 1);
    EXPECT_EQ(bb.get<int64_t>("intVal"), 1);
    bb.set<int32_t>("intVal", 1);
    EXPECT_EQ(bb.get<int64_t>("intVal"), 1);

    bb.set<uint8_t>("uintVal", 1u);
    EXPECT_EQ(bb.get<uint64_t>("uintVal"), 1u);
    bb.set<uint32_t>("uintVal", 1u);
    EXPECT_EQ(bb.get<uint64_t>("uintVal"), 1u);

    bb.set<float>("decimal", 2.0);
    EXPECT_EQ(bb.get<double>("decimal"), 2.0);
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
    EXPECT_EQ(bb.get<uint64_t>("usignedIntVal"), 5u);
}

} // namespace
} // namespace alica
