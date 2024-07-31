#include "alica_tests/TestWorldModel.h"
#include <alica/test/Util.h>
#include <engine/PlanStatus.h>
#include <engine/Types.h>
#include <test_alica.h>
#include <variant>

#include <gtest/gtest.h>

namespace alica::test
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

struct BBAccessType
{
    static constexpr const char* INPUT = "input";
    static constexpr const char* OUTPUT = "output";
    static constexpr const char* PROTECTED = "protected";
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
        srcBlueprint.addKey("srcKey", srcTypeName, BBAccessType::PROTECTED);
        BlackboardBlueprint targetBlueprint;
        targetBlueprint.addKey("targetKey", targetTypeName, BBAccessType::INPUT);
        Blackboard srcBB(&srcBlueprint);
        Blackboard targetBB(&targetBlueprint);
        srcBB._impl.set("srcKey", srcValue);
        targetBB._impl.map("srcKey", "targetKey", srcBB._impl);
        if constexpr (std::is_same_v<TargetType, std::any>) {
            return std::any_cast<SrcType>(targetBB._impl.get<TargetType>("targetKey")) == std::any_cast<SrcType>(targetValue);
        } else {
            return targetBB._impl.get<TargetType>("targetKey") == targetValue;
        }
    }

    alica::internal::BlackboardImpl& getBlackboardImpl(alica::Blackboard& bb) { return bb._impl; }
};

// provides access to blackboard impl in tests
class SingleAgentBlackboardTestFixture : public SingleAgentTestFixture
{
public:
    alica::internal::BlackboardImpl& getBlackboardImpl(alica::Blackboard& bb) { return bb._impl; }
};

TEST_F(SingleAgentTestFixture, testGlobalBlackboard)
{
    // Checks if the global blackboard can be accessed from the context
    LockedBlackboardRW(_tc->editGlobalBlackboard()).set("global_test_key", 123);
    ASSERT_EQ(LockedBlackboardRO(_tc->getGlobalBlackboard()).get<int64_t>("global_test_key"), 123);
}

TEST_F(SingleAgentTestFixture, testValueMappingBehaviours)
{
    // Checks if value mapping for behaviours succeeds

    // Transition to the plan corresponding to this test case
    ASSERT_TRUE(_tc->setTransitionCond("TestMasterPlan", "ChooseTestState", "BlackboardTestState")) << _tc->getLastFailure();
    STEP_UNTIL(_tc, _tc->getActivePlan("BlackboardTestPlan"));
    auto plan = _tc->getActivePlan("BlackboardTestPlan");
    ASSERT_NE(plan, nullptr) << _tc->getLastFailure();

    ASSERT_TRUE(_tc->setTransitionCond("BlackboardTestPlan", "ChooseBlackboardTestState", "ValueMappingBehaviourTestState")) << _tc->getLastFailure();

    // Step until the plan succeeds
    UnlockedBlackboard gb(_tc->editGlobalBlackboard());
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->isSuccess(plan)) << gb.get<std::optional<std::string>>("testError").value();
}

TEST_F(SingleAgentTestFixture, testValueMappingConditions)
{
    // Checks if value mapping for conditions succeeds

    // Transition to the plan corresponding to this test case
    ASSERT_TRUE(_tc->setTransitionCond("TestMasterPlan", "ChooseTestState", "BlackboardTestState")) << _tc->getLastFailure();
    STEP_UNTIL(_tc, _tc->getActivePlan("BlackboardTestPlan"));
    auto plan = _tc->getActivePlan("BlackboardTestPlan");
    ASSERT_NE(plan, nullptr) << _tc->getLastFailure();

    ASSERT_TRUE(_tc->setTransitionCond("BlackboardTestPlan", "ChooseBlackboardTestState", "ValueMappingConditionTestState")) << _tc->getLastFailure();

    // Step until the plan succeeds
    UnlockedBlackboard gb(_tc->editGlobalBlackboard());
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->isSuccess(plan)) << gb.get<std::optional<std::string>>("testError").value();
}

TEST_F(SingleAgentTestFixture, testValueMappingPlans)
{
    // Checks if value mapping for conditions succeeds

    // Transition to the plan corresponding to this test case
    ASSERT_TRUE(_tc->setTransitionCond("TestMasterPlan", "ChooseTestState", "BlackboardTestState")) << _tc->getLastFailure();
    STEP_UNTIL(_tc, _tc->getActivePlan("BlackboardTestPlan"));
    auto plan = _tc->getActivePlan("BlackboardTestPlan");
    ASSERT_NE(plan, nullptr) << _tc->getLastFailure();

    ASSERT_TRUE(_tc->setTransitionCond("BlackboardTestPlan", "ChooseBlackboardTestState", "ValueMappingPlanTestState")) << _tc->getLastFailure();

    // Step until the plan succeeds
    UnlockedBlackboard gb(_tc->editGlobalBlackboard());
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->isSuccess(plan)) << gb.get<std::optional<std::string>>("testError").value();
}

TEST_F(SingleAgentBlackboardTestFixture, testValueMappingKeyNotFound)
{
    BlackboardBlueprint targetBlueprint;
    targetBlueprint.addKey("targetKey", BBType::BOOL, BBAccessType::PROTECTED);

    Blackboard targetBB(&targetBlueprint);
    EXPECT_THROW({ getBlackboardImpl(targetBB).mapValue("wrongTargetKey", "true"); }, BlackboardException);
}

TEST_F(SingleAgentBlackboardTestFixture, testValueMappingUnknownType)
{
    BlackboardBlueprint targetBlueprint;
    targetBlueprint.addKey("targetKey", BBType::ANY, BBAccessType::PROTECTED);

    Blackboard targetBB(&targetBlueprint);
    EXPECT_THROW({ getBlackboardImpl(targetBB).mapValue("targetKey", "test"); }, BlackboardException);
}

TEST_F(SingleAgentBlackboardTestFixture, testValueMappingCantParseValue)
{
    BlackboardBlueprint targetBlueprint;
    targetBlueprint.addKey("targetKey", BBType::BOOL, BBAccessType::PROTECTED);

    Blackboard targetBB(&targetBlueprint);
    EXPECT_THROW({ getBlackboardImpl(targetBB).mapValue("targetKey", "test"); }, YAML::BadConversion);
}

TEST_F(TestBlackboard, testJsonTwoBehaviorKeyMapping)
{
    // Two parent values mapped to same behavior input value to differentiate call contexts
    ae->start();
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(200));

    std::shared_ptr<alicaTests::TestWorldModel> wm =
            LockedBlackboardRW(ac->editGlobalBlackboard()).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    EXPECT_EQ(wm->passedParameters["behaviorInputKey"], 5);       // Value set in first behavior call
    EXPECT_EQ(wm->passedParameters["behaviorSecondInputKey"], 7); // Value set in second behavior call
}

TEST_F(TestBlackboard, testJsonPlanKeyMapping)
{
    // Pass values of mapped keys form a plan into another plan
    ae->start();
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(200));
    std::shared_ptr<alicaTests::TestWorldModel> wm =
            LockedBlackboardRW(ac->editGlobalBlackboard()).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    EXPECT_EQ(wm->passedParameters["planInputFromMaster"], 8);
}

TEST_F(TestBlackboard, testJsonBehaviorKeyMapping)
{
    // Pass values of mapped keys form a plan into a behavior and out of a bahavior into a plan
    ae->start();
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(200));
    std::shared_ptr<alicaTests::TestWorldModel> wm =
            LockedBlackboardRW(ac->editGlobalBlackboard()).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    EXPECT_EQ(wm->passedParameters["behaviorInputKey"], 5); // Value set in plan init -> read in behavior
    EXPECT_EQ(wm->passedParameters["planInputKey"], 6);     // Value set in behavior -> read in plan termination
}

TEST_F(TestBlackboard, testJsonBlackboardPlan)
{
    // Check if a key defined in json of a plan is accessible
    ae->start();
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(100));
    std::shared_ptr<alicaTests::TestWorldModel> wm =
            LockedBlackboardRW(ac->editGlobalBlackboard()).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    EXPECT_EQ(wm->passedParameters["planKey"], 1);
}

TEST_F(TestBlackboard, testJsonBlackboardBehavior)
{
    // Check if a key defined in json of a behavior is accessible
    ae->start();
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(100));
    std::shared_ptr<alicaTests::TestWorldModel> wm =
            LockedBlackboardRW(ac->editGlobalBlackboard()).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    EXPECT_EQ(wm->passedParameters["behaviorKey"], 2);
}

TEST_F(TestBlackboard, testNotInheritBlackboardFlag)
{
    ae->start();
    ae->getAlicaClock().sleep(alica::AlicaTime::milliseconds(100));
    // Behaviour has inheritBlackboard set to false
    EXPECT_FALSE(alica::test::Util::getBasicBehaviour(ae, 831400441334251602)->getInheritBlackboard());
    // Plan has inheritBlackboard set to false
    EXPECT_FALSE(alica::test::Util::getBasicPlan(ae, 1692837668719979457)->getInheritBlackboard());
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
    blueprint->addKey("intVal", BBType::INT64, BBAccessType::PROTECTED);
    blueprint->addKey("uintVal", BBType::UINT64, BBAccessType::PROTECTED);
    blueprint->addKey("doubleVal", BBType::DOUBLE, BBAccessType::PROTECTED);
    blueprint->addKey("boolVal", BBType::BOOL, BBAccessType::PROTECTED);
    blueprint->addKey("stringVal", BBType::STRING, BBAccessType::PROTECTED);
    alica::Blackboard blackboard = alica::Blackboard(blueprint.get());
    alica::LockedBlackboardRW bb = LockedBlackboardRW(blackboard);

    bb.set<int64_t>("intVal", 14);
    bb.set<uint64_t>("uintVal", 15u);
    bb.set<double>("doubleVal", 2.7);
    bb.set<bool>("boolVal", true);
    bb.set<std::string>("stringVal", "test string");

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

TEST_F(TestBlackboard, testAnyWithDifferentTypes)
{
    std::unique_ptr<alica::BlackboardBlueprint> blueprint = std::make_unique<alica::BlackboardBlueprint>();
    blueprint->addKey("anyType", BBType::ANY, BBAccessType::PROTECTED);
    blueprint->addKey("knownType", BBType::ANY, BBAccessType::PROTECTED);
    blueprint->addKey("unknownType", BBType::ANY, BBAccessType::PROTECTED);
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
    blueprintSrc->addKey("anyTypeSrc", BBType::ANY, BBAccessType::PROTECTED);
    blueprintSrc->addKey("knownTypeSrc", BBType::INT64, BBAccessType::PROTECTED);
    blueprintSrc->addKey("unknownTypeSrc", BBType::ANY, BBAccessType::PROTECTED);

    std::unique_ptr<alica::BlackboardBlueprint> blueprintTarget = std::make_unique<alica::BlackboardBlueprint>();
    blueprintTarget->addKey("anyTypeTarget", BBType::ANY, BBAccessType::PROTECTED);
    blueprintTarget->addKey("knownTypeTarget", BBType::INT64, BBAccessType::PROTECTED);
    blueprintTarget->addKey("unknownTypeTarget", BBType::ANY, BBAccessType::PROTECTED);

    alica::Blackboard srcBlackboard = alica::Blackboard(blueprintSrc.get());
    auto& srcBb = getBlackboardImpl(srcBlackboard);
    alica::Blackboard targetBlackboard = alica::Blackboard(blueprintTarget.get());
    auto& targetBb = getBlackboardImpl(targetBlackboard);

    srcBb.set<int64_t>("knownTypeSrc", 1);
    targetBb.map("knownTypeSrc", "knownTypeTarget", srcBb);
    EXPECT_EQ(targetBb.get<int64_t>("knownTypeTarget"), 1.0);

    srcBb.set<alica::PlanStatus>("unknownTypeSrc", alica::PlanStatus::Success);
    targetBb.set<alica::PlanStatus>("unknownTypeTarget", alica::PlanStatus::Failed);
    targetBb.map("unknownTypeSrc", "unknownTypeTarget", srcBb);
    EXPECT_EQ(targetBb.get<alica::PlanStatus>("unknownTypeTarget"), alica::PlanStatus::Success);

    srcBb.set<std::any>("anyTypeSrc", std::any{19L});
    targetBb.map("anyTypeSrc", "anyTypeTarget", srcBb);
    EXPECT_EQ(targetBb.get<int64_t>("anyTypeTarget"), 19L);
}

TEST_F(TestBlackboard, setWithConvertibleType)
{
    std::unique_ptr<alica::BlackboardBlueprint> blueprint = std::make_unique<alica::BlackboardBlueprint>();
    blueprint->addKey("intVal", BBType::INT64, BBAccessType::PROTECTED);
    blueprint->addKey("uintVal", BBType::UINT64, BBAccessType::PROTECTED);
    blueprint->addKey("decimal", BBType::DOUBLE, BBAccessType::PROTECTED);
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

    // set using r values
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

    // set using lvalues
    int intVal = 20;
    bb.set("intVal", intVal);
    double doubleVal = 20.5;
    bb.set("doubleVal", doubleVal);
    bool boolVal = true;
    bb.set("boolVal", boolVal);
    int64_t longVal = 8ll;
    bb.set("longIntVal", longVal);
    uint64_t ulongVal = 9ull;
    bb.set("usignedLongIntVal", ulongVal);
    std::string stringVal = "str";
    bb.set("stringVal", stringVal);
    UnknownType unknown(1234);
    bb.set("unknownType", unknown);

    EXPECT_EQ(bb.get<int64_t>("intVal"), intVal);
    EXPECT_EQ(bb.get<double>("doubleVal"), doubleVal);
    EXPECT_EQ(bb.get<bool>("boolVal"), boolVal);
    EXPECT_EQ(bb.get<int64_t>("longIntVal"), longVal);
    EXPECT_EQ(bb.get<uint64_t>("usignedLongIntVal"), ulongVal);
    EXPECT_EQ(bb.get<std::string>("stringVal"), stringVal); // will be interpreted as a bool (true)
    EXPECT_EQ(bb.get<UnknownType>("unknownType").value, 1234);
}

TEST_F(TestBlackboard, TestUnlockedBB)
{
    // Test all basic operations on an unlocked blackboard
    Blackboard blackboard;
    UnlockedBlackboard bb{blackboard};
    EXPECT_TRUE(bb.empty());
    bb.set("intVal", 1234);
    EXPECT_EQ(bb.get<int64_t>("intVal"), 1234);
    EXPECT_TRUE(bb.hasValue("intVal"));
    EXPECT_EQ(bb.size(), 1);
    EXPECT_FALSE(bb.empty());
}

TEST_F(SingleAgentTestFixture, testPopulateBlackboardPlan)
{
    // Checks if PopulateBlackboard implementation in the standard library works as expected

    // Transition to the plan corresponding to this test case
    ASSERT_TRUE(_tc->setTransitionCond("TestMasterPlan", "ChooseTestState", "BlackboardTestState")) << _tc->getLastFailure();
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->getActivePlan("BlackboardTestPlan")) << _tc->getLastFailure();
    ASSERT_TRUE(_tc->setTransitionCond("BlackboardTestPlan", "ChooseBlackboardTestState", "PopulateBlackboardTestState")) << _tc->getLastFailure();
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->getActivePlan("PopulateBlackboardTestPlan")) << _tc->getLastFailure();

    // Ensure all blackboard values are populated correctly by checking if we land up in OtherChecksState
    // Note: The transition conditions do the checking
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->isStateActive("PopulateBlackboardTestPlan", "OtherChecksState")) << _tc->getLastFailure();

    // Ensure that `not_in_data_key` is not populated because it is not present in the input data
    auto plan = _tc->getActivePlan("PopulateBlackboardTestPlan");
    ASSERT_TRUE(alica::LockedBlackboardRO(*plan->getBlackboard()).get<std::string>("not_in_data_key").empty());

    // Ensure that `not_in_bb` is not on the blackboard though it is in the input data because no new keys should be created during populating the blackboard
    ASSERT_FALSE(alica::LockedBlackboardRO(*plan->getBlackboard()).hasValue("not_in_bb"));

    // Transition to success & ensure that the PopulateBlackboard implementation can successfully terminate
    ASSERT_TRUE(_tc->setTransitionCond("PopulateBlackboardTestPlan", "OtherChecksState", "PopulatedState")) << _tc->getLastFailure();
    ASSERT_TRUE(_tc->getActivePlan("BlackboardTestPlan"));
    STEP_UNTIL_ASSERT_TRUE(_tc, _tc->isSuccess(_tc->getActivePlan("BlackboardTestPlan")));
}

} // namespace alica::test
