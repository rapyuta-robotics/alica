#include "alica_tests/TestWorldModel.h"
#include "test_alica.h"
#include <alica/test/Util.h>
#include <chrono>
#include <engine/blackboard/Blackboard.h>
#include <gtest/gtest.h>
#include <thread>

namespace alica
{
namespace
{

class TestInheritBlackboard : public test::SingleAgentTestFixture
{
protected:
    const char* getMasterPlanName() const override { return "TestInheritBlackboardMaster"; }
};

TEST_F(TestInheritBlackboard, testInheritBlackboard)
{
    // Use inherited blackboards and check if keys are accessible accross inheriting plans & behaviours
    STEP_UNTIL(_tc, _tc->getActiveBehaviour("TestInheritBlackboardBehaviour"));
    ASSERT_NE(_tc->getActiveBehaviour("TestInheritBlackboardBehaviour"), nullptr) << _tc->getLastFailure();
    STEP_UNTIL(_tc, LockedBlackboardRO{*_tc->getGlobalBlackboardShared()}.hasValue("behaviourKeyInMaster"));
    LockedBlackboardRO gb(*_tc->getGlobalBlackboardShared());
    ASSERT_TRUE(gb.hasValue("behaviourKeyInMaster"));
    ASSERT_EQ(gb.get<int64_t>("masterKeyInBehaviour"), 123);
    ASSERT_EQ(gb.get<int64_t>("behaviourKeyInMaster"), 323);
}

class TestGlobalBlackboard : public test::SingleAgentTestFixture
{
protected:
    const char* getMasterPlanName() const override { return "TestGlobalBlackboardMaster"; }
};

TEST_F(TestGlobalBlackboard, testGlobalBlackboard)
{
    // Master plan's blackboard is marked as global (inherited) & sub-plans & behaviours inherit from it. Therefore all keys
    // set in every plan & behaviour should be available in the global blackboard
    STEP_UNTIL(_tc, _tc->getActiveBehaviour("TestInheritBlackboardBehaviour"));
    ASSERT_NE(_tc->getActiveBehaviour("TestInheritBlackboardBehaviour"), nullptr) << _tc->getLastFailure();
    STEP_UNTIL(_tc, LockedBlackboardRO{*_tc->getGlobalBlackboardShared()}.hasValue("behaviourKeyInMaster"));
    LockedBlackboardRO gb(*_tc->getGlobalBlackboardShared());
    ASSERT_TRUE(gb.hasValue("behaviourKeyInMaster"));
    ASSERT_EQ(gb.get<int64_t>("masterKeyInBehaviour"), 123);
    ASSERT_EQ(gb.get<int64_t>("behaviourKeyInMaster"), 323);
    ASSERT_TRUE(gb.hasValue("masterKey"));
    ASSERT_EQ(gb.get<int64_t>("masterKey"), 123);
    ASSERT_TRUE(gb.hasValue("behaviourKey"));
    ASSERT_EQ(gb.get<int64_t>("behaviourKey"), 323);
}

} // namespace
} // namespace alica
