#include <engine/AlicaClock.h>
#include <engine/DefaultUtilityFunction.h>
#include <gtest/gtest.h>
#include <test_alica.h>

namespace alica
{
namespace
{

class AlicaEngineTestInit : public AlicaTestFixture
{
protected:
    const char* getRoleSetName() const override { return "Roleset"; }
    const char* getMasterPlanName() const override { return "MasterPlan"; }
    bool stepEngine() const override { return false; }
};

/**
 * Initialises an instance of the AlicaEngine and shuts it down again. This test is nice for basic memory leak testing.
 */
TEST_F(AlicaEngineTestInit, initAndShutdown)
{
    ASSERT_NO_SIGNAL
    EXPECT_NE(ac, nullptr);
}

TEST_F(AlicaEngineTestInit, globalBlackboardInit)
{
    LockedBlackboardRO gbb(ac->getGlobalBlackboard());
    ASSERT_EQ(gbb.get<std::string>("agentName"), getHostName());
    ASSERT_EQ(gbb.get<AgentId>("agentID"), ac->getConfig()["Local"]["ID"].as<AgentId>());
}

} // namespace
} // namespace alica
