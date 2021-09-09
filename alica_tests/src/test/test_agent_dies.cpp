#include "test_alica.h"

#include <alica_tests/TestWorldModel.h>

#include <alica/test/Util.h>
#include <communication/AlicaDummyCommunication.h>
#include <engine/AlicaClock.h>
#include <engine/AlicaEngine.h>
#include <engine/IAlicaCommunication.h>
#include <engine/PlanBase.h>
#include <engine/TeamObserver.h>
#include <engine/model/Plan.h>
#include <engine/model/State.h>
#include <engine/model/Task.h>
#include <engine/teammanager/TeamManager.h>

#include <gtest/gtest.h>

namespace alica
{
namespace
{

class AlicaEngineAgentDiesTest : public AlicaTestMultiAgentFixture
{
protected:
    const int agentCount = 2;
    const char* getRoleSetName() const override { return "RolesetTA"; }
    const char* getMasterPlanName() const override { return "MultiAgentTestMaster"; }
    int getAgentCount() const override { return agentCount; }
    const char* getHostName(int agentNumber) const override
    {
        if (agentNumber) {
            return "hairy";
        } else {
            return "nase";
        }
    }
};

class TestClock : public AlicaClock
{
public:
    TestClock()
            : _now(AlicaClock::now())
    {
    }
    AlicaTime now() const override { return _now; }
    void increment(AlicaTime t) { _now += t; }

private:
    AlicaTime _now;
};

inline TestClock& getTestClock(AlicaContext* ac)
{
    return static_cast<TestClock&>(const_cast<AlicaClock&>(ac->getAlicaClock()));
}

TEST_F(AlicaEngineAgentDiesTest, AgentIsRemoved)
{
    ASSERT_NO_SIGNAL

    acs[0]->setClock<TestClock>();
    acs[1]->setClock<TestClock>();

    aes[0]->start();
    aes[1]->start();
    // Let agent announce their presence
    aes[0]->getAlicaClock().sleep(getDiscoveryTimeout());
    // Process presence announcement
    acs[0]->stepEngine();
    acs[1]->stepEngine();

    RunningPlan::setAssignmentProtectionTime(AlicaTime::seconds(1000.0));
    aes[0]->editTeamManager().setTeamTimeout(AlicaTime::milliseconds(500));
    aes[1]->editTeamManager().setTeamTimeout(AlicaTime::milliseconds(500));
    acs[0]->stepEngine();
    acs[1]->stepEngine();
    getTestClock(acs[0]).increment(AlicaTime::milliseconds(50));
    getTestClock(acs[1]).increment(AlicaTime::milliseconds(50));

    alicaTests::TestWorldModel::getOne()->setTransitionCondition1413201227586(true);
    alicaTests::TestWorldModel::getTwo()->setTransitionCondition1413201227586(true);

    acs[0]->stepEngine();
    acs[1]->stepEngine();
    getTestClock(acs[0]).increment(AlicaTime::milliseconds(50));
    getTestClock(acs[1]).increment(AlicaTime::milliseconds(50));

    ASSERT_TRUE(alica::test::Util::isStateActive(aes[0], 1413201213955));
    ASSERT_TRUE(alica::test::Util::isStateActive(aes[1], 1413201213955));

    ASSERT_EQ(2, alica::test::Util::getTeamSize(aes[0]));
    ASSERT_EQ(2, alica::test::Util::getTeamSize(aes[1]));

    ASSERT_TRUE(alica::test::Util::isPlanActive(aes[0], 1413200862180));
    ASSERT_TRUE(alica::test::Util::isPlanActive(aes[1], 1413200862180));

    ASSERT_EQ(2, aes[0]->getPlanBase().getRootNode()->getChildren()[0]->getAssignment().size());
    ASSERT_EQ(2, aes[1]->getPlanBase().getRootNode()->getChildren()[0]->getAssignment().size());

    const_cast<IAlicaCommunication&>(acs[0]->getCommunicator()).stopCommunication();
    const_cast<IAlicaCommunication&>(acs[1]->getCommunicator()).stopCommunication();

    acs[0]->stepEngine();
    acs[1]->stepEngine();
    getTestClock(acs[0]).increment(AlicaTime::milliseconds(2000));
    getTestClock(acs[1]).increment(AlicaTime::milliseconds(2000));

    acs[0]->stepEngine();
    acs[1]->stepEngine();

    getTestClock(acs[0]).increment(AlicaTime::milliseconds(50));
    getTestClock(acs[1]).increment(AlicaTime::milliseconds(50));

    acs[0]->stepEngine();
    acs[1]->stepEngine();

    ASSERT_EQ(1, alica::test::Util::getTeamSize(aes[0]));
    ASSERT_EQ(1, alica::test::Util::getTeamSize(aes[1]));

    ASSERT_EQ(0u, aes[0]->getPlanBase().getRootNode()->getChildren().size());
    ASSERT_EQ(0u, aes[1]->getPlanBase().getRootNode()->getChildren().size());

    const_cast<IAlicaCommunication&>(acs[0]->getCommunicator()).startCommunication();
    const_cast<IAlicaCommunication&>(acs[1]->getCommunicator()).startCommunication();

    getTestClock(acs[0]).increment(AlicaTime::milliseconds(50));
    getTestClock(acs[1]).increment(AlicaTime::milliseconds(50));

    acs[0]->stepEngine();
    acs[1]->stepEngine();

    getTestClock(acs[0]).increment(AlicaTime::milliseconds(50));
    getTestClock(acs[1]).increment(AlicaTime::milliseconds(50));

    acs[0]->stepEngine();
    acs[1]->stepEngine();

    getTestClock(acs[0]).increment(AlicaTime::milliseconds(50));
    getTestClock(acs[1]).increment(AlicaTime::milliseconds(50));

    acs[0]->stepEngine();
    acs[1]->stepEngine();

    ASSERT_EQ(2, alica::test::Util::getTeamSize(aes[0]));
    ASSERT_EQ(2, alica::test::Util::getTeamSize(aes[1]));

    ASSERT_EQ(2, aes[0]->getPlanBase().getRootNode()->getChildren()[0]->getAssignment().size());
    ASSERT_EQ(2, aes[1]->getPlanBase().getRootNode()->getChildren()[0]->getAssignment().size());
}
} // namespace
} // namespace alica