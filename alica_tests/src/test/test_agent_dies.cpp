#include <engine/AlicaClock.h>
#include <engine/AlicaEngine.h>
#include <engine/IAlicaCommunication.h>
#include <engine/model/Task.h>
#include <gtest/gtest.h>
#include <test_alica.h>

#include "TestWorldModel.h"
#include "engine/PlanBase.h"
#include "engine/TeamObserver.h"
#include "engine/model/Plan.h"
#include "engine/model/State.h"
#include "engine/teammanager/TeamManager.h"
#include <communication/AlicaDummyCommunication.h>

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

    tcs[0]->setClock<TestClock>();
    tcs[1]->setClock<TestClock>();

    tcs[0]->startEngine();
    tcs[1]->startEngine();
    // Let agent announce their presence
    tcs[0]->getAlicaClock().sleep(getDiscoveryTimeout());
    // Process presence announcement
    tcs[0]->stepEngine();
    tcs[1]->stepEngine();

    RunningPlan::setAssignmentProtectionTime(AlicaTime::seconds(1000.0));
    tcs[0]->setTeamTimeout(AlicaTime::milliseconds(500));
    tcs[1]->setTeamTimeout(AlicaTime::milliseconds(500));
    tcs[0]->stepEngine();
    tcs[1]->stepEngine();
    getTestClock(tcs[0]).increment(AlicaTime::milliseconds(50));
    getTestClock(tcs[1]).increment(AlicaTime::milliseconds(50));

    alicaTests::TestWorldModel::getOne()->setTransitionCondition1413201227586(true);
    alicaTests::TestWorldModel::getTwo()->setTransitionCondition1413201227586(true);

    tcs[0]->stepEngine();
    tcs[1]->stepEngine();
    getTestClock(tcs[0]).increment(AlicaTime::milliseconds(50));
    getTestClock(tcs[1]).increment(AlicaTime::milliseconds(50));

    ASSERT_EQ(tcs[0]->getRootNode()->getActiveState()->getId(), 1413201213955);
    ASSERT_EQ(tcs[1]->getRootNode()->getActiveState()->getId(), 1413201213955);

    ASSERT_EQ(2, tcs[0]->getTeamSize());
    ASSERT_EQ(2, tcs[1]->getTeamSize());

    ASSERT_EQ(tcs[0]->getRootNode()->getChildren()[0]->getActivePlan()->getId(), 1413200862180);
    ASSERT_EQ(tcs[1]->getRootNode()->getChildren()[0]->getActivePlan()->getId(), 1413200862180);

    ASSERT_EQ(2, tcs[0]->getRootNode()->getChildren()[0]->getAssignment().size());
    ASSERT_EQ(2, tcs[1]->getRootNode()->getChildren()[0]->getAssignment().size());

    const_cast<IAlicaCommunication&>(tcs[0]->getCommunicator()).stopCommunication();
    const_cast<IAlicaCommunication&>(tcs[1]->getCommunicator()).stopCommunication();

    tcs[0]->stepEngine();
    tcs[1]->stepEngine();
    getTestClock(tcs[0]).increment(AlicaTime::milliseconds(2000));
    getTestClock(tcs[1]).increment(AlicaTime::milliseconds(2000));

    tcs[0]->stepEngine();
    tcs[1]->stepEngine();

    getTestClock(tcs[0]).increment(AlicaTime::milliseconds(50));
    getTestClock(tcs[1]).increment(AlicaTime::milliseconds(50));

    tcs[0]->stepEngine();
    tcs[1]->stepEngine();

    ASSERT_EQ(1, tcs[0]->getTeamSize());
    ASSERT_EQ(1, tcs[1]->getTeamSize());

    ASSERT_EQ(0u, tcs[0]->getRootNode()->getChildren().size());
    ASSERT_EQ(0u, tcs[1]->getRootNode()->getChildren().size());

    const_cast<IAlicaCommunication&>(tcs[0]->getCommunicator()).startCommunication();
    const_cast<IAlicaCommunication&>(tcs[1]->getCommunicator()).startCommunication();

    getTestClock(tcs[0]).increment(AlicaTime::milliseconds(50));
    getTestClock(tcs[1]).increment(AlicaTime::milliseconds(50));

    tcs[0]->stepEngine();
    tcs[1]->stepEngine();

    getTestClock(tcs[0]).increment(AlicaTime::milliseconds(50));
    getTestClock(tcs[1]).increment(AlicaTime::milliseconds(50));

    tcs[0]->stepEngine();
    tcs[1]->stepEngine();

    getTestClock(tcs[0]).increment(AlicaTime::milliseconds(50));
    getTestClock(tcs[1]).increment(AlicaTime::milliseconds(50));

    tcs[0]->stepEngine();
    tcs[1]->stepEngine();

    ASSERT_EQ(2, tcs[0]->getTeamSize());
    ASSERT_EQ(2, tcs[1]->getTeamSize());

    ASSERT_EQ(2, tcs[0]->getRootNode()->getChildren()[0]->getAssignment().size());
    ASSERT_EQ(2, tcs[1]->getRootNode()->getChildren()[0]->getAssignment().size());
}
}
}