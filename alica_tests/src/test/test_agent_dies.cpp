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

TEST_F(AlicaEngineAgentDiesTest, AgentIsRemoved)
{
    ASSERT_NO_SIGNAL

    TestClock* c1 = new TestClock();
    TestClock* c2 = new TestClock();
    acs[0]->setClock<std::unique_ptr<AlicaClock>(c1);
    acs[1]->setClock<std::unique_ptr<AlicaClock>(c2);

    aes[0]->start();
    aes[1]->start();
    // Let agent announce their presence
    aes[0]->getAlicaClock().sleep(getDiscoveryTimeout());
    // Process presence announcement
    step(aes[0]);
    step(aes[1]);

    RunningPlan::setAssignmentProtectionTime(AlicaTime::seconds(1000.0));
    aes[0]->editTeamManager().setTeamTimeout(AlicaTime::milliseconds(500));
    aes[1]->editTeamManager().setTeamTimeout(AlicaTime::milliseconds(500));
    step(aes[0]);
    step(aes[1]);
    c1->increment(AlicaTime::milliseconds(50));
    c2->increment(AlicaTime::milliseconds(50));

    alicaTests::TestWorldModel::getOne()->setTransitionCondition1413201227586(true);
    alicaTests::TestWorldModel::getTwo()->setTransitionCondition1413201227586(true);

    step(aes[0]);
    step(aes[1]);
    c1->increment(AlicaTime::milliseconds(50));
    c2->increment(AlicaTime::milliseconds(50));

    ASSERT_EQ(aes[0]->getPlanBase().getRootNode()->getActiveState()->getId(), 1413201213955);
    ASSERT_EQ(aes[1]->getPlanBase().getRootNode()->getActiveState()->getId(), 1413201213955);

    ASSERT_EQ(2, aes[0]->getTeamManager().getActiveAgentIds().size());
    ASSERT_EQ(2, aes[1]->getTeamManager().getActiveAgentIds().size());

    ASSERT_EQ(aes[0]->getPlanBase().getRootNode()->getChildren()[0]->getActivePlan()->getId(), 1413200862180);
    ASSERT_EQ(aes[1]->getPlanBase().getRootNode()->getChildren()[0]->getActivePlan()->getId(), 1413200862180);

    ASSERT_EQ(2, aes[0]->getPlanBase().getRootNode()->getChildren()[0]->getAssignment().size());
    ASSERT_EQ(2, aes[1]->getPlanBase().getRootNode()->getChildren()[0]->getAssignment().size());

    const_cast<IAlicaCommunication&>(aes[0]->getCommunicator()).stopCommunication();
    const_cast<IAlicaCommunication&>(aes[1]->getCommunicator()).stopCommunication();

    step(aes[0]);
    step(aes[1]);
    c1->increment(AlicaTime::milliseconds(2000));
    c2->increment(AlicaTime::milliseconds(2000));

    step(aes[0]);
    step(aes[1]);

    c1->increment(AlicaTime::milliseconds(50));
    c2->increment(AlicaTime::milliseconds(50));

    step(aes[0]);
    step(aes[1]);

    ASSERT_EQ(1, aes[0]->getTeamManager().getActiveAgentIds().size());
    ASSERT_EQ(1, aes[1]->getTeamManager().getActiveAgentIds().size());

    ASSERT_EQ(0, aes[0]->getPlanBase().getRootNode()->getChildren().size());
    ASSERT_EQ(0, aes[1]->getPlanBase().getRootNode()->getChildren().size());

    const_cast<IAlicaCommunication&>(aes[0]->getCommunicator()).startCommunication();
    const_cast<IAlicaCommunication&>(aes[1]->getCommunicator()).startCommunication();

    c1->increment(AlicaTime::milliseconds(50));
    c2->increment(AlicaTime::milliseconds(50));

    step(aes[0]);
    step(aes[1]);

    c1->increment(AlicaTime::milliseconds(50));
    c2->increment(AlicaTime::milliseconds(50));

    step(aes[0]);
    step(aes[1]);

    c1->increment(AlicaTime::milliseconds(50));
    c2->increment(AlicaTime::milliseconds(50));

    step(aes[0]);
    step(aes[1]);

    ASSERT_EQ(2, aes[0]->getTeamManager().getActiveAgentIds().size());
    ASSERT_EQ(2, aes[1]->getTeamManager().getActiveAgentIds().size());

    ASSERT_EQ(2, aes[0]->getPlanBase().getRootNode()->getChildren()[0]->getAssignment().size());
    ASSERT_EQ(2, aes[1]->getPlanBase().getRootNode()->getChildren()[0]->getAssignment().size());
}
}
}