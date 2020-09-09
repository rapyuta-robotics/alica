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

class AlicaEngineAgentDiscoveryTest : public AlicaTestMultiAgentFixture
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

    void verifyAgents(const alica::Agent* original, const alica::Agent* discovered)
    {
        ASSERT_NE(original, nullptr);
        ASSERT_NE(discovered, nullptr);
        ASSERT_EQ(discovered->getName(), original->getName());
        ASSERT_EQ(discovered->getSdk(), original->getSdk());
        ASSERT_EQ(discovered->getPlanHash(), original->getPlanHash());
        ASSERT_EQ(*(discovered->getId()), *(original->getId()));
    }
};

TEST_F(AlicaEngineAgentDiscoveryTest, AgentDiscovered)
{
    ASSERT_NO_SIGNAL
    ASSERT_EQ(1, tcs[0]->getTeamSize());
    ASSERT_EQ(1, tcs[1]->getTeamSize());
    tcs[0]->startEngine();
    tcs[1]->startEngine();
    // Let agent announce their presence
    tcs[0]->getAlicaClock().sleep(getDiscoveryTimeout());
    // Process presence announcement
    tcs[0]->stepEngine();
    tcs[1]->stepEngine();
    ASSERT_EQ(2, tcs[0]->getTeamSize());
    ASSERT_EQ(2, tcs[1]->getTeamSize());

    uint64_t id = 8;
    const alica::Agent* hairydiscovered = tcs[0]->getAgentByID(tcs[0]->getID(id));
    const alica::Agent* hairyoriginal = tcs[1]->getLocalAgent();
    verifyAgents(hairydiscovered, hairyoriginal);

    id = 9;
    const alica::Agent* nasediscovered = tcs[1]->getAgentByID(tcs[1]->getID(id));
    const alica::Agent* naseoriginal = tcs[0]->getLocalAgent();
    verifyAgents(nasediscovered, naseoriginal);

    // Reject agent with mismatching plan hash
    id = 11;
    alica::AgentAnnouncement aa;
    aa.planHash = tcs[0]->getVersion() + 1;
    aa.senderSdk = tcs[0]->getVersion();
    aa.senderID = tcs[0]->getID(id);
    aa.roleId = 1222973297047; // Attacker
    aa.senderName = "myo";
    tcs[0]->handleAgentAnnouncement(aa);
    tcs[0]->stepEngine();
    const alica::Agent* myodiscovered = tcs[0]->getAgentByID(tcs[0]->getID(id));
    ASSERT_EQ(myodiscovered, nullptr);
}
}
}