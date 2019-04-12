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
    ASSERT_EQ(1, aes[0]->getTeamManager().getActiveAgentIds().size());
    ASSERT_EQ(1, aes[1]->getTeamManager().getActiveAgentIds().size());
    aes[0]->start();
    aes[1]->start();
    // Let agent announce their presence
    aes[0]->getAlicaClock().sleep(getDiscoveryTimeout());
    // Process presence announcement
    step(aes[0]);
    step(aes[1]);
    ASSERT_EQ(2, aes[0]->getTeamManager().getActiveAgentIds().size());
    ASSERT_EQ(2, aes[1]->getTeamManager().getActiveAgentIds().size());

    int id = 8;
    const alica::Agent* hairydiscovered = aes[0]->getTeamManager().getAgentByID(aes[0]->getId(id));
    const alica::Agent* hairyoriginal = aes[1]->getTeamManager().getLocalAgent();
    verifyAgents(hairydiscovered, hairyoriginal);

    id = 9;
    const alica::Agent* nasediscovered = aes[1]->getTeamManager().getAgentByID(aes[1]->getId(id));
    const alica::Agent* naseoriginal = aes[0]->getTeamManager().getLocalAgent();
    verifyAgents(nasediscovered, naseoriginal);

    // Reject agent with mismatching plan hash
    id = 11;
    alica::AgentAnnouncement aa;
    aa.planHash = aes[0]->getVersion() + 1;
    aa.senderSdk = aes[0]->getVersion();
    aa.senderID = aes[0]->getId(id);
    aa.role = "Attacker";
    aa.senderName = "myo";
    aes[0]->editTeamManager().handleAgentAnnouncement(aa);
    step(aes[0]);
    const alica::Agent* myodiscovered = aes[0]->getTeamManager().getAgentByID(aes[0]->getId(id));
    ASSERT_EQ(myodiscovered, nullptr);
}
}
}